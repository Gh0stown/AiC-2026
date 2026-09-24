#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""视觉识别节点 —— 订阅相机, 识别三类目标, **终端打印 + 弹出带框结果图 + 每次运行独立存档**。

比赛方要的标准用法（三个终端, 只用 roslaunch / rosrun）
--------------------------------------------------------
    # ① 世界 + 定位 + 导航
    roslaunch competition_robot navigation.launch
    # ② 识别节点（本文件）—— 会弹出一个窗口显示带 YOLO 框的结果图
    rosrun competition_robot vision_detect.py
    # ③ 巡检路线（到识别点位会自动请求识别）
    rosrun competition_robot patrol.py --file $(rospack find competition_robot)/config/recognition_route.yaml

话题
----
    订阅  /camera/rgb/image_raw     图像
    订阅  /vision/request           (std_msgs/String, JSON) 请求识别一次
                                    {"point":"A_north", "kind":"standee", "index":2}
    订阅  /vision/summary           打印 + 落盘累计汇总（空消息即可）
    发布  /vision/result            (std_msgs/String, JSON) 本次结果

每次运行独立存档
----------------
    vision_runs/<时间戳>[_标签]/
        images/0001_A_north.jpg ...   带识别框的结果图（每张也同时弹窗显示）
        summary.txt                   终端那份汇总的文本
        results.json                  全部逐点结果 + 汇总（机器可读）
    总文件夹可用 --out-root 改；同名再跑不会覆盖（不同时间戳）。

★ 三个任务的口径（跟模型一致, 别搞混）
    * 人偶立牌：模型检**整块立牌**, 2 类（社区 / 非社区人员）
    * 红绿灯：模型**直接检"亮着的那颗灯珠"**（3 类 red/yellow/green_light），不是灯箱；
              框的类别就是灯态
    * 车牌：YOLO 框车牌 -> 裁剪 -> HyperLPR3 读字符

★ 归属（为什么不能直接数画面里的框）
    在 A_north 点位, 画面里往往同时能看到 A_south 的立牌。所以本节点用当前位姿把
    "该点位对应的那一组"投影出来, 只保留落在该组框里的检测 —— 每个点位只算自己那组,
    汇总才不会重复计数。
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import threading
import time


def _ensure_venv():
    """★ rosrun 用的是**系统 python3**, 里面没有 ultralytics / hyperlpr3。

    那样三个模型会全部加载失败, 而"缺模型不崩"的设计会让它**静默报 0 个** ——
    比赛时会以为"这儿没人"。所以: 如果仓库里有 .venv 且当前解释器不是它, 就自己换过去重跑。
    找不到 venv 也不崩, 后面会打印明确的提示。
    """
    try:
        import ultralytics                                # noqa: F401
        return
    except Exception:                                     # noqa: BLE001
        pass
    d = os.path.dirname(os.path.abspath(__file__))
    for _ in range(8):
        cand = os.path.join(d, '.venv', 'bin', 'python')
        if os.path.isfile(cand) and os.path.abspath(cand) != os.path.abspath(sys.executable):
            print('[vision] 当前解释器没有 ultralytics, 切到 %s 重跑' % cand, flush=True)
            os.execv(cand, [cand] + sys.argv)             # 环境变量(ROS_*)自动继承
        nd = os.path.dirname(d)
        if nd == d:
            break
        d = nd


_ensure_venv()

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String

try:
    from cv_bridge import CvBridge
except ImportError:                                   # 没有 cv_bridge 时也能静态检查
    CvBridge = None

# ---- tools/ 里的实现是唯一真值源, 这里不重写 ----
_HERE = os.path.dirname(os.path.abspath(__file__))


def _find_ws(start):
    """往上找到含 tools/vision_infer.py 的那层 = 仓库根。

    ★ 不要用"往上数 N 层": 从源码跑是 <ws>/src/<pkg>/scripts,
      而 catkin 装完是 <ws>/devel/lib/<pkg> —— 两种深度不同, 数层必错一个。
      (原来写死 2 层 -> 算成 <ws>/src -> import vision_infer 直接 ModuleNotFoundError)
    """
    d = start
    for _ in range(8):
        if os.path.isfile(os.path.join(d, 'tools', 'vision_infer.py')):
            return d
        nd = os.path.dirname(d)
        if nd == d:
            break
        d = nd
    return os.path.dirname(os.path.dirname(start))     # 兜底


_WS = _find_ws(_HERE)                                  # <仓库根> = catkin 工作区根
sys.path.insert(0, os.path.join(_WS, 'tools'))

import vision_infer as VI                             # noqa: E402
import gen_recognition_points as G                    # noqa: E402

# 画框配色 (BGR)
COL = {'community': (80, 200, 255), 'non_community': (0, 165, 255),
       'light': (90, 240, 90), 'plate': (255, 150, 80)}
CN = {'community': '社区人员', 'non_community': '非社区人员'}


def log(msg=''):
    print(msg, flush=True)


_FONT_WARNED = [False]


def draw_texts(img_bgr, items):
    """用 PIL 画文字 —— **cv2.putText 只支持 ASCII, 中文会变成 `?`**（踩过）。

    items: [(text, (x, y), (b, g, r), size_px), ...]
    字体走 tools/cnfont.py（Noto CJK -> 文泉驿 -> ... -> 找不到就退英文标签）。
    """
    if not items:
        return img_bgr
    try:
        from PIL import Image, ImageDraw
        from cnfont import load as _load_font
    except Exception:                                     # noqa: BLE001
        return img_bgr
    pil = Image.fromarray(cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB))
    d = ImageDraw.Draw(pil)
    for text, (x, y), (b, g, r), size in items:
        f, cjk_ok = _load_font(int(size))
        if not cjk_ok and not _FONT_WARNED[0]:
            _FONT_WARNED[0] = True
            log('[vision] ⚠ 没找到中文字体, 图上的中文会变成方框 —— '
                '装一下: sudo apt install -y fonts-noto-cjk')
        # 描边提高可读性（先画黑边再画本色）
        for dx, dy in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            d.text((x + dx, y + dy), text, font=f, fill=(0, 0, 0))
        d.text((x, y), text, font=f, fill=(r, g, b))
    return cv2.cvtColor(np.array(pil), cv2.COLOR_RGB2BGR)


class VisionDetect(object):
    def __init__(self, a):
        self.a = a
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.img = None
        self.pose = None
        self.results = []                     # 逐点结果 (落盘用)
        self.blocks = {'A': [0, 0], 'B': [0, 0]}
        self.point_standee = {}          # 点位 -> (社区数, 非社区数), 只留最新一次
        self.lights = {}
        self.plates = {}
        self.n_saved = 0
        self.shown = None
        self.stop_show = False

        self.run_dir = make_run_dir(a.out_root, a.tag)
        self.img_dir = os.path.join(self.run_dir, 'images')
        os.makedirs(self.img_dir, exist_ok=True)

        self.objs = self._load_geometry()
        self._tf = None
        if a.attribute:
            try:
                import tf2_ros
                self._tf = tf2_ros.Buffer()
                self._tf_listener = tf2_ros.TransformListener(self._tf)
            except Exception as e:                          # noqa: BLE001
                log('[vision] TF 不可用, 改为不按组归属: %s' % e)

        av = VI.available(a.models_dir)
        log('视觉识别节点已启动')
        log('  模型: ' + '  '.join('%s=%s' % (k, '有' if v else '缺') for k, v in av.items()))
        for k, v in av.items():
            if not v:
                log('  ⚠ %s 的模型缺失, 这一项会输出"没识别到"' % k)
        log('  相机 %s   请求 /vision/request   汇总 /vision/summary' % a.image_topic)
        log('  本次结果存到: %s' % self.run_dir)
        if not a.show:
            log('  （--no-show: 不弹窗）')

        rospy.Subscriber(a.image_topic, Image, self.on_img, queue_size=1)
        rospy.Subscriber('/vision/request', String, self.on_request, queue_size=5)
        rospy.Subscriber('/vision/summary', String, lambda m: self.finish(), queue_size=2)
        self.pub = rospy.Publisher('/vision/result', String, queue_size=5)

        if a.show:
            threading.Thread(target=self._show_loop, daemon=True).start()
        rospy.on_shutdown(self.finish)

    # ---------------------------------------------------------------- 几何
    def _load_geometry(self):
        try:
            out = {}
            widths = G.load_standees()
            for p in G.load_world_props():
                uri = p['uri']
                if uri.startswith('standee_'):
                    grp = p['name'].rsplit('_', 1)[0]        # A_north_1 -> A_north
                    g = G.standee_box(p['x'], p['y'], p['yaw'],
                                      float(widths.get(uri, 0.05)))
                    out.setdefault(grp, []).append(
                        dict(cls='non_community' if '_F' in uri else 'community',
                             pts=g['pts']))
            return out
        except Exception as e:                              # noqa: BLE001
            log('[vision] 载入立牌几何失败（退化为"数画面里所有框"）: %s' % e)
            return {}

    def cur_pose(self):
        if self._tf is not None:
            try:
                tr = self._tf.lookup_transform('map', 'base_footprint',
                                               rospy.Time(0), rospy.Duration(0.05))
                t, q = tr.transform.translation, tr.transform.rotation
                yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                return (t.x, t.y, yaw)
            except Exception:                               # noqa: BLE001
                pass
        return self.pose

    @staticmethod
    def _project(pts, pose, rob):
        p = np.asarray(pts, dtype=float)
        camx, camy = G.camera_xy(pose[0], pose[1], pose[2], rob)
        camz = rob['mount'][2]
        fwd = np.array([math.cos(pose[2]), math.sin(pose[2])])
        left = np.array([-math.sin(pose[2]), math.cos(pose[2])])
        d = p[:, :2] - np.array([camx, camy])
        depth, lateral = d @ fwd, d @ left
        vert = p[:, 2] - camz
        sd = np.maximum(depth, 1e-9)
        return (rob['W'] / 2.0 - rob['fx'] * lateral / sd,
                rob['H'] / 2.0 - rob['fx'] * vert / sd, depth)

    def _expected_boxes(self, point, pose):
        if not pose or point not in self.objs:
            return None
        rob = G.load_robot()
        out = []
        for o in self.objs[point]:
            u, v, dep = self._project(o['pts'], pose, rob)
            if (dep < 0.05).any():
                continue
            out.append((o['cls'], [u.min(), v.min(), u.max(), v.max()]))
        return out or None

    # ---------------------------------------------------------------- 回调
    def on_img(self, m):
        try:
            img = self.bridge.imgmsg_to_cv2(m, 'bgr8')
        except Exception:                                   # noqa: BLE001
            return
        with self.lock:
            self.img = img

    def grab(self, n):
        out = []
        for _ in range(max(1, n)):
            with self.lock:
                if self.img is not None:
                    out.append(self.img.copy())
            rospy.sleep(self.a.frame_gap)
        return out

    def on_request(self, m):
        try:
            req = json.loads(m.data) if m.data.strip() else {}
        except Exception:                                   # noqa: BLE001
            req = {'point': m.data.strip()}
        point = str(req.get('point', '?')).strip()
        kind = str(req.get('kind', 'all')).strip()
        idx = req.get('index')
        frames = self.grab(self.a.frames)
        if not frames:
            log('[vision] 没拿到图像（相机没数据?）')
            return
        res = self.recognize(frames, point, kind)
        res.update(point=point, index=idx, kind=kind,
                   stamp=time.strftime('%Y-%m-%d %H:%M:%S'))
        self.results.append(res)

        log('[识别] ── %s%s' % (point, ('（第%s站）' % idx) if idx else ''))
        for ln in res['lines']:
            log('         ' + ln)

        # 画框 + 存图 + 弹窗
        img = self.annotate(frames[-1], res, point, idx)
        fn = '%04d_%s.jpg' % (self.n_saved + 1, point)
        path = os.path.join(self.img_dir, fn)
        try:
            cv2.imwrite(path, img)
            self.n_saved += 1
            res['image'] = os.path.join(os.path.basename(self.run_dir), 'images', fn)
            with self.lock:
                self.shown = img
        except Exception as e:                              # noqa: BLE001
            log('  ⚠ 存图失败: %s' % e)
        self.pub.publish(String(data=json.dumps(res, ensure_ascii=False)))

    # ---------------------------------------------------------------- 识别
    def recognize(self, frames, point, kind):
        pose = self.cur_pose()
        lines, out, boxes = [], {}, []
        if kind in ('all', 'standee'):
            ds = []
            for f in frames:
                ds += VI.read_standees(f, conf=self.a.conf, models_dir=self.a.models_dir)
            ds = dedup(ds)
            exp = self._expected_boxes(point, pose) if self.a.attribute else None
            kept, extra = ds, 0
            if exp:
                kept = [d for d in ds if hits_any(d['box'], exp)]
                extra = len(ds) - len(kept)
            tot, c, n = VI.count_standees(kept)
            out['standee'] = dict(total=tot, community=c, non_community=n)
            boxes += [dict(cls=d['cls'], box=d['box'], conf=d['conf'],
                           label=CN.get(d['cls'], d['cls'])) for d in kept]
            line = '人偶 %d 个：社区 %d / 非社区 %d' % (tot, c, n)
            if extra:
                line += '        （画面里另有 %d 个邻居, 已按组归属排除）' % extra
            if not exp and self.a.attribute:
                line += '        ⚠ 没拿到位姿, 未按组归属'
            lines.append(line)
            # ★ 每个点位只保留**最新一次**结果, 再由点位重算街区合计。
            #   原来直接 += 累加: 同一个点位被请求两次(重访/重试)就会翻倍
            #   (实测 A_north 请求两次 -> 10 个变 20 个)。
            self.point_standee[point] = (c, n)
            blk = point[:1].upper()
            if blk in ('A', 'B'):                          # 注意别写成 in 'AB': 空 point 会命中
                tt = [0, 0]
                for _p, (_c, _n) in self.point_standee.items():
                    if _p[:1].upper() == blk:
                        tt[0] += _c
                        tt[1] += _n
                self.blocks.setdefault(blk, [0, 0])
                self.blocks[blk][0], self.blocks[blk][1] = tt
        if kind in ('all', 'light'):
            st, cf, box = VI.read_light(frames[-1], conf=0.25, models_dir=self.a.models_dir)
            if st == 'none':
                for f in frames[::-1]:
                    st, cf, box = VI.read_light(f, conf=0.25, models_dir=self.a.models_dir)
                    if st != 'none':
                        break
            out['light'] = dict(state=st, conf=cf)
            self.lights[point] = st
            if box:
                boxes.append(dict(cls='light', box=box, conf=cf,
                                  label=VI.LIGHT_CN.get(st, st)))
            lines.append('红绿灯: %s（直接检灯珠, conf %.2f）'
                         % (VI.LIGHT_CN.get(st, st), cf))
        if kind in ('all', 'plate'):
            # ★ 多帧投票, 不是取"最自信的那一次" —— 漏字的那次往往更自信（见 vote_plate 注释）
            reads = []
            for f in frames:
                tx, cf, box = VI.read_plate(f, conf=0.25, margin=self.a.plate_margin,
                                            models_dir=self.a.models_dir)
                if tx:
                    reads.append((tx, cf, box))
            text, pconf, bbox, votes, nread = vote_plate(reads, self.a.plate_len)
            out['plate'] = dict(text=text, conf=pconf, votes=votes,
                                n_frames=len(frames), n_read=nread,
                                candidates=[t for t, _, _ in reads])
            self.plates[point] = text
            if bbox:
                boxes.append(dict(cls='plate', box=bbox, conf=pconf, label=text or '车牌'))
            if text:
                line = '车牌: %s（conf %.2f' % (text, pconf)
                if len(frames) > 1:
                    line += ', %d/%d 帧读到, 票 %d' % (nread, len(frames), votes)
                line += '）'
            else:
                line = '车牌: 没读到'
            lines.append(line)
        out['lines'] = lines
        out['boxes'] = boxes
        return out

    # ---------------------------------------------------------------- 画框
    def annotate(self, img, res, point, idx):
        """画框 + 顶部标题 + 结果行。框用 cv2, 文字用 PIL（要显示中文）。"""
        im = img.copy()
        texts = []
        for b in res.get('boxes', []):
            x0, y0, x1, y1 = [int(round(v)) for v in b['box']]
            col = COL.get(b['cls'], (255, 255, 255))
            cv2.rectangle(im, (x0, y0), (x1, y1), col, 3)
            texts.append(('%s %.2f' % (b.get('label', b['cls']), b['conf']),
                          (x0, max(6, y0 - 30)), col, 24))
        # 顶部黑条 + 标题 / 结果行
        n_line = len(res.get('lines', []))
        band = 52 + n_line * 34
        cv2.rectangle(im, (0, 0), (im.shape[1], band), (0, 0, 0), -1)
        texts.append(('%s%s' % (point, ('  第%s站' % idx) if idx else ''),
                      (14, 12), (60, 220, 255), 30))
        for i, ln in enumerate(res.get('lines', [])):
            texts.append((ln, (14, 52 + i * 34), (235, 235, 235), 24))
        return draw_texts(im, texts)

    def _show_loop(self):
        """弹窗显示（单独线程, 不在 ROS 回调里 imshow）"""
        name = 'vision_detect (q 退出)'
        try:
            cv2.namedWindow(name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(name, 960, 720)
        except Exception as e:                              # noqa: BLE001
            log('[vision] 打不开显示窗口（无 DISPLAY?）: %s' % e)
            return
        while not self.stop_show and not rospy.is_shutdown():
            with self.lock:
                im = None if self.shown is None else self.shown.copy()
            if im is not None:
                try:
                    cv2.imshow(name, im)
                    if cv2.waitKey(30) & 0xFF == ord('q'):
                        break
                except Exception:                           # noqa: BLE001
                    break
            else:
                time.sleep(0.1)
        try:
            cv2.destroyAllWindows()
        except Exception:                                   # noqa: BLE001
            pass

    # ---------------------------------------------------------------- 汇总
    def summary_lines(self):
        out = ['═' * 16 + ' 识别汇总 ' + '═' * 16]
        for b in sorted(self.blocks):
            c, n = self.blocks[b]
            out.append('  街区 %s: 共 %d 人   社区 %d / 非社区 %d' % (b, c + n, c, n))
        if self.lights:
            out.append('  红绿灯: ' + '   '.join('%s=%s' % (k, VI.LIGHT_CN.get(v, v))
                                               for k, v in sorted(self.lights.items())))
        if self.plates:
            out.append('  车牌:   ' + '   '.join('%s=%s' % (k, v or '(没读到)')
                                               for k, v in sorted(self.plates.items())))
        out.append('─' * 44)
        return out

    def finish(self):
        """打印汇总 + 落盘（summary.txt / results.json）。可重复调用。"""
        try:
            lines = self.summary_lines()
            log('')
            for ln in lines:
                log(ln)
            with open(os.path.join(self.run_dir, 'summary.txt'), 'w') as f:
                f.write('\n'.join(lines) + '\n')
            with open(os.path.join(self.run_dir, 'results.json'), 'w') as f:
                json.dump(dict(blocks=self.blocks, lights=self.lights,
                               plates=self.plates, points=self.results),
                          f, ensure_ascii=False, indent=2)
            log('  结果已存: %s（%d 张带框图）' % (self.run_dir, self.n_saved))
            self.pub.publish(String(data=json.dumps(
                dict(kind='summary', blocks=self.blocks, lights=self.lights,
                     plates=self.plates, run_dir=self.run_dir), ensure_ascii=False)))
        except Exception as e:                              # noqa: BLE001
            log('[vision] 汇总落盘失败: %s' % e)
        self.stop_show = True


# =============================================================================
def make_run_dir(root, tag=None):
    ts = time.strftime('%Y-%m-%d_%H%M%S')
    d = os.path.join(root, ts + (('_' + tag) if tag else ''))
    os.makedirs(d, exist_ok=True)
    return d


def iou(a, b):
    ix0, iy0 = max(a[0], b[0]), max(a[1], b[1])
    ix1, iy1 = min(a[2], b[2]), min(a[3], b[3])
    inter = max(0.0, ix1 - ix0) * max(0.0, iy1 - iy0)
    ua = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / ua if ua > 0 else 0.0


def dedup(dets, thr=0.5):
    """同一目标在多帧里会被检出多次 —— 按 IoU 去重, 置信度取最大。"""
    keep = []
    for d in sorted(dets, key=lambda x: -x['conf']):
        if all(iou(d['box'], k['box']) < thr for k in keep):
            keep.append(d)
    return keep


def norm_plate(t):
    """车牌归一化: 去掉分隔点/横线/空格, 转大写。用于投票时把"同一个车牌"归到一组。"""
    return ''.join(ch for ch in (t or '').upper() if ch not in '·-—. ')


def vote_plate(readings, want_len=7):
    """多帧投票 -> (text, conf, box, votes, n_read)

    ★ 为什么不能只取"置信度最高的那一次":
      这个车牌 OCR 的典型错误是**漏一位**(实测 苏A·PL12A -> 苏APL2A, 置信度还有 0.93),
      也就是**错的往往比对的更自信** —— 取 max(conf) 正好会挑中错的那个。
      所以: 先按归一化字符串投票; 再把**长度符合中文车牌(7 位)**的候选排在前面;
      同票再看置信度。

    readings: [(text, conf, box), ...]  (只放"读到了"的帧)
    want_len: 期望位数, 0 = 不按长度优先
    """
    if not readings:
        return ('', 0.0, None, 0, 0)
    grp = {}
    for t, cf, box in readings:
        g = grp.setdefault(norm_plate(t), dict(text=t, n=0, conf=0.0, box=box))
        g['n'] += 1
        if cf > g['conf']:                       # 这一组里置信度最高的那次作为代表
            g['conf'], g['text'], g['box'] = cf, t, box

    def rank(g):
        ln = len(norm_plate(g['text']))
        return (1 if (want_len and ln == want_len) else 0, g['n'], g['conf'])

    best = max(grp.values(), key=rank)
    return (best['text'], best['conf'], best['box'], best['n'], len(readings))


def hits_any(box, expected, thr=0.25):
    """检测框是否落在"该点位那一组"的任意预期框里（按组归属, 避免把邻居算进来）。"""
    cx, cy = (box[0] + box[2]) / 2.0, (box[1] + box[3]) / 2.0
    for _cls, eb in expected:
        if iou(box, eb) >= thr:
            return True
        if eb[0] <= cx <= eb[2] and eb[1] <= cy <= eb[3]:
            return True
    return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--image-topic', default='/camera/rgb/image_raw')
    ap.add_argument('--models-dir', default=None, help='权重目录 (默认 <仓库根>/weights)')
    ap.add_argument('--out-root', default=None,
                    help='结果总文件夹 (默认 <仓库根>/vision_runs); 每次运行一个子文件夹')
    ap.add_argument('--tag', default=None, help='给本次运行的文件夹加个后缀')
    ap.add_argument('--conf', type=float, default=0.3, help='立牌检测置信度阈值')
    ap.add_argument('--plate-margin', type=float, default=10,
                    help='车牌裁剪外扩 px（纯识别网络偏好紧裁剪, 10 左右合适）')
    ap.add_argument('--plate-len', type=int, default=7,
                    help='中文车牌位数; 多帧投票时优先选这个长度的候选 (0=不按长度优先)')
    ap.add_argument('--frames', type=int, default=3, help='每次请求取几帧做去重合并')
    ap.add_argument('--frame-gap', type=float, default=0.15)
    ap.add_argument('--show', dest='show', action='store_true', default=True,
                    help='弹出带识别框的结果图（默认开）')
    ap.add_argument('--no-show', dest='show', action='store_false')
    ap.add_argument('--attribute', dest='attribute', action='store_true', default=True,
                    help='按点位对应的那一组归属（默认开, 避免把邻居算进来）')
    ap.add_argument('--no-attribute', dest='attribute', action='store_false')
    a = ap.parse_args()
    if a.out_root is None:
        a.out_root = os.path.join(_WS, 'vision_runs')

    rospy.init_node('vision_detect', anonymous=False)
    VisionDetect(a)
    rospy.spin()
    return 0


if __name__ == '__main__':
    sys.exit(main())
