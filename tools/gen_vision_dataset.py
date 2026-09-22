#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""从 Gazebo 批量生成视觉数据集（默认**只出图**，供 X-AnyLabeling 手工标注）。

默认行为（2026-09-22 按使用习惯改的）
-------------------------------------
    * **只存图片**，不写 YOLO 框 —— 手工标注更可控（自动投影框有残差：
      传送位姿残差 + 灯箱受重力下沉约 3cm，1m 处≈35px）
    * 图片**平铺**在 `<out>/images/`，文件名带场景（如 `000004_A_north.jpg`），
      一个文件夹直接导进 X-AnyLabeling
    * 仍然写 `meta.jsonl`：每张图的位姿 / **灯态** / **车牌真值** / 物体清单
      —— 灯态和车牌字符串是"整图标签"，手工标注很难标，评估时要用
    * 想连自动框一起出：加 `--with-labels`（可用于和手工框对照）

类别建议（手工标注时按这个顺序, 与后续训练一致）:
    0 standee        社区人员立牌 c01..c08
    1 non_community  非社区人员 F1/F2
    2 traffic_light  红绿灯灯箱
    3 plate          车牌

原始说明（自动标注的原理，仍适用于 --with-labels）:


为什么投影标注是准的
------------------
场里每个道具的位姿都有唯一真值源（`standees.yaml` / `cars.yaml` /
`traffic_lights.yaml` + world 里的 include pose），相机模型也在
`robot_params.yaml` 里。所以只要知道车在哪，就能把每个物体的 3D 包围盒
**投影**成像素框 —— 标注 0 人工、0 误差。
（几何与相机数学直接复用 `gen_recognition_points.py`，不维护第二份真值。）

采什么
------
    --mode points   在 10 个识别点位附近抖动（= 真实工作包线，占多数）
    --mode random   可行驶区域内的随机位姿（提高泛化）
    --mode neg      画面里没有目标的空帧（抑制误检）

怎么采
------
用 `/gazebo/set_model_state` 把车**传送**到采样位姿 —— 不需要导航、不需要建图，
只要 `roslaunch competition_robot robot_gazebo.launch`（自带相机 + 红绿灯循环）。
传送后等"新的一帧图 + 真值位姿到位"再存，避免存到旧位姿的画面。

用法
----
    # 不连仿真，先看采样位姿和标注统计（几秒）
    python3 tools/gen_vision_dataset.py --n 1200 --dry-run

    # 真采（另开终端先把仿真起起来）
    roslaunch competition_robot robot_gazebo.launch gui:=false rviz:=false
    python3 tools/gen_vision_dataset.py --n 1200 --out datasets/vision

类别
----
    0 standee        社区人员立牌 (c01..c08)
    1 non_community  非社区人员立牌 (F1/F2)
    2 traffic_light  红绿灯灯箱
    3 plate          车牌（车牌字符串记在 meta.jsonl 里, 交给 HyperLPR3 识别）

产出
----
    datasets/vision/{images,labels}/{train,val}/...
    datasets/vision/data.yaml     YOLO 数据集描述
    datasets/vision/meta.jsonl    每张图: 位姿 / 物体 / 灯态 / 车牌 / 场景 id
"""
from __future__ import annotations

import argparse
import json
import math
import os
import random
import sys
import time

import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gen_recognition_points as G          # noqa: E402  几何/相机唯一真值源
import setup_cars as SC                     # noqa: E402  车牌几何
import gen_standees as GS                   # noqa: E402  立牌几何

CLASSES = ['standee', 'non_community', 'traffic_light', 'plate']
CAM_Z_MIN = 0.05          # 相机近裁剪面 (robot_params.yaml 的 clip_near)


# =============================================================================
#  物体 -> 3D 包围盒（角点, 世界系）
# =============================================================================
def _box_pts(cx, cy, yaw, hx, hy, z0, z1):
    c, s = math.cos(yaw), math.sin(yaw)
    out = []
    for dx in (+hx, -hx):
        for dy in (+hy, -hy):
            for z in (z0, z1):
                out.append([cx + dx * c - dy * s, cy + dx * s + dy * c, z])
    return out


def build_objects():
    """所有可标注物体。返回 [{cls, name, pts, extra}]"""
    objs = []

    # ---- 人偶立牌 ----
    # 宽度表来自 standees.yaml (G.load_standees), 摆放位姿来自 world 的 include
    # (G.load_world_props) —— 两处都是唯一真值源, 不重复写死。
    widths = G.load_standees()                 # {model: width_m}
    for p in G.load_world_props():
        uri = p['uri']
        if not uri.startswith('standee_'):
            continue
        w = float(widths.get(uri, 0.05))
        geom = G.standee_box(p['x'], p['y'], p['yaw'], w)
        objs.append(dict(cls='non_community' if '_F' in uri else 'standee',
                         name=p['name'], pts=geom['pts'],
                         extra=dict(model=uri, w=w)))

    # ---- 红绿灯灯箱 + 三颗灯珠 (灯珠位置是"读颜色"时要用到的关键点) ----
    for name, lt in G.load_lights().items():
        g = G.light_geom(lt)
        objs.append(dict(cls='traffic_light', name=name, pts=g['pts'],
                         extra=dict(lamps=[[l['color']] + list(l['p']) for l in g['lamps']])))

    # ---- 车牌 (唯一真值源: cars.yaml 的位姿 + setup_cars 的车牌局部偏置) ----
    for name, c in G.load_cars().items():
        # 车牌在车体局部的偏置 (与 setup_cars.write_car_model 一致)
        lx = SC.BOARD_T / 2.0 + SC.PLATE_T / 2.0 + SC.PLATE_CX
        lz = SC.BASE_T + SC.PLATE_CZ
        cx, cy, yaw = float(c['x']), float(c['y']), float(c.get('yaw', 0.0))
        cc, ss = math.cos(yaw), math.sin(yaw)
        px, py = cx + lx * cc, cy + lx * ss
        objs.append(dict(cls='plate', name=name,
                         pts=_box_pts(px, py, yaw, SC.PLATE_T / 2.0, SC.PLATE_W / 2.0,
                                      lz - SC.PLATE_H / 2.0, lz + SC.PLATE_H / 2.0),
                         extra=dict(plate=c.get('plate', ''))))
    return objs


# =============================================================================
#  投影: 3D 角点 -> 像素框
# =============================================================================
def project(pts, x, y, yaw, rob):
    """与 gen_recognition_points.evaluate 用**同一套**相机数学。

    约定: u = W/2 - fx*lateral/depth, v = H/2 - fx*vert/depth
    (left 在图像左侧 -> 列号更小)
    """
    p = np.asarray(pts, dtype=float)
    camx, camy = G.camera_xy(x, y, yaw, rob)
    camz = rob['mount'][2]
    fwd = np.array([math.cos(yaw), math.sin(yaw)])
    left = np.array([-math.sin(yaw), math.cos(yaw)])
    d = p[:, :2] - np.array([camx, camy])
    depth = d @ fwd
    lateral = d @ left
    vert = p[:, 2] - camz
    sd = np.maximum(depth, 1e-9)
    u = rob['W'] / 2.0 - rob['fx'] * lateral / sd
    v = rob['H'] / 2.0 - rob['fx'] * vert / sd
    return u, v, depth


def label_of(obj, x, y, yaw, rob, min_px=20.0, min_visible=0.6):
    """-> (cls, 框, 可见比例) 或 None"""
    u, v, depth = project(obj['pts'], x, y, yaw, rob)
    if (depth < CAM_Z_MIN).any():
        return None                      # 有角点在相机后面/裁剪面内 -> 不成框
    inside = ((u >= 0) & (u < rob['W']) & (v >= 0) & (v < rob['H']))
    vis = float(inside.mean())
    if vis < min_visible:
        return None                      # 大部分在画面外
    x0, y0, x1, y1 = float(u.min()), float(v.min()), float(u.max()), float(v.max())
    # 只保留画面内的部分 (YOLO 允许截断框, 但要去掉完全在外面的)
    x0, y0 = max(x0, 0.0), max(y0, 0.0)
    x1, y1 = min(x1, rob['W'] - 1), min(y1, rob['H'] - 1)
    if (x1 - x0) < min_px or (y1 - y0) < min_px:
        return None
    return obj['cls'], (x0, y0, x1, y1), vis


def yolo_line(cls, box, rob):
    x0, y0, x1, y1 = box
    return '%d %.6f %.6f %.6f %.6f' % (
        CLASSES.index(cls),
        (x0 + x1) / 2.0 / rob['W'], (y0 + y1) / 2.0 / rob['H'],
        (x1 - x0) / rob['W'], (y1 - y0) / rob['H'])


# =============================================================================
#  位姿采样
# =============================================================================
def load_points():
    """识别点位: (name, x, y, yaw_rad)。

    ★ recognition_points.yaml 里的 pose[2] **本来就是弧度** (tl_top 存的是 -3.1412),
      不要再 math.radians() 一次 —— 我试过, 标注统计立刻变差 (空帧 149->256)。
    """
    d = yaml.safe_load(open(os.path.join(G.ROBOT, 'config', 'recognition_points.yaml')))
    return [(p['name'], float(p['pose'][0]), float(p['pose'][1]),
             float(p['pose'][2])) for p in d['points']]


def in_road_free(x, y, rob, blocks):
    """车心在可行驶区域里, 且不压街区 (与点位工具同一套判据)"""
    if abs(x) > G.FIELD_LINE - rob['half_x'] or abs(y) > G.FIELD_LINE - rob['half_y']:
        return False
    # in_road 的入参是 (N,K,2): N 个位姿 x 每个 K 个采样点 —— 这里只有一个点
    if not bool(G.in_road(np.array([[[x, y]]], dtype=float))[0]):
        return False
    for _, bx, by, bhx, bhy in blocks:
        if abs(x - bx) < bhx + rob['half_x'] and abs(y - by) < bhy + rob['half_y']:
            return False
    return True


def sample_poses(n, rob, blocks, args):
    """-> [(scenario, mode, x, y, yaw)]"""
    rng = random.Random(args.seed)
    pts = [p for p in load_points() if p[0] in args.points.split(',')] if args.points else load_points()
    mix = dict((k, float(v)) for k, v in (kv.split(':') for kv in args.mode_mix.split(',')))
    out = []
    for mode, frac in mix.items():
        k = int(round(n * frac))
        if mode == 'points':
            for i in range(k):
                name, px, py, pyaw = pts[i % len(pts)]
                j = args.points_jitter
                out.append((name, mode,
                            px + rng.uniform(-j, j), py + rng.uniform(-j, j),
                            pyaw + math.radians(rng.uniform(-args.points_yaw, args.points_yaw))))
        else:
            got = 0
            guard = 0
            while got < k and guard < k * 400:
                guard += 1
                x = rng.uniform(-2.0, 2.0)
                y = rng.uniform(-2.0, 2.0)
                yaw = rng.uniform(-math.pi, math.pi)
                if not in_road_free(x, y, rob, blocks):
                    continue
                if mode == 'neg':
                    # 空帧: 画面里不能有目标
                    objs = build_objects()
                    if any(label_of(o, x, y, yaw, rob) for o in objs):
                        continue
                out.append(('random' if mode != 'neg' else 'neg', mode, x, y, yaw))
                got += 1
    rng.shuffle(out)
    return out


def split_train_val(poses, val_frac, val_points):
    """按**场景**切分, 而不是按帧随机切 —— 这样 val 量的是"换视角的泛化"。

    val_points 里的识别点位整体进 val; 随机/空帧按比例切。
    """
    val, tr = [], []
    for p in poses:
        scenario, mode = p[0], p[1]
        if scenario in val_points:
            val.append(p)
        elif mode in ('random', 'neg'):
            (val if random.random() < val_frac else tr).append(p)
        else:
            tr.append(p)
    return tr, val


# =============================================================================
#  采集 (连仿真)
# =============================================================================
def capture(args, poses, out_dir):
    import rospy
    from gazebo_msgs.msg import ModelState
    from gazebo_msgs.srv import SetModelState
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Image
    from std_msgs.msg import String

    rospy.init_node('gen_vision_dataset', anonymous=True, disable_signals=True)
    try:
        rospy.wait_for_service('/gazebo/set_model_state', timeout=20)
    except rospy.ROSException:
        print('找不到 /gazebo/set_model_state —— 先把仿真起起来:')
        print('  roslaunch competition_robot robot_gazebo.launch gui:=false rviz:=false')
        return 1
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    def on_img(m):
        st['img'] = m
        st['stamp'] = m.header.stamp          # ★ 用**图自己的时间戳**判新鲜度

    st = dict(img=None, stamp=rospy.Time(0), truth=None, light='?')

    def on_truth(m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        st['truth'] = (m.pose.pose.position.x, m.pose.pose.position.y, yaw)

    rospy.Subscriber('/camera/rgb/image_raw', Image, on_img, queue_size=1)
    rospy.Subscriber('/odom_groundtruth', Odometry, on_truth, queue_size=5)
    rospy.Subscriber('/traffic_light/state', String,
                     lambda m: st.update(light=m.data.strip().lower()), queue_size=5)

    import cv2
    from cv_bridge import CvBridge
    bridge = CvBridge()

    img_dir = os.path.join(out_dir, 'images')
    lbl_dir = os.path.join(out_dir, 'labels')
    subs = ('train', 'val') if args.split else ('',)
    for sub in subs:
        os.makedirs(os.path.join(img_dir, sub), exist_ok=True)
        if args.with_labels:
            os.makedirs(os.path.join(lbl_dir, sub), exist_ok=True)
    meta = open(os.path.join(out_dir, 'meta.jsonl'), 'w')
    objects = build_objects()

    print('开始采集 %d 张 -> %s   (%s)'
          % (len(poses), out_dir, '含 YOLO 框' if args.with_labels else '只出图, 平铺'))
    done = skipped = 0
    for idx, (scenario, mode, x, y, yaw) in enumerate(poses):
        # --- 传送 ---
        ms = ModelState()
        ms.model_name = args.robot_model
        ms.pose.position.x, ms.pose.position.y, ms.pose.position.z = x, y, 0.02
        ms.pose.orientation.z = math.sin(yaw / 2.0)
        ms.pose.orientation.w = math.cos(yaw / 2.0)
        ms.reference_frame = 'world'
        try:
            set_state(ms)
        except Exception as e:
            print('  传送失败: %s' % e)
            skipped += 1
            continue
        # --- 等"新的一帧 + 位姿到位" ---
        # ★ 两个坑 (2026-09-22 踩过):
        #   ① 不能拿"消息到达时间"当新鲜度 —— 相机有出图延迟, 传送后 0.1s 到达的帧
        #      可能是**传送前**渲染的。必须比**图自己的 header.stamp**。
        #   ② 标注要用**真值位姿**(含 yaw), 不要用指令位姿 —— 传送有误差时才对得上。
        t_cmd = rospy.Time.now()
        t_wall = time.time()
        ok = False
        while time.time() - t_wall < 6.0:
            rospy.sleep(0.05)
            if st['img'] is None or st['stamp'] <= t_cmd:
                continue
            tr = st['truth']
            if tr and math.hypot(tr[0] - x, tr[1] - y) < 0.06 \
                    and abs(float(G.wrap(tr[2] - yaw))) < math.radians(6.0):
                ok = True
                break
        if not ok:
            skipped += 1
            continue
        # 用真值位姿算标注 (比指令位姿更可信)
        lx, ly, lyaw = st['truth']

        frame = bridge.imgmsg_to_cv2(st['img'], desired_encoding='bgr8')
        labels = []
        for o in objects:
            r = label_of(o, lx, ly, lyaw, ROBOT, args.min_box_px, args.min_visible)
            if r:
                labels.append((r[0], r[1], o))
        # 空帧检测 (neg 模式要求没有目标; 采不到就丢掉这一帧)
        if mode == 'neg' and labels:
            skipped += 1
            continue
        if mode != 'neg' and not labels:
            skipped += 1
            continue

        sub = ('val' if (scenario in args.val_points_set) else 'train') if args.split else ''
        fn = '%06d_%s' % (idx, scenario)
        ext = args.img_ext
        cv2.imwrite(os.path.join(img_dir, sub, fn + '.' + ext), frame,
                    [cv2.IMWRITE_JPEG_QUALITY, args.jpeg_quality] if ext == 'jpg' else [])
        if args.with_labels:
            with open(os.path.join(lbl_dir, sub, fn + '.txt'), 'w') as f:
                for cls, box, _o in labels:
                    f.write(yolo_line(cls, box, ROBOT) + '\n')
        meta.write(json.dumps(dict(
            file=('images/%s/%s.%s' % (sub, fn, ext)) if sub else ('images/%s.%s' % (fn, ext)),
            split=sub or '-', scenario=scenario, mode=mode,
            pose=[round(x, 4), round(y, 4), round(yaw, 4)],
            pose_truth=[round(lx, 4), round(ly, 4), round(lyaw, 4)],
            light=st['light'],
            objects=[dict(cls=c, box=[round(v, 1) for v in b], name=o['name'],
                          **{k: v for k, v in o['extra'].items() if k != 'lamps'})
                     for c, b, o in labels],
            lamps=[dict(name=o['name'], lamps=o['extra']['lamps'])
                   for c, b, o in labels if c == 'traffic_light'],
        ), ensure_ascii=False) + '\n')
        done += 1
        if done % 25 == 0:
            print('  %d/%d (跳过 %d)' % (done, len(poses), skipped))
    meta.close()
    if args.with_labels:
        write_data_yaml(out_dir)
    print('完成: 存了 %d 张, 跳过 %d 张 -> %s' % (done, skipped, out_dir))
    return 0


def write_data_yaml(out_dir):
    p = os.path.join(out_dir, 'data.yaml')
    with open(p, 'w') as f:
        f.write('# YOLO 数据集 (由 tools/gen_vision_dataset.py 合成生成)\n')
        f.write('path: %s\n' % os.path.abspath(out_dir))
        f.write('train: images/train\nval: images/val\n')
        f.write('nc: %d\nnames: %s\n' % (len(CLASSES), CLASSES))
    return p


# =============================================================================
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--n', type=int, default=200,
                    help='目标图片数 (固定场景 200 张就够; 要练大模型再加)')
    ap.add_argument('--out', default='datasets/vision')
    ap.add_argument('--mode-mix', default='points:0.6,random:0.3,neg:0.1')
    ap.add_argument('--points', default='', help='只用这些识别点位 (逗号分隔), 空=全部')
    ap.add_argument('--points-jitter', type=float, default=0.30, help='点位抖动 (m)')
    ap.add_argument('--points-yaw', type=float, default=25.0, help='点位朝向抖动 (deg)')
    ap.add_argument('--min-box-px', type=float, default=20.0)
    ap.add_argument('--min-visible', type=float, default=0.6, help='框内角点比例下限')
    ap.add_argument('--val-frac', type=float, default=0.15)
    ap.add_argument('--val-points', default='tl_top,tl_bot',
                    help='这些识别点位的图整份进 val, 量"换视角"的泛化 '
                         '(名字取自 recognition_points.yaml, 如 tl_top / A_north)')
    ap.add_argument('--img-ext', default='jpg', choices=['jpg', 'png'])
    ap.add_argument('--jpeg-quality', type=int, default=92)
    ap.add_argument('--robot-model', default='competition_robot')
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--with-labels', action='store_true',
                    help='同时写 YOLO 框 (默认只出图; 自动框有残差, 一般不需要)')
    ap.add_argument('--split', action='store_true',
                    help='按 train/val 分子目录存 (默认平铺, 方便导进标注工具)')
    ap.add_argument('--dry-run', action='store_true', help='不连仿真, 只算位姿/标注并打印统计')
    args = ap.parse_args()

    global ROBOT
    ROBOT = G.load_robot()
    blocks = G.load_blocks()
    poses = sample_poses(args.n, ROBOT, blocks, args)
    args.val_points_set = set(x.strip() for x in args.val_points.split(',') if x.strip())
    tr, val = split_train_val(poses, args.val_frac, args.val_points_set)

    if args.dry_run:
        objects = build_objects()
        print('相机 %dx%d  fx=%.1f  安装 z=%.3f  近裁剪 %.2f m'
              % (ROBOT['W'], ROBOT['H'], ROBOT['fx'], ROBOT['mount'][2], CAM_Z_MIN))
        print('物体 %d 个: %s' % (len(objects), ', '.join(
            '%s=%d' % (c, sum(1 for o in objects if o['cls'] == c)) for c in CLASSES)))
        print('采样 %d 个位姿 -> train %d / val %d' % (len(poses), len(tr), len(val)))
        cnt = {c: 0 for c in CLASSES}
        sizes = {c: [] for c in CLASSES}
        miss = 0
        for scenario, mode, x, y, yaw in poses[:400]:
            ls = [label_of(o, x, y, yaw, ROBOT, args.min_box_px, args.min_visible)
                  for o in objects]
            ls = [l for l in ls if l]
            if not ls:
                miss += 1
            for cls, box, _v in ls:
                cnt[cls] += 1
                sizes[cls].append(max(box[2] - box[0], box[3] - box[1]))
        k = min(len(poses), 400)
        print('\n前 %d 个位姿的（投影）标注统计 —— 只作参考, 手工标注以实际为准:' % k)
        for c in CLASSES:
            s = sizes[c]
            print('  %-14s %5d 个框   中位边长 %s px' % (
                c, cnt[c], ('%.0f' % np.median(s)) if s else '-'))
        print('  空帧 %d/%d' % (miss, k))
        return 0

    os.makedirs(args.out, exist_ok=True)
    return capture(args, tr + val, args.out)


if __name__ == '__main__':
    sys.exit(main())
