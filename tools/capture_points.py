#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""到识别点位拍照 —— 每个识别点都拍, 每次跑存一个独立文件夹。

跑法和 patrol.py 一致: 原地转向 -> move_base 直线过去 -> 到点原地转到拍照朝向,
然后**停下等画面稳定 -> 抓 N 帧 -> 存盘**。航点(waypoints)只作为路过的引导点,
不停车拍照(它们的作用是让规划器沿环线走, 不会从街区中间抄近路)。

输出结构 (每次运行一个文件夹):
    captures/
      2026-09-21_211530/            <- 本次运行
        run.yaml                    <- 本次元信息(路线/相机/帧数/git commit)
        index.csv                   <- 每个点: 序号/目标/位姿/文件名/真值/用时
        01_tl_top_f0.png            <- 目标名 + 帧号
        01_tl_top_f1.png
        02_A_north_f0.png
        ...

真值也一起存下来, 方便后面验收识别节点:
    * 车牌   -> 车牌字符串 (来自 config/cars.yaml)
    * 人偶   -> 该方向的人偶模型名 + 个数 (来自 world/config)
    * 红绿灯 -> 拍照瞬间 /traffic_light/state 的状态

用法:
    python3 tools/capture_points.py                       # 存到 captures/<时间戳>/
    python3 tools/capture_points.py --tag night           # 文件夹名加后缀
    python3 tools/capture_points.py --frames 5            # 每个点 5 帧
    python3 tools/capture_points.py --only tl_top,car_1   # 只拍指定点
    python3 tools/capture_points.py --format jpg          # 存 jpg (省空间)
    python3 tools/capture_points.py --no-drive            # 不移动, 只抓当前画面(调试)
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import subprocess
import sys
import time

import cv2
import rospy
import yaml
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from patrol import Patrol, load as load_route        # noqa: E402

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_ROUTE = os.path.join(WS, 'src', 'competition_robot', 'config', 'recognition_route.yaml')
CAPTURES = os.path.join(WS, 'captures')
ARENA_CFG = os.path.join(WS, 'src', 'competition_arena', 'config')
CAM_TOPIC = '/camera/rgb/image_raw'
LIGHT_TOPIC = '/traffic_light/state'


def git_head():
    try:
        return subprocess.check_output(['git', '-C', WS, 'rev-parse', '--short', 'HEAD'],
                                       stderr=subprocess.DEVNULL).decode().strip()
    except Exception:
        return '?'


def load_ground_truth():
    """每个识别点的期望结果 (验收识别节点时当答案用)"""
    gt = {}
    try:
        cars = yaml.safe_load(open(os.path.join(ARENA_CFG, 'cars.yaml')))['cars']
        for c in cars:
            gt[c['model']] = dict(kind='plate', expect=c['plate'], note=c.get('parking', ''))
    except Exception as e:
        rospy.logwarn('读 cars.yaml 失败: %s' % e)
    try:
        s = yaml.safe_load(open(os.path.join(ARENA_CFG, 'standees.yaml')))
        dim = {x['model']: x for x in s['standees']}
        blocks = s.get('blocks', [])
        for b in blocks:
            for e in b.get('edges', []):
                name = '%s_%s' % (b['name'], e['edge'])
                people = e.get('people', [])
                non = [p for p in people if dim.get(p, {}).get('category') == 'non_community']
                gt[name] = dict(kind='standee', expect=len(people),
                                note='%s (含非社区 %d)' % ('+'.join(people), len(non)))
    except Exception as e:
        rospy.logwarn('读 standees.yaml 失败: %s' % e)
    for nm in ('tl_top', 'tl_bot'):
        gt[nm] = dict(kind='traffic_light', expect='', note='状态在 index.csv 的 light_state 列')
    return gt


class Capture(Patrol):
    """继承 patrol 的走法, 加"到点拍照"。"""

    def __init__(self, a, outdir):
        Patrol.__init__(self, a)
        self.bridge = CvBridge()
        self.img = None
        self.light = ''
        self.outdir = outdir
        self.camera_topic = a.camera
        self._sub = None
        # ★ 相机**按需订阅**: Gazebo 的相机在没有订阅者时不渲染, 1280x960 一旦
        #   常开渲染会把仿真拖慢好几倍 (本沙箱 RTF 0.66 -> 0.28)。所以只在
        #   拍照前后这几秒订阅, 拍完立刻退订。
        rospy.Subscriber(LIGHT_TOPIC, String, self.cb_light, queue_size=2)

    def cam_on(self):
        if self._sub is None:
            self.img = None
            self._sub = rospy.Subscriber(self.camera_topic, Image, self.cb_img, queue_size=2)

    def cam_off(self):
        if self._sub is not None:
            self._sub.unregister()
            self._sub = None

    def cb_img(self, m):
        try:
            self.img = self.bridge.imgmsg_to_cv2(m, 'bgr8')
        except Exception as e:
            rospy.logwarn_throttle(5.0, '图像转换失败: %s' % e)

    def cb_light(self, m):
        self.light = m.data

    def wait_camera(self, timeout=25.0):
        """临时订阅相机, 拿到一帧就返回 (顺便验证话题通)"""
        self.cam_on()
        t0 = time.time()
        while not rospy.is_shutdown() and self.img is None:
            if time.time() - t0 > timeout:
                self.cam_off()
                return False
            rospy.sleep(0.2)
        self.cam_off()
        return True

    def shoot(self, stem, frames, settle, fmt, jpeg_quality):
        """等画面稳定后抓 frames 帧, 返回文件名列表 (只在抓帧期间订阅相机)"""
        rospy.sleep(settle)
        self.cam_on()
        t0 = time.time()
        while self.img is None and time.time() - t0 < 15.0:
            rospy.sleep(0.2)
        files = []
        for i in range(frames):
            # 每帧都等一帧新的, 避免重复存同一张
            before = self.img
            t0 = time.time()
            while self.img is before and time.time() - t0 < 2.0:
                rospy.sleep(0.05)
            if self.img is None:
                rospy.logwarn('  抓帧失败: 相机没有数据')
                break
            fn = '%s_f%d.%s' % (stem, i, fmt)
            path = os.path.join(self.outdir, fn)
            if fmt == 'jpg':
                cv2.imwrite(path, self.img, [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality])
            else:
                cv2.imwrite(path, self.img)
            files.append(fn)
            rospy.sleep(0.25)
        self.cam_off()
        return files


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--route', default=DEFAULT_ROUTE)
    ap.add_argument('--out', default=None, help='输出文件夹 (默认 captures/<时间戳>)')
    ap.add_argument('--tag', default='', help='文件夹名后缀')
    ap.add_argument('--frames', type=int, default=3, help='每个点抓几帧')
    ap.add_argument('--settle', type=float, default=1.2, help='到点后等几秒再拍')
    ap.add_argument('--format', choices=['png', 'jpg'], default='png')
    ap.add_argument('--quality', type=int, default=95, help='jpg 质量')
    ap.add_argument('--only', default=None, help='只拍这些目标 (逗号分隔)')
    ap.add_argument('--camera', default=CAM_TOPIC)
    ap.add_argument('--no-drive', dest='drive', action='store_false',
                    help='不移动, 只在当前位置抓一轮 (调试用)')
    ap.add_argument('--timeout', type=float, default=60.0)
    ap.add_argument('--max-w', type=float, default=1.0)
    ap.add_argument('--k-w', type=float, default=1.2)
    ap.add_argument('--no-pre-rotate', dest='pre_rotate', action='store_false')
    ap.add_argument('--goal-includes-yaw', dest='goal_includes_yaw', action='store_true')
    ap.add_argument('--park-pos-tol', dest='park_pos_tol', type=float, default=0.012)
    ap.add_argument('--park-yaw-tol', dest='park_yaw_tol', type=float, default=0.02)
    a = ap.parse_args()

    stamp = time.strftime('%Y-%m-%d_%H%M%S')
    outdir = a.out or os.path.join(CAPTURES, stamp + (('_' + a.tag) if a.tag else ''))
    os.makedirs(outdir, exist_ok=True)

    wps, start, start_yaw, ret, park = load_route(a.route)
    gt = load_ground_truth()
    only = set(x.strip() for x in a.only.split(',')) if a.only else None
    # 只对"识别点"拍照 (task 不是 waypoint 的)
    def _tgt(w):
        h = w['name'].split('_', 1)
        return h[1] if len(h) > 1 else w['name']
    shots = [w for w in wps if w.get('task') and w['task'] != 'waypoint'
             and (only is None or _tgt(w) in only or w['name'] in only)]

    rospy.init_node('capture_points', anonymous=True)
    cap = Capture(a, outdir)
    print('=' * 88)
    print('到识别点位拍照')
    print('=' * 88)
    print('  路线    : %s  (%d 站, 其中识别点 %d 个)'
          % (os.path.relpath(a.route, WS), len(wps), len(shots)))
    print('  输出    : %s' % os.path.relpath(outdir, WS))
    print('  相机    : %s   每点 %d 帧, 到点后等 %.1fs' % (a.camera, a.frames, a.settle))
    print('  本次目标: %s' % ', '.join(w['name'] for w in shots))

    if not cap.wait_camera():
        rospy.logerr('相机 %s 没有数据 —— 仿真起来了吗? 话题名对不对?'
                     ' (rostopic hz %s)' % (a.camera, a.camera))
        return 2

    rows, t_start = [], time.time()
    for w in wps:
        if a.drive:
            ok = cap.go_to(w['name'], w['x'], w['y'], w.get('yaw'))
            if not ok:
                rospy.logwarn('  跳过 %s (没到点)' % w['name'])
        else:
            ok = True
        if not (w.get('task') and w['task'] != 'waypoint'):
            continue                                  # 航点只路过, 不拍
        if only is not None and not (tgt in only or w['name'] in only):
            continue
        pose = cap.pose() or (float('nan'),) * 3
        # 路线里名字形如 "01_tl_top"
        head = w['name'].split('_', 1)
        order = int(head[0]) if head[0].isdigit() else len(rows) + 1
        tgt = head[1] if len(head) > 1 else w['name']
        g = gt.get(tgt, {})
        stem = '%02d_%s' % (order, tgt)
        t1 = time.time()
        files = cap.shoot(stem, a.frames, a.settle, a.format, a.quality) if ok else []
        rows.append(dict(
            seq=len(rows) + 1, target=tgt, task=w.get('task', ''),
            x=round(w['x'], 4), y=round(w['y'], 4),
            yaw_deg=round(math.degrees(w.get('yaw') or 0.0), 1),
            real_x=round(pose[0], 4), real_y=round(pose[1], 4),
            real_yaw_deg=round(math.degrees(pose[2]), 1),
            ground_truth=g.get('expect', ''), gt_note=g.get('note', ''),
            light_state=cap.light, files=';'.join(files),
            shoot_s=round(time.time() - t1, 2), reached=int(ok)))
        rospy.loginfo('  📷 %-9s %d 帧 -> %s   %s'
                      % (tgt, len(files), ', '.join(files) or '(空)',
                         ('真值 %s' % g['expect']) if g.get('expect') else ''))

    # ---- 元信息 + 索引 ----
    meta = dict(
        created=time.strftime('%Y-%m-%d %H:%M:%S'),
        run_dir=os.path.basename(outdir),
        route=os.path.relpath(a.route, WS),
        git_head=git_head(),
        camera_topic=a.camera, frames_per_point=a.frames, settle_s=a.settle,
        format=a.format, drive=bool(a.drive),
        route_points=len(wps), captured_points=len(rows),
        duration_s=round(time.time() - t_start, 1),
        note='每个识别点一张/多张原图 + index.csv(含真值), 供识别节点验收用',
    )
    with open(os.path.join(outdir, 'run.yaml'), 'w') as f:
        yaml.safe_dump(meta, f, allow_unicode=True, sort_keys=False)
    if rows:
        with open(os.path.join(outdir, 'index.csv'), 'w', newline='') as f:
            wr = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            wr.writeheader()
            for r in rows:
                wr.writerow(r)

    print()
    print('─' * 88)
    print('  拍完 %d 个点, 用时 %.0fs' % (len(rows), time.time() - t_start))
    print('  图片与索引: %s' % os.path.relpath(outdir, WS))
    print('  快速看图  : python3 tools/show_captures.py %s' % os.path.relpath(outdir, WS))
    return 0


if __name__ == '__main__':
    sys.exit(main())
