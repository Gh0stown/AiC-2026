#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""在**采集专用世界**里拍数据集（只出图 + meta, 不写框）。

配合 `tools/build_collect_world.py` 生成的 `collect.world` 使用。三个工位：

    A 立牌圈   小车在圈心附近, 逐个立牌**正对着**拍 K 张 (立牌全都正面朝圈心,
               所以拍到的永远是正面; 一帧里还会顺带带上左右邻居, 对训练是好事)
    B 红绿灯   每个灯 x **红黄绿三种状态** x 几个距离, 正对着拍 (状态是**强制**的,
               保证三类均衡, 不用等它自己循环)
    C 车牌     每辆车 x 几个距离, 正对着拍

★ 只出图不写框: 投影出来的框有系统性偏差 (issue #10), 手工用 X-AnyLabeling 标更可靠。
   meta.jsonl 里仍然记 **灯态** 和 **车牌字符串** —— 这两样手工标注标不出来, 评估要用。

用法
----
    # 1. 建世界 (只需一次)
    python3 tools/build_collect_world.py
    # 2. 起这个世界的仿真
    roslaunch competition_robot robot_gazebo.launch \\
        world:=$(rospack find competition_arena)/worlds/collect.world x:=0 y:=0 yaw:=0
    # 3. 先干跑看规划 (不用仿真)
    python3 tools/gen_collect_dataset.py --dry-run
    # 4. 真采
    python3 tools/gen_collect_dataset.py --out datasets/collect
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
import setup_cars as SC                     # noqa: E402
import gen_standees as GS                   # noqa: E402

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LAYOUT = os.path.join(WS, 'src', 'competition_arena', 'config', 'collect_layout.yaml')


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


# =============================================================================
#  从布局算出每个物体的 3D 包围盒 (只用来判断"在不在画面里", 不写进标签)
# =============================================================================
def boxes_from_layout(layout):
    widths = G.load_standees()               # {model: width_m}
    out = []
    for o in layout['objects']:
        cls, x, y, yaw = o['cls'], float(o['x']), float(o['y']), float(o['yaw'])
        if cls in ('standee', 'non_community'):
            w = float(widths.get(o['model'], 0.05))
            g = G.standee_box(x, y, yaw, w)
            out.append(dict(cls=cls, name=o['name'], x=x, y=y, pts=g['pts'], yaw=yaw,
                            face=(-math.cos(yaw), -math.sin(yaw))))
        elif cls == 'traffic_light':
            g = G.light_geom(dict(x=x, y=y, yaw=yaw))
            out.append(dict(cls=cls, name=o['name'], x=x, y=y, pts=g['pts'], yaw=yaw,
                            face=(math.cos(yaw), math.sin(yaw))))
        elif cls == 'plate':
            lx = SC.BOARD_T / 2.0 + SC.PLATE_T / 2.0 + SC.PLATE_CX
            lz = SC.BASE_T + SC.PLATE_CZ
            px, py = x + lx * math.cos(yaw), y + lx * math.sin(yaw)
            pts = []
            for dx in (+SC.PLATE_T / 2.0, -SC.PLATE_T / 2.0):
                for dy in (+SC.PLATE_W / 2.0, -SC.PLATE_W / 2.0):
                    for z in (lz - SC.PLATE_H / 2.0, lz + SC.PLATE_H / 2.0):
                        pts.append([px + dx * math.cos(yaw) - dy * math.sin(yaw),
                                    py + dx * math.sin(yaw) + dy * math.cos(yaw), z])
            out.append(dict(cls=cls, name=o['name'], x=x, y=y, pts=pts, yaw=yaw,
                            face=(math.cos(yaw), math.sin(yaw))))
    return out


def full_in_frame(pts, x, y, yaw, rob, margin=8.0):
    """整个目标(8 个角点)是否都在画面内 (留 margin 像素边距)。

    ★ 灯箱/车牌要"拍全": 红绿灯在 0.8m 时灯箱顶部只剩 76px 就贴边了 ✗,
      所以这两个工位要求**完整可见**, 不满足就换距离/重抽。
    """
    u, v, dep = G_proj(pts, x, y, yaw, rob)
    if (dep < 0.05).any():
        return False
    return bool(((u >= margin) & (u < rob['W'] - margin) &
                 (v >= margin) & (v < rob['H'] - margin)).all())


def dists_of(spec):
    """'1.0,1.4,1.8' 或 '1.0:2.0:0.25' (起:止:步长) -> 距离列表"""
    if ':' in spec:
        a, b, c = [float(v) for v in spec.split(':')]
        out, x = [], a
        while x <= b + 1e-9:
            out.append(round(x, 3))
            x += c
        return out
    return [float(v) for v in spec.split(',') if v.strip()]


def visible_names(objs, x, y, yaw, rob):
    """这一帧里大致能看到哪些物体 (投影粗判, 位置精度受 #10 影响, 但"在不在画面里"够用)"""
    names = []
    for o in objs:
        u, v, dep = G_proj(o['pts'], x, y, yaw, rob)
        if (dep < 0.05).any():
            continue
        inside = ((u >= 0) & (u < rob['W']) & (v >= 0) & (v < rob['H']))
        if inside.mean() >= 0.6 and (u.max() - u.min()) >= 20:
            names.append(o['name'])
    return names


def G_proj(pts, x, y, yaw, rob):
    """与 gen_recognition_points.evaluate 同一套相机数学"""
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
    return (rob['W'] / 2.0 - rob['fx'] * lateral / sd,
            rob['H'] / 2.0 - rob['fx'] * vert / sd, depth)


# =============================================================================
#  拍摄规划
# =============================================================================
skipped_full = [0]          # 因"目标拍不全"被跳过的位姿数


def plan(layout, args, rng):
    skipped_full[0] = 0
    args.rob = G.load_robot()
    # ★ 必须用 boxes_from_layout() 的结果: 它才带 3D 角点 ('pts')。
    #   直接用 layout['objects'] 会在 visible_names() 里 KeyError。
    objs = boxes_from_layout(layout)
    standees = [o for o in objs if o['cls'] in ('standee', 'non_community')]
    lights = [o for o in objs if o['cls'] == 'traffic_light']
    cars = [o for o in objs if o['cls'] == 'plate']
    plans = []

    # ---- A 立牌圈 ----
    for o in standees:
        for k in range(args.ring_views):
            # ★ 守卫: 拍太近立牌下沿会被切出画面 (相机只有 0.2m 高、水平看)。
            #   注意量的是**相机**到目标的距离, 不是车心 —— 相机在车心前方 0.149m。
            got = None
            for _try in range(40):
                th = rng.uniform(0, 2 * math.pi)
                r = rng.uniform(0, args.ring_offset)
                x, y = r * math.cos(th), r * math.sin(th)        # 圈心附近
                yaw = wrap(math.atan2(o['y'] - y, o['x'] - x)
                           + math.radians(rng.uniform(-args.yaw_jitter, args.yaw_jitter)))
                cx, cy = G.camera_xy(x, y, yaw, G.load_robot())
                d_cam = math.hypot(o['x'] - cx, o['y'] - cy)
                if d_cam >= args.min_shot_dist:
                    got = (x, y, yaw, d_cam)
                    break
            if got is None:
                x, y, yaw = 0.0, 0.0, math.atan2(o['y'], o['x'])
                cx, cy = G.camera_xy(x, y, yaw, G.load_robot())
                got = (x, y, yaw, math.hypot(o['x'] - cx, o['y'] - cy))
            plans.append(dict(phase='ring', target=o['name'], x=got[0], y=got[1],
                              yaw=got[2], dist=got[3], light=None))

    # ---- B 红绿灯 (距离扫描 x 三种状态, 且灯箱必须完整在画面里) ----
    for o in lights:
        for st in args.light_states.split(','):
            for d in dists_of(args.light_dists):
                for _ in range(args.per):
                    got = None
                    for _try in range(30):
                        lat = rng.uniform(-args.lat_jitter, args.lat_jitter)
                        x = o['x'] + d + rng.uniform(-0.03, 0.03)
                        y = o['y'] + lat
                        yaw = wrap(math.atan2(o['y'] - y, o['x'] - x)
                                   + math.radians(rng.uniform(-4, 4)))
                        if full_in_frame(o['pts'], x, y, yaw, args.rob, args.frame_margin):
                            got = (x, y, yaw)
                            break
                    if got is None:
                        skipped_full[0] += 1
                        continue
                    plans.append(dict(phase='light', target=o['name'], x=got[0], y=got[1],
                                      yaw=got[2], dist=d, light=st.strip()))

    # ---- C 车牌 (距离扫描, 车牌必须完整在画面里) ----
    for o in cars:
        for d in dists_of(args.car_dists):
            for _ in range(args.per):
                got = None
                for _try in range(30):
                    lat = rng.uniform(-args.lat_jitter, args.lat_jitter)
                    x = o['x'] + d + rng.uniform(-0.03, 0.03)
                    y = o['y'] + lat
                    yaw = wrap(math.atan2(o['y'] - y, o['x'] - x)
                               + math.radians(rng.uniform(-5, 5)))
                    if full_in_frame(o['pts'], x, y, yaw, args.rob, args.frame_margin):
                        got = (x, y, yaw)
                        break
                if got is None:
                    skipped_full[0] += 1
                    continue
                plans.append(dict(phase='plate', target=o['name'], x=got[0], y=got[1],
                                  yaw=got[2], dist=d, light=None))
    return plans, objs


# =============================================================================
#  抓帧
# =============================================================================
def capture(args, plans, objs, out_dir):
    import rospy
    from cv_bridge import CvBridge
    from gazebo_msgs.msg import ModelState
    from gazebo_msgs.srv import SetModelState
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    import cv2

    rospy.init_node('gen_collect_dataset', anonymous=True, disable_signals=True)
    try:
        rospy.wait_for_service('/gazebo/set_model_state', timeout=20)
    except rospy.ROSException:
        print('找不到 /gazebo/set_model_state —— 先把采集世界的仿真起起来:')
        print('  roslaunch competition_robot robot_gazebo.launch \\')
        print('      world:=$(rospack find competition_arena)/worlds/collect.world x:=0 y:=0 yaw:=0')
        return 1
    setst = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    cmd = rospy.Publisher('/traffic_light/command', String, queue_size=1)

    # ★ 用哪台相机小车: rig 是独立模型, 相机话题带自己的命名空间, 位姿从 Gazebo 服务读
    #   (rig 没有 /odom_groundtruth); 完整机器人则用真值话题。
    if args.rig:
        model = 'collect_rig_%d' % args.rig
        cam_topic = '/rig%d/camera/rgb/image_raw' % args.rig
        from gazebo_msgs.srv import GetModelState
        gs = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        def get_truth():
            try:
                r = gs(model, 'world')
                q = r.pose.orientation
                return (r.pose.position.x, r.pose.position.y,
                        math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                   1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
            except Exception:
                return None
    else:
        model = args.robot_model
        cam_topic = '/camera/rgb/image_raw'
        get_truth = lambda: st.get('truth')          # noqa: E731
    print('  载体: %s   相机话题: %s' % (model, cam_topic))

    st = dict(img=None, stamp=rospy.Time(0), truth=None, light='?')

    def on_img(m):
        st['img'], st['stamp'] = m, m.header.stamp

    def on_truth(m):
        q = m.pose.pose.orientation
        st['truth'] = (m.pose.pose.position.x, m.pose.pose.position.y,
                       math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                  1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
    rospy.Subscriber(cam_topic, Image, on_img, queue_size=1)
    rospy.Subscriber('/odom_groundtruth', Odometry, on_truth, queue_size=5)
    rospy.Subscriber('/traffic_light/state', String,
                     lambda m: st.update(light=m.data.strip().lower()), queue_size=5)
    bridge = CvBridge()
    rob = G.load_robot()

    img_dir = os.path.join(out_dir, 'images')
    os.makedirs(img_dir, exist_ok=True)
    meta = open(os.path.join(out_dir, 'meta.jsonl'), 'w')
    print('开始采集 %d 张 -> %s' % (len(plans), out_dir))

    done = skipped = 0
    for idx, p in enumerate(plans):
        # 红绿灯: 先强制灯态
        if p['phase'] == 'light' and p['light']:
            cmd.publish(String(data=p['light']))
            t0 = time.time()
            while st['light'] != p['light'] and time.time() - t0 < 6.0:
                rospy.sleep(0.1)
        ms = ModelState()
        ms.model_name = model
        # ★ rig 的相机 sensor 没有 <pose>, 相机就在模型原点上 —— 所以模型原点必须抬到
        #   相机高度 (mount[2]), 否则相机比投影假设的低 0.18 m, 目标整体被顶出画面。
        #   完整机器人 (competition_robot) 则相反: 它有重力, 落到 0 之后相机正好在
        #   mount[2], 所以给个小 z 让它落下去就行。
        z = rob['mount'][2] if args.rig else 0.02
        ms.pose.position.x, ms.pose.position.y, ms.pose.position.z = p['x'], p['y'], z
        ms.pose.orientation.z = math.sin(p['yaw'] / 2.0)
        ms.pose.orientation.w = math.cos(p['yaw'] / 2.0)
        ms.reference_frame = 'world'
        setst(ms)
        t_cmd, t_wall, ok = rospy.Time.now(), time.time(), False
        while time.time() - t_wall < 6.0:
            rospy.sleep(0.05)
            if st['img'] is None or st['stamp'] <= t_cmd:
                continue
            tr = get_truth()
            if tr and math.hypot(tr[0] - p['x'], tr[1] - p['y']) < 0.06 \
                    and abs(float(wrap(tr[2] - p['yaw']))) < math.radians(6.0):
                ok = True
                break
        if not ok:
            skipped += 1
            continue
        lx, ly, lyaw = get_truth() or (p['x'], p['y'], p['yaw'])
        frame = bridge.imgmsg_to_cv2(st['img'], 'bgr8')
        fn = '%04d_%s_%s' % (idx, p['phase'], p['target'])
        cv2.imwrite(os.path.join(img_dir, fn + '.' + args.img_ext), frame,
                    [cv2.IMWRITE_JPEG_QUALITY, args.jpeg_quality]
                    if args.img_ext == 'jpg' else [])
        meta.write(json.dumps(dict(
            file='images/%s.%s' % (fn, args.img_ext), phase=p['phase'],
            target=p['target'], dist=round(p['dist'], 3),
            pose=[round(p['x'], 4), round(p['y'], 4), round(p['yaw'], 4)],
            pose_truth=[round(lx, 4), round(ly, 4), round(lyaw, 4)],
            light=st['light'], commanded_light=p.get('light'),
            seen=visible_names(objs, lx, ly, lyaw, rob),
            note='seen 是投影粗判(位置精度受 issue #10 影响), 只用来核覆盖率',
        ), ensure_ascii=False) + '\n')
        done += 1
        if done % 10 == 0:
            print('  %d/%d (跳过 %d)' % (done, len(plans), skipped))
    cmd.publish(String(data='auto'))
    meta.close()
    print('完成: 存了 %d 张, 跳过 %d 张 -> %s' % (done, skipped, out_dir))
    return 0


# =============================================================================
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--layout', default=LAYOUT)
    ap.add_argument('--out', default='datasets/collect')
    ap.add_argument('--ring-views', type=int, default=8, help='每个立牌拍几张')
    ap.add_argument('--ring-offset', type=float, default=0.15,
                    help='小车在圈心附近随机偏移的上限 (m) —— 让视角/距离有变化, '
                         '但别太大: 相对圈半径超过 ~30%% 就会有目标被挤出画面 '
                         '(R=0.6 时 0.25 会让命中率掉到 65%%)')
    ap.add_argument('--yaw-jitter', type=float, default=6.0, help='朝向抖动 (度)')
    ap.add_argument('--min-shot-dist', type=float, default=0.48,
                    help='相机到目标的距离下限 (m)。低于这个, 立牌下沿会被切出画面 '
                         '(相机高 0.2m 水平看, 全身需要 d>=0.48)')
    ap.add_argument('--lat-jitter', type=float, default=0.06, help='侧向抖动 (m)')
    ap.add_argument('--per', type=int, default=1, help='每个 (距离/状态) 拍几张')
    ap.add_argument('--light-states', default='red,yellow,green')
    ap.add_argument('--light-dists', default='1.0:2.0:0.25',
                    help='红绿灯距离扫描: "起:止:步长" 或逗号列表。'
                         '★ 别小于 1.0m —— 灯箱顶部在 0.68m 就贴到画面边缘了')
    ap.add_argument('--car-dists', default='0.6,0.9,1.2')
    ap.add_argument('--frame-margin', type=float, default=8.0,
                    help='灯箱/车牌要求完整在画面内, 留这么多像素边距')
    ap.add_argument('--img-ext', default='jpg', choices=['jpg', 'png'])
    ap.add_argument('--jpeg-quality', type=int, default=92)
    ap.add_argument('--robot-model', default='competition_robot')
    ap.add_argument('--rig', type=int, default=0,
                    help='用第几台采集相机小车 (1/2/3); 0=用完整机器人 competition_robot')
    ap.add_argument('--only-phase', default='',
                    help='只拍某个工位: ring / light / plate (三台车并行时各管一个)')
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--dry-run', action='store_true', help='不连仿真, 只打印拍摄规划')
    a = ap.parse_args()

    if not os.path.exists(a.layout):
        print('没有布局文件 %s —— 先跑: python3 tools/build_collect_world.py' % a.layout)
        return 1
    layout = yaml.safe_load(open(a.layout))
    rng = random.Random(a.seed)
    plans, objs = plan(layout, a, rng)
    if a.only_phase:
        want = set(x.strip() for x in a.only_phase.split(',') if x.strip())
        plans = [p for p in plans if p['phase'] in want]
        if not plans:
            print('--only-phase %s 过滤后没有拍摄任务' % a.only_phase)
            return 1

    if a.dry_run:
        from collections import Counter
        print('采集规划 (布局: %s)' % a.layout)
        print('  物体: 立牌圈 %d, 红绿灯 %d, 车牌 %d'
              % (len([o for o in objs if o['cls'] in ('standee', 'non_community')]),
                 len([o for o in objs if o['cls'] == 'traffic_light']),
                 len([o for o in objs if o['cls'] == 'plate'])))
        print('  拍摄: 共 %d 张 -> %s' % (len(plans), Counter(p['phase'] for p in plans)))
        for ph in ('ring', 'light', 'plate'):
            ps = [p for p in plans if p['phase'] == ph]
            if not ps:
                continue
            ds = [p['dist'] for p in ps]
            print('    %-6s 目标 %d 个 x 视角 -> %d 张, 距离 %.2f~%.2f m'
                  % (ph, len(set(p['target'] for p in ps)), len(ps), min(ds), max(ds)))
        # 覆盖率: 每个目标被拍几次
        c = Counter(p['target'] for p in plans)
        print('  每个目标的张数: %s' % dict(sorted(c.items())))
        # ★ 离线也跑一遍可见性判断: 这样几何/字段类的错误不用起仿真就能暴露
        rob = G.load_robot()
        hit = sum(1 for p in plans if p['target'] in
                  visible_names(objs, p['x'], p['y'], p['yaw'], rob))
        print('  目标在画面里的比例(投影粗判): %d/%d = %.0f%%%s'
              % (hit, len(plans), 100.0 * hit / max(1, len(plans)),
                 ('   (另有 %d 个位姿因"拍不全"被跳过)' % skipped_full[0])
                 if skipped_full[0] else ''))
        return 0

    os.makedirs(a.out, exist_ok=True)
    return capture(a, plans, objs, a.out)


if __name__ == '__main__':
    sys.exit(main())
