#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""几何投影自检 —— 量"投影出来的框"和"画面里的目标"差多少。

为什么需要它（issue #10 的教训）
--------------------------------
合成数据集的框是**几何投影**算出来的。当投影和渲染对不上时，
**mAP / edge score 都发现不了** —— 因为 YOLO 是在"学标签"：
标签整体偏上，模型就学着整体偏上，mAP 反而 0.995；
框下沿正好压在目标的强梯度边缘上时，edge score 也很漂亮。
**只有拿真图量，才能发现"框和画面对不上"。**

它量什么
--------
把车传送到离目标已知距离处，拍一张，检测**亮着的灯珠**（发光色块，最不容易检错），
和几何投影出来的位置比 → 得到像素垂直偏差 Δv。

判定口径（关键）
----------------
    Δv  与 1/depth 成正比   ->  **相机高度**错 (Δz = Δv*depth/fx)
    Δv  与 depth 无关       ->  **主点**错 (cy 偏移)

所以要在**两个以上距离**各量一次。只在一个距离上调参，很容易补错方向。

用法
----
    roslaunch competition_robot robot_gazebo.launch gui:=false rviz:=false
    python3 tools/check_projection.py                    # 默认 1.0/1.6/2.4 m
    python3 tools/check_projection.py --dists 0.8,2.4

改完 `gen_recognition_points.py` 顶部的 `CAM_Z_CALIB`（或主点）后重跑本工具复核。
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time

import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import cv2                                          # noqa: E402
import rospy                                        # noqa: E402
from cv_bridge import CvBridge                      # noqa: E402
from gazebo_msgs.msg import ModelState              # noqa: E402
from gazebo_msgs.srv import SetModelState           # noqa: E402
from nav_msgs.msg import Odometry                   # noqa: E402
from sensor_msgs.msg import Image                   # noqa: E402
from std_msgs.msg import String                     # noqa: E402

import gen_recognition_points as G                  # noqa: E402
import gen_vision_dataset as D                      # noqa: E402

COLORS = {'red': ((0, 90, 120), (10, 255, 255)),
          'yellow': ((20, 90, 120), (35, 255, 255)),
          'green': ((45, 90, 120), (95, 255, 255))}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--light', default='tl_top')
    ap.add_argument('--state', default='green')
    ap.add_argument('--dists', default='1.0,1.6,2.4', help='相机到灯珠的距离 (m)')
    ap.add_argument('--robot-model', default='competition_robot')
    a = ap.parse_args()

    rospy.init_node('check_projection', anonymous=True, disable_signals=True)
    try:
        rospy.wait_for_service('/gazebo/set_model_state', timeout=20)
    except rospy.ROSException:
        print('找不到 /gazebo/set_model_state —— 先把仿真起起来:')
        print('  roslaunch competition_robot robot_gazebo.launch gui:=false rviz:=false')
        return 1
    setst = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    cmd = rospy.Publisher('/traffic_light/command', String, queue_size=1)

    st = dict(img=None, stamp=rospy.Time(0), truth=None)
    rospy.Subscriber('/camera/rgb/image_raw', Image,
                     lambda m: st.update(img=m, stamp=m.header.stamp), queue_size=1)

    def on_truth(m):
        q = m.pose.pose.orientation
        st['truth'] = (m.pose.pose.position.x, m.pose.pose.position.y,
                       math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                  1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
    rospy.Subscriber('/odom_groundtruth', Odometry, on_truth, queue_size=5)
    bridge = CvBridge()

    rob = G.load_robot()
    lt = G.load_lights()[a.light]
    lamps = {l['color']: l['p'] for l in G.light_geom(lt)['lamps']}
    target = lamps[a.state]
    cmd.publish(String(data=a.state))
    rospy.sleep(1.5)
    lx, ly = float(lt['x']), float(lt['y'])

    print('几何投影自检  (标定后相机 z=%.5f m, fx=%.1f)' % (rob['mount'][2], rob['fx']))
    print('靶标: %s 的 %s 灯珠, 世界 (%.3f, %.3f, %.3f)'
          % (a.light, a.state, target[0], target[1], target[2]))
    print()
    print('%-7s %-9s %-22s %-22s %s' % ('距离', 'depth', '投影 (u,v)', '实测 (u,v)', 'Δv (px)'))
    print('-' * 88)

    rows = []
    for d in [float(s) for s in a.dists.split(',')]:
        # 灯箱正面朝 +x -> 相机放在 +x 侧, 朝 -x 看 (yaw = π)
        yaw = math.pi
        camx, camy = target[0] + d, target[1]        # 想让**相机**离灯珠 d
        x = camx - math.cos(yaw) * rob['mount'][0]
        y = camy - math.sin(yaw) * rob['mount'][0]
        ms = ModelState()
        ms.model_name = a.robot_model
        ms.pose.position.x, ms.pose.position.y, ms.pose.position.z = x, y, 0.02
        ms.pose.orientation.z, ms.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        ms.reference_frame = 'world'
        setst(ms)
        t_cmd = rospy.Time.now()
        t0 = time.time()
        ok = False
        while time.time() - t0 < 6.0:
            rospy.sleep(0.05)
            if st['img'] is None or st['stamp'] <= t_cmd:
                continue
            tr = st['truth']
            if tr and math.hypot(tr[0] - x, tr[1] - y) < 0.04:
                ok = True
                break
        if not ok:
            print('%-7.2f 传送没到位, 跳过' % d)
            continue
        frame = bridge.imgmsg_to_cv2(st['img'], 'bgr8')
        u, v, dep = D.project([target], x, y, yaw, rob)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lo, hi = COLORS[a.state]
        m = cv2.inRange(hsv, np.array(lo), np.array(hi))
        n, lab, stats, cent = cv2.connectedComponentsWithStats(m)
        blobs = [(stats[i, 4], cent[i]) for i in range(1, n) if stats[i, 4] > 100]
        if not blobs:
            print('%-7.2f %-9.3f (%.1f, %.1f)  没找到亮灯珠' % (d, dep[0], u[0], v[0]))
            continue
        area, c = max(blobs)
        dv = c[1] - v[0]
        du = c[0] - u[0]
        rows.append((dep[0], dv, du))
        print('%-7.2f %-9.3f (%7.1f,%7.1f)   (%7.1f,%7.1f)   %+6.1f  (Δu %+.1f, 面积 %d)'
              % (d, dep[0], u[0], v[0], c[0], c[1], dv, du, area))

    if len(rows) >= 2:
        print()
        (d1, dv1, _), (d2, dv2, _) = rows[0], rows[-1]
        print('判定: 距离 %.2f->%.2f m, Δv %+.1f->%+.1f px' % (d1, d2, dv1, dv2))
        if abs(dv1) < 3 and abs(dv2) < 3:
            print('  ✓ 投影和画面吻合 (<3 px), 当前标定可用')
        else:
            pred_z = dv1 * d1 / rob['fx']          # 若按"高度差"解释, 需要的 Δz
            pred_const = dv1                        # 若按"主点"解释, 需要固定平移这么多
            expect_if_z = pred_z * rob['fx'] / d2   # 高度解释下, 远处应当只剩这么多
            print('  如果根因是**相机高度**: Δz ≈ %+.4f m, 那么 %.2f m 处应约 %+.1f px'
                  % (pred_z, d2, expect_if_z))
            print('  如果根因是**主点偏移**: %.2f m 处应仍是 %+.1f px (与距离无关)'
                  % (d2, pred_const))
            print('  实测 %.2f m 处是 %+.1f px -> 更像 **%s**'
                  % (d2, dv2,
                     '相机高度' if abs(dv2 - expect_if_z) < abs(dv2 - pred_const)
                     else '主点偏移'))
    cmd.publish(String(data='auto'))
    return 0


if __name__ == '__main__':
    sys.exit(main())
