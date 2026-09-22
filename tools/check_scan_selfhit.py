#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""检查 /scan 里有没有"打到自己车体"的假回波。

为什么要查: 3D 雷达装得比车顶高, 下俯光束会打到自己的车顶/支架上, 在**车尾**
形成一个恒定距离的小扇形 (实测 ~0.207 m, 车体局部 147°~213°)。它跟着车走,
SLAM 会把它当成一面墙 -> 轨迹被拖住、地图歪掉。

做法: 用 /odom_groundtruth 的真值位姿 + 场地已知**围墙内沿 +-2.176 m**
      (白线是 +-2.076, 围墙在它外面 0.10m —— 见 check_scan_geometry.py 的说明)
      算出每个方向"应该"看到多远的墙, 再和实测比。

用法:
    roslaunch competition_robot robot_gazebo.launch gui:=false   # 另开一个终端
    python3 tools/check_scan_selfhit.py
"""
from __future__ import annotations

import math
import sys

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

WX, WY = 2.176, 2.176        # 场地内沿


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main():
    rospy.init_node('check_scan_selfhit', anonymous=True)
    p = rospy.get_param('/robot_params', {})
    try:
        # /scan 现在是 2D 雷达发的; 拿不到就用 3D 的安装位置 (两者仿真里同高)
        sen = p['sensors']
        src = sen['lidar2d'] if sen.get('lidar2d', {}).get('enabled') else sen['lidar3d']
        mount = [float(v) for v in src['mount']]
    except Exception:
        mount = [0.13374, 0.0, 0.2]
        print('  (没读到 /robot_params, 用默认雷达偏移 %s)' % mount)

    scan = rospy.wait_for_message('/scan', LaserScan, timeout=30)
    gt = rospy.wait_for_message('/odom_groundtruth', Odometry, timeout=30)
    bx, by = gt.pose.pose.position.x, gt.pose.pose.position.y
    byaw = yaw_of(gt.pose.pose.orientation)
    # 雷达在世界系的位置和朝向
    lx = bx + mount[0] * math.cos(byaw) - mount[1] * math.sin(byaw)
    ly = by + mount[0] * math.sin(byaw) + mount[1] * math.cos(byaw)
    print('  车 (%.3f, %.3f, %.1f deg)   雷达 (%.3f, %.3f)'
          % (bx, by, math.degrees(byaw), lx, ly))

    n = len(scan.ranges)
    errs, bad, selfhit = [], [], []
    for i in range(n):
        r = scan.ranges[i]
        if not math.isfinite(r) or r <= 0:
            continue
        a = scan.angle_min + scan.angle_increment * i
        wa = a + byaw                                   # 世界方向
        dx, dy = math.cos(wa), math.sin(wa)
        cand = []
        if dx > 1e-6:
            cand.append((WX - lx) / dx)
        if dx < -1e-6:
            cand.append((-WX - lx) / dx)
        if dy > 1e-6:
            cand.append((WY - ly) / dy)
        if dy < -1e-6:
            cand.append((-WY - ly) / dy)
        exp = min(c for c in cand if c > 0)
        # 3D 雷达报的是斜距, 墙有高度, 所以这里只做粗判
        errs.append(r - exp)
        if r < exp - 0.15:
            bad.append((math.degrees(a) % 360, r, exp))
            # 车体局部 120~240 度 = 车尾, 且距离很短 -> 典型自遮挡
            loc = math.degrees(a) % 360
            if 120 <= loc <= 240 and r < 0.5:
                selfhit.append((loc, r))

    if not errs:
        print('  /scan 没有有效回波'); return
    errs.sort()
    med = errs[len(errs) // 2]
    print('  有效束 %d, 实测-预期 中位 %+.3f m, 比预期近 15cm 以上的 %d 束'
          % (len(errs), med, len(bad)))
    if selfhit:
        rs = [r for _, r in selfhit]
        print('  ✗ 车尾短回波 %d 束 (距离 %.3f~%.3f m) —— 这就是打到自己的假回波!'
              % (len(selfhit), min(rs), max(rs)))
        print('    检查 /pointcloud_to_scan 的 self_filter 有没有生效:')
        print('    grep self_filter ~/.ros/log/latest/*.log  或看启动日志')
        sys.exit(1)
    print('  ✓ 没发现车体自身回波扇形, /scan 干净')


if __name__ == '__main__':
    main()
