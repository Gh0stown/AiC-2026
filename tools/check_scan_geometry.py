#!/usr/bin/env python3
"""把 /scan 和场地已知几何对一遍: 每个方向应该看到哪面墙、距离多少"""
import math
import rospy
import tf
from sensor_msgs.msg import LaserScan

# 场地内沿
WX, WY = 2.076, 2.076
# 车在 (1.772, 1.782), yaw=pi; 雷达相对 base_link (0.134, 0, 0.20)
bx, by, byaw = 1.772, 1.782, math.pi
lx = bx + 0.134 * math.cos(byaw)
ly = by + 0.134 * math.sin(byaw)
print('  雷达位置: (%.4f, %.4f), 车 yaw=%.1f deg' % (lx, ly, math.degrees(byaw)))
print('  场地内沿: x +-%.3f, y +-%.3f' % (WX, WY))
print('  到各墙距离: +x %.3f  -x %.3f  +y %.3f  -y %.3f'
      % (WX-lx, lx+WX, WY-ly, ly+WY))
print()

rospy.init_node('check_scan_geometry', anonymous=True)
msg = rospy.wait_for_message('/scan', LaserScan, timeout=30)

# 雷达在 laser_link 下, 但 scan 的 angle=0 是 laser_link 的 +x, 也就是车头方向
# 车头朝 -x (yaw=pi), 所以 scan 的 0 度指向世界 -x
print('  %-8s %-10s %-10s %-10s' % ('scan角度', '世界方向', '实测距离', '预期(最近的墙)'))
import numpy as np
for deg in range(0, 360, 30):
    i = int(round(math.radians(deg) / msg.angle_increment)) % len(msg.ranges)
    r = msg.ranges[i]
    # scan 角度 -> 世界方向: 车 yaw=pi, 所以 world_ang = pi + deg
    wa = math.pi + math.radians(deg)
    dx, dy = math.cos(wa), math.sin(wa)
    # 到四面墙的距离
    cands = []
    if dx > 1e-6: cands.append((WX - lx) / dx)
    if dx < -1e-6: cands.append((-WX - lx) / dx)
    if dy > 1e-6: cands.append((WY - ly) / dy)
    if dy < -1e-6: cands.append((-WY - ly) / dy)
    exp = min(c for c in cands if c > 0)
    print('  %5d deg  (%+5.0f,%+5.0f)  %8.3f    %8.3f   %s'
          % (deg, dx, dy, r, exp, 'OK' if abs(r-exp) < 0.35 else ('差异 %.2f' % (r-exp))))
