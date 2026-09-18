#!/usr/bin/env python3
"""直接看 /points 在 laser_link 坐标系里的分布"""
import numpy as np, rospy
from sensor_msgs.msg import PointCloud

rospy.init_node('check_cloud', anonymous=True)
msg = rospy.wait_for_message('/points', PointCloud, timeout=30)
pts = np.array([[p.x, p.y, p.z] for p in msg.points])
print('  点数: %d, frame_id: %s' % (len(pts), msg.header.frame_id))
print('  x 范围 [%.3f, %.3f]   y 范围 [%.3f, %.3f]   z 范围 [%.3f, %.3f]'
      % (pts[:,0].min(), pts[:,0].max(), pts[:,1].min(), pts[:,1].max(),
         pts[:,2].min(), pts[:,2].max()))
print()
# 按方位角分 8 个扇区, 看每个扇区最近的点
ang = np.degrees(np.arctan2(pts[:,1], pts[:,0]))
r = np.hypot(pts[:,0], pts[:,1])
print('  %-10s %-10s %-10s %s' % ('方位角', '最近r', '最远r', '最近点在(x,y,z)'))
for lo in range(-180, 180, 45):
    m = (ang >= lo) & (ang < lo+45)
    if m.sum() == 0: continue
    i = np.argmin(r[m]); sub = pts[m]
    print('  %4d~%4d  %8.3f  %8.3f   (%.3f, %.3f, %.3f)'
          % (lo, lo+45, r[m].min(), r[m].max(), sub[i,0], sub[i,1], sub[i,2]))
