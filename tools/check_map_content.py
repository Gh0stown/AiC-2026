#!/usr/bin/env python3
"""正经读一次 /map, 统计占用/空闲/未知 (注意 data 是 int8[])"""
import sys
import rospy
from nav_msgs.msg import OccupancyGrid

rospy.init_node('check_map_content', anonymous=True)
msg = rospy.wait_for_message('/map', OccupancyGrid, timeout=30)
d = msg.data
n = len(d)
occ = sum(1 for v in d if v == 0)
free = sum(1 for v in d if v == 100)
unk = n - occ - free
res = msg.info.resolution
print('  地图尺寸   : %d x %d 格 @ %.3f m/格  = %.1f x %.1f m'
      % (msg.info.width, msg.info.height, res,
         msg.info.width*res, msg.info.height*res))
print('  原点       : (%.2f, %.2f)' % (msg.info.origin.position.x, msg.info.origin.position.y))
print('  障碍(100)  : %8d  (%.1f m^2)' % (occ, occ*res*res))
print('  空闲(0)    : %8d  (%.1f m^2)' % (free, free*res*res))
print('  未知(-1)   : %8d' % unk)
print('  => %s' % ('地图建出来了 ✅' if occ > 200 and free > 5000 else '内容仍偏少 ⚠'))
