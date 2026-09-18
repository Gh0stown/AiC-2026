#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""给 move_base 发一个目标点并等它跑完 —— 不用开 RViz 也能试导航。

用法:
    python3 tools/go_to.py 0 0                # 去场地中心 (yaw 随意)
    python3 tools/go_to.py -1.5 -1.5 1.57     # 去某个角, 朝 +y
    python3 tools/go_to.py 0 0 --timeout 60

退出码: 0 = 到了, 1 = 失败/超时
"""
from __future__ import annotations

import argparse
import math
import sys

import actionlib
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid

# 目标点离"障碍格"要留多少:
#   墙边 0.18 m 是 costmap 的"内切禁区"(inscribed_radius, 由 footprint 顶点算出)
#   再加上车体自身前后凸出 ~0.20 m  -> 圆心实际要离墙 ~0.38 m
#   这里取 0.30(偏保守但不会拦掉本来能到的点), 低于它的目标会自动挪到最近合法点。
ROBOT_CLEARANCE = 0.30


def keepout_check(goal_x, goal_y, snap=True, margin=0.05):
    """用 /map 检查目标点周围有没有 0.19 m 以上的空地。
    地图里墙有 2 格 (0.10 m) 厚, 所以贴墙目标很容易落进"放不下车"的区域。
    放不下时, 返回 (是否安全, 建议点)。"""
    try:
        m = rospy.wait_for_message('/map', OccupancyGrid, timeout=5)
    except rospy.ROSException:
        rospy.logwarn('读不到 /map, 跳过可达性检查')
        return True, (goal_x, goal_y)
    res = m.info.resolution
    w, h = m.info.width, m.info.height
    grid = np.array(m.data, dtype=np.int8).reshape(h, w)
    occ = (grid > 50)                       # 障碍
    r = int(round((ROBOT_CLEARANCE + margin) / res))
    # 用 numpy 平移做"膨胀", 得到不能去的格子
    bad = np.zeros_like(occ)
    for dy in range(-r, r + 1):
        for dx in range(-r, r + 1):
            if dx * dx + dy * dy <= r * r:
                bad |= np.roll(np.roll(occ, dy, 0), dx, 1)
    def to_map(x, y):
        return (int(round((x - m.info.origin.position.x) / res)),
                int(round((y - m.info.origin.position.y) / res)))
    gx, gy = to_map(goal_x, goal_y)
    if not (0 <= gx < w and 0 <= gy < h):
        rospy.logwarn('目标 (%.2f, %.2f) 不在地图范围里' % (goal_x, goal_y))
        return False, (goal_x, goal_y)
    if not bad[gy, gx] and grid[gy, gx] <= 50:
        return True, (goal_x, goal_y)
    # 找最近的安全格
    ys, xs = np.nonzero(~bad)
    if len(ys) == 0:
        return False, (goal_x, goal_y)
    d2 = (xs - gx) ** 2 + (ys - gy) ** 2
    i = int(np.argmin(d2))
    sx = xs[i] * res + m.info.origin.position.x
    sy = ys[i] * res + m.info.origin.position.y
    rospy.logwarn('目标 (%.2f, %.2f) 离墙太近, 车体放不下 (需要 %.2f m 空地)'
                  % (goal_x, goal_y, ROBOT_CLEARANCE + margin))
    if snap:
        rospy.logwarn('  -> 自动挪到最近的合法点 (%.2f, %.2f), 距离 %.2f m'
                      % (sx, sy, math.hypot(sx - goal_x, sy - goal_y)))
        return False, (sx, sy)
    return False, (goal_x, goal_y)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('x', type=float)
    ap.add_argument('y', type=float)
    ap.add_argument('yaw', type=float, nargs='?', default=0.0, help='弧度, 默认 0')
    ap.add_argument('--frame', default='map')
    ap.add_argument('--timeout', type=float, default=120.0)
    ap.add_argument('--no-snap', action='store_true',
                    help='目标点离墙太近时不要自动挪, 直接报错退出')
    a = ap.parse_args()

    rospy.init_node('go_to', anonymous=True)
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo('等待 move_base ...')
    if not client.wait_for_server(rospy.Duration(20.0)):
        rospy.logerr('没有 move_base (导航起了吗? roslaunch competition_robot navigation.launch)')
        sys.exit(1)

    ok, (tx, ty) = keepout_check(a.x, a.y, snap=not a.no_snap)
    if not ok and a.no_snap:
        rospy.logerr('目标点放不下车体, 换个离墙远一点的 (>=0.3 m)')
        sys.exit(2)

    g = MoveBaseGoal()
    g.target_pose.header.frame_id = a.frame
    g.target_pose.header.stamp = rospy.Time.now()
    g.target_pose.pose.position.x = tx
    g.target_pose.pose.position.y = ty
    g.target_pose.pose.orientation.z = math.sin(a.yaw / 2.0)
    g.target_pose.pose.orientation.w = math.cos(a.yaw / 2.0)

    rospy.loginfo('目标 (%.2f, %.2f, %.2f rad) [%s]' % (tx, ty, a.yaw, a.frame))
    client.send_goal(g)
    ok = client.wait_for_result(rospy.Duration(a.timeout))
    if not ok:
        client.cancel_goal()
        rospy.logerr('超时 %.0f s, 已取消' % a.timeout)
        sys.exit(1)
    st = client.get_state()
    if st == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo('✓ 到达 (用了 %.1f s)' % (rospy.Time.now() - g.target_pose.header.stamp).to_sec())
        sys.exit(0)
    rospy.logerr('✗ 失败, action state = %d' % st)
    sys.exit(1)


if __name__ == '__main__':
    main()
