#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""航点巡航 —— 按用户的做法: **先用 cmd_vel 原地转到方位, 再让 move_base(DWA) 走直线**。

为什么这么做: DWA 在"边转边走"时末端会画一段弧线(实测 90° 转向偏 8~9 cm),
而在原地把朝向先对好、再直线过去就非常准(实测 3 m 直线横向偏差 2.4 cm)。

用法:
    python3 tools/patrol.py                          # 用 config/waypoints.yaml
    python3 tools/patrol.py --file xxx.yaml
    python3 tools/patrol.py --dry-run                # 只打印路线, 不动
    python3 tools/patrol.py --no-return-start        # 最后不回起点
    python3 tools/patrol.py --loop                   # 一直循环
    python3 tools/patrol.py --save-trace /tmp/t.csv  # 记录轨迹(画图用)

航点文件格式 (world 和 pixel 二选一, pixel 按 coord_world_grid.png 那张图的像素):
    start: [1.772, 1.782]
    waypoints:
      - {name: A, pixel: [1138, 145]}          # 或 world: [1.709, 1.709]
      - {name: B, world: [0.0, -1.0], yaw: 1.57}   # 可选: 到点后再原地转到这个朝向
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time

import actionlib
import numpy as np
import rospy
import tf2_ros
import yaml
from geometry_msgs.msg import PoseStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from map_pixels import px2x, px2y, GOAL_LIMIT      # noqa: E402

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_FILE = os.path.join(WS, 'src', 'competition_robot', 'config', 'waypoints.yaml')


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Patrol(object):
    def __init__(self, a):
        self.a = a
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.tf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.gt = None
        self.trace = []
        rospy.Subscriber('/odom_groundtruth', Odometry, self.cb_gt, queue_size=50)

    def cb_gt(self, m):
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.gt = (p.x, p.y, yaw)
        self.trace.append((m.header.stamp.to_sec(), p.x, p.y, yaw))

    def pose(self):
        """机器人当前位姿 (map 系); 优先 TF, 拿不到就退回真值/里程计"""
        try:
            t = self.tf.lookup_transform('map', 'base_footprint', rospy.Time(0),
                                         rospy.Duration(1.0))
            p = t.transform.translation
            q = t.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            return (p.x, p.y, yaw)
        except Exception as e:
            rospy.logwarn_throttle(5.0, '查 TF 失败: %s' % e)
            return self.gt

    def stop(self):
        self.cmd_pub.publish(Twist())
        rospy.sleep(0.2)

    def rotate_to(self, target_yaw, tol=0.02, timeout=15.0):
        """原地转到 target_yaw (rad). 返回最终误差(rad)"""
        rate = rospy.Rate(20)
        t0 = rospy.Time.now()
        prev = None
        while not rospy.is_shutdown():
            p = self.pose()
            if p is None:
                rate.sleep(); continue
            err = wrap(target_yaw - p[2])
            if abs(err) < tol:
                break
            if (rospy.Time.now() - t0).to_sec() > timeout:
                rospy.logwarn('  原地转向超时, 剩余误差 %.1f deg' % math.degrees(err))
                break
            w = max(-self.a.max_w, min(self.a.max_w, self.a.k_w * err))
            # 末端用更小的角速度, 才能停在 1 度以内 (20Hz 下 0.15rad/s 一步 0.43 度)
            lo = 0.12 if abs(err) < 0.15 else 0.25
            if abs(w) < lo:
                w = math.copysign(lo, w)
            t = Twist()
            t.angular.z = w
            self.cmd_pub.publish(t)
            prev = err
            rate.sleep()
        self.stop()
        p = self.pose()
        return abs(wrap(target_yaw - p[2])) if p else -1.0

    def send_goal(self, x, y, yaw, timeout):
        g = MoveBaseGoal()
        g.target_pose.header.frame_id = 'map'
        g.target_pose.header.stamp = rospy.Time.now()
        g.target_pose.pose.position.x = x
        g.target_pose.pose.position.y = y
        g.target_pose.pose.orientation.z = math.sin(yaw / 2.0)
        g.target_pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.client.send_goal(g)
        ok = self.client.wait_for_result(rospy.Duration(timeout))
        st = self.client.get_state()
        return ok and st == actionlib.GoalStatus.SUCCEEDED, st

    def go_to(self, name, x, y, yaw_final=None):
        t0 = time.time()
        p = self.pose()
        if p is None:
            rospy.logerr('拿不到位姿, 跳过 %s' % name)
            return False
        bearing = math.atan2(y - p[1], x - p[0])
        if self.a.pre_rotate:
            self.rotate_to(bearing)
        ok, st = self.send_goal(x, y, yaw_final if yaw_final is not None else bearing,
                                self.a.timeout)
        if ok:
            if yaw_final is not None:
                e_yaw = self.rotate_to(yaw_final)   # 到点后再把朝向摆正 (cmd_vel)
                rospy.loginfo('     └ 原地摆正到 %.1f deg, 残余 %.1f deg'
                              % (math.degrees(wrap(yaw_final)), math.degrees(e_yaw)))
            e_amcl = None
            g = self.pose()
            if g:
                e_amcl = math.hypot(g[0] - x, g[1] - y)
            e_gt = math.hypot(self.gt[0] - x, self.gt[1] - y) if self.gt else None
            rospy.loginfo('  ✓ %-8s 到点  用时 %4.1fs  AMCL误差 %.3f m%s'
                          % (name, time.time() - t0, e_amcl or -1,
                             ('  真值误差 %.3f m' % e_gt) if e_gt is not None else ''))
            return True
        rospy.logwarn('  ✗ %-8s 失败/超时 (action state=%d, %.1fs)' % (name, st, time.time() - t0))
        return False


def load(path):
    cfg = yaml.safe_load(open(path))
    wps = []
    for i, w in enumerate(cfg.get('waypoints', [])):
        if 'world' in w:
            x, y = float(w['world'][0]), float(w['world'][1])
        elif 'pixel' in w:
            x, y = px2x(w['pixel'][0]), px2y(w['pixel'][1])
        else:
            raise ValueError('第 %d 个航点既没有 world 也没有 pixel' % (i + 1))
        wps.append(dict(name=w.get('name', 'P%d' % (i + 1)), x=x, y=y,
                        yaw=w.get('yaw')))
    start = cfg.get('start')
    start_yaw = cfg.get('start_yaw')
    return (wps,
            ([float(start[0]), float(start[1])] if start else None),
            (float(start_yaw) if start_yaw is not None else None))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--file', default=DEFAULT_FILE)
    ap.add_argument('--dry-run', action='store_true', help='只打印路线')
    ap.add_argument('--no-return-start', action='store_true')
    ap.add_argument('--no-pre-rotate', dest='pre_rotate', action='store_false')
    ap.add_argument('--loop', action='store_true')
    ap.add_argument('--timeout', type=float, default=60.0, help='每段超时(仿真秒)')
    ap.add_argument('--max-w', type=float, default=1.0, help='原地转向最大角速度')
    ap.add_argument('--k-w', type=float, default=1.2, help='转向 P 增益')
    ap.add_argument('--save-trace', help='把轨迹存成 csv')
    ap.set_defaults(pre_rotate=True)
    a = ap.parse_args()

    wps, start, start_yaw = load(a.file)
    print('航点 %d 个 (来自 %s)' % (len(wps), a.file))
    for w in wps:
        flag = '' if max(abs(w['x']), abs(w['y'])) <= GOAL_LIMIT else '  ⚠ 太靠墙'
        print('   %-8s (%+.3f, %+.3f)%s' % (w['name'], w['x'], w['y'], flag))
    if a.dry_run:
        return

    rospy.init_node('patrol', anonymous=True)
    p = Patrol(a)
    rospy.loginfo('等 move_base ...')
    if not p.client.wait_for_server(rospy.Duration(30.0)):
        rospy.logerr('没有 move_base: 先 roslaunch competition_robot navigation.launch')
        sys.exit(1)

    # 出发前的朝向 (巡航结束后要转回来)
    p0 = None
    for _ in range(50):
        p0 = p.pose()
        if p0:
            break
        rospy.sleep(0.2)
    yaw_home = start_yaw if start_yaw is not None else (p0[2] if p0 else 0.0)
    rospy.loginfo('出发朝向 %.1f deg (巡航结束会转回这个朝向)'
                  % math.degrees(wrap(yaw_home)))

    ok_n = 0
    try:
        while not rospy.is_shutdown():
            for w in wps:
                if p.go_to(w['name'], w['x'], w['y'], w['yaw']):
                    ok_n += 1
            if start and not a.no_return_start:
                p.go_to('start', start[0], start[1], yaw_home)
            rospy.loginfo('一圈跑完: %d/%d 个航点成功' % (ok_n, len(wps)))
            if not a.loop:
                break
    except KeyboardInterrupt:
        pass
    finally:
        p.stop()
        if a.save_trace and p.trace:
            with open(a.save_trace, 'w') as f:
                f.write('t,x,y,yaw\n')
                for r in p.trace:
                    f.write('%.3f,%.4f,%.4f,%.4f\n' % r)
            print('轨迹已存 ->', a.save_trace)


if __name__ == '__main__':
    main()
