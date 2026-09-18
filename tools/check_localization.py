#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""测量定位精度: 把 AMCL 的位姿估计和 Gazebo 真值逐帧对比。

用法:
    tools/check_localization.py                  # 一直测, Ctrl-C 出结果
    tools/check_localization.py --duration 60    # 测 60 秒
    tools/check_localization.py --csv a.csv      # 顺便存逐帧数据

前提: 仿真已经在跑 (roslaunch competition_robot navigation.launch)。
本工具只订阅、不发指令, 所以可以一边让 patrol.py 跑一边测。

为什么能直接比: 本场地里 map 系和 world 系重合
(几何生成地图时 origin 就是按世界系算的), 所以
    /amcl_pose        (map 系, 定位结果)
    /odom_groundtruth (world 系, Gazebo 真值)
两个 pose 可以直接相减。

注意: **车不动的时候测出来的误差没有意义** (静止时粒子云不收窄)。
要让车跑起来测, 比如同时开 `tools/patrol.py`。
"""
from __future__ import annotations

import argparse
import math
import os
import sys
from collections import deque

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def pct(v, p):
    if not v:
        return float('nan')
    s = sorted(v)
    i = min(len(s) - 1, max(0, int(round(p / 100.0 * (len(s) - 1)))))
    return s[i]


class Measurer(object):
    def __init__(self, csv_path=None):
        self.amcl = None
        self.truth = None
        self.samples = []         # (t, err_xy, err_yaw_deg)
        self.n_amcl = 0           # 收到多少条 /amcl_pose
        self.hist = deque(maxlen=4000)   # 真值历史, 用来按时间戳回查
        self.csv = open(csv_path, 'w') if csv_path else None
        if self.csv:
            self.csv.write('t,amcl_x,amcl_y,gt_x,gt_y,err_xy,err_yaw_deg\n')

        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.cb_amcl, queue_size=20)
        rospy.Subscriber('/odom_groundtruth', Odometry, self.cb_truth, queue_size=50)

    def cb_truth(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.hist.append((m.header.stamp.to_sec(), p.x, p.y, yaw_of(q)))

    def cb_amcl(self, m):
        """每收到一条 AMCL 位姿, 就按**它自己的时间戳**回查真值并比对。

        这样做而不是"拿最新真值比", 是因为 AMCL 可能算得慢、位姿滞后;
        但比较双方在**同一时刻**的值仍然是公平的 (问的是
        "AMCL 认为 t 时刻它在哪" vs "t 时刻它真在哪")。
        原来要求两路时间戳相差 <0.05s, 一卡就丢样本, 已改成时间戳回查。
        """
        p, q = m.pose.pose.position, m.pose.pose.orientation
        t = m.header.stamp.to_sec()
        self.amcl = (t, p.x, p.y, yaw_of(q))
        self.n_amcl += 1
        if not self.hist:
            return
        # 找时间戳最接近的真值 (真值按序到达, 线性扫描足够快)
        best, bestd = None, 1e9
        for h in reversed(self.hist):
            d = abs(h[0] - t)
            if d < bestd:
                best, bestd = h, d
            elif d > bestd + 0.05:      # 越走越远, 可以停了
                break
        if best is None or bestd > 0.25:   # 差太远就不算 (真值是另一段时间的)
            return
        _, ax, ay, ayaw = self.amcl
        _, tx, ty, tyaw = best
        exy = math.hypot(ax - tx, ay - ty)
        eyaw = math.degrees(wrap(ayaw - tyaw))
        self.samples.append((t, exy, eyaw))
        if self.csv:
            self.csv.write('%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.3f\n'
                           % (t, ax, ay, tx, ty, exy, eyaw))

    def report(self):
        d = [s[1] for s in self.samples]
        yaw = [abs(s[2]) for s in self.samples]
        if not d:
            print('  没采到样本 —— 确认仿真在跑, 并且 /amcl_pose 与 /odom_groundtruth 都有数据')
            return None
        # 去掉开头 3 秒 (AMCL 刚起步粒子云还没收窄)
        t0 = self.samples[0][0]
        keep = [s for s in self.samples if s[0] - t0 > 3.0]
        use = keep if len(keep) > 5 else self.samples
        use_d = [s[1] for s in use]
        use_y = [abs(s[2]) for s in use]
        span = self.samples[-1][0] - self.samples[0][0]
        r = dict(n=len(use_d), mean=sum(use_d) / len(use_d),
                 med=pct(use_d, 50), p90=pct(use_d, 90), mx=max(use_d),
                 yaw_med=pct(use_y, 50), yaw_p90=pct(use_y, 90))
        print('  收到 /amcl_pose %d 条, 成功配对 %d 条, 跨度 %.1f s'
              % (self.n_amcl, len(use_d), span))
        print('  定位更新率 %.2f Hz   (配对成功率 %.0f%%)'
              % (len(use_d) / span if span > 0 else 0,
                 100.0 * len(self.samples) / max(1, self.n_amcl)))
        print('  位置误差  中位 %.4f m   均值 %.4f m   P90 %.4f m   最大 %.4f m'
              % (r['med'], r['mean'], r['p90'], r['mx']))
        print('  朝向误差  中位 %.2f deg  P90 %.2f deg' % (r['yaw_med'], r['yaw_p90']))
        return r


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, default=0.0, help='测多少秒, 0=一直测')
    ap.add_argument('--csv', help='把逐帧数据存成 csv')
    a = ap.parse_args()

    rospy.init_node('check_localization', anonymous=True)
    m = Measurer(a.csv)
    print('开始测量 (只订阅, 不发指令) ...')
    try:
        if a.duration > 0:
            rospy.sleep(a.duration)
        else:
            while not rospy.is_shutdown():
                rospy.sleep(0.5)
    except KeyboardInterrupt:
        pass
    print('=== 定位误差 (AMCL vs 真值) ===')
    m.report()
    if m.csv:
        m.csv.close()
        print('逐帧数据 ->', a.csv)


if __name__ == '__main__':
    sys.exit(main())
