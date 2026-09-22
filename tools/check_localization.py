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
import time
from collections import deque

import rospy
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
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
    """量 AMCL 定位误差。

    ★ issue #7 的教训: **不要**为了"静止时也能采到样本"去改 AMCL 的
      update_min_d/a (设 0.0 会让粒子集过早收敛, 导航落点系统性偏差 +81%)。
      正确做法是换个数据源 —— 用 **TF 的 map->base_footprint**:
      AMCL 持续维护这条变换, 车静止时也有值, 比 /amcl_pose 话题可靠。

    取数方式: /odom_groundtruth (50Hz) 驱动, 每条真值去取一次当前定位位姿,
    限速到 rate_limit Hz。定位位姿优先 TF, 拿不到才退回最近一条 /amcl_pose。
    """

    def __init__(self, csv_path=None, rate_limit=10.0):
        self.amcl = None
        self.samples = []         # (t, err_xy, err_yaw_deg)
        self.n_amcl = 0           # 收到多少条 /amcl_pose
        self.n_tf = 0             # 有多少条样本来自 TF
        self.n_truth = 0          # 收到多少条真值
        self.last_t = 0.0
        self.rate_limit = rate_limit
        self.tf = tf2_ros.Buffer(cache_time=rospy.Duration(15.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf)
        self.csv = open(csv_path, 'w') if csv_path else None
        if self.csv:
            self.csv.write('t,est_x,est_y,gt_x,gt_y,err_xy,err_yaw_deg,src\n')

        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.cb_amcl, queue_size=20)
        rospy.Subscriber('/odom_groundtruth', Odometry, self.cb_truth, queue_size=50)

    def pose_now(self):
        """当前定位位姿 -> (x, y, yaw, 'tf'|'amcl'); 都拿不到返回 None"""
        try:
            tr = self.tf.lookup_transform('map', 'base_footprint',
                                          rospy.Time(0), rospy.Duration(0.05))
            p, q = tr.transform.translation, tr.transform.rotation
            return (p.x, p.y, yaw_of(q), 'tf')
        except Exception:
            if self.amcl is not None:
                return (self.amcl[1], self.amcl[2], self.amcl[3], 'amcl')
            return None

    def cb_amcl(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.amcl = (m.header.stamp.to_sec(), p.x, p.y, yaw_of(q))
        self.n_amcl += 1        # 只作为兜底/诊断, 不再用它驱动采样

    def cb_truth(self, m):
        self.n_truth += 1
        p, q = m.pose.pose.position, m.pose.pose.orientation
        t = m.header.stamp.to_sec()
        if t - self.last_t < 1.0 / self.rate_limit:      # 限速
            return
        est = self.pose_now()
        if est is None:
            return
        self.last_t = t
        if est[3] == 'tf':
            self.n_tf += 1
        exy = math.hypot(est[0] - p.x, est[1] - p.y)
        eyaw = math.degrees(wrap(est[2] - yaw_of(q)))
        self.samples.append((t, exy, eyaw))
        if self.csv:
            self.csv.write('%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.3f,%s\n'
                           % (t, est[0], est[1], p.x, p.y, exy, eyaw, est[3]))

    def report(self):
        d = [s[1] for s in self.samples]
        yaw = [abs(s[2]) for s in self.samples]
        if not d:
            print('  没采到样本 —— 分数据源看:')
            print('    /odom_groundtruth 收到 %d 条 %s' % (self.n_truth,
                  '' if self.n_truth else '**一条都没收到 -> 仿真没起?**'))
            print('    定位位姿源: TF(map->base_footprint) %s; /amcl_pose 收到 %d 条'
                  % ('可用' if self.n_tf else '不可用', self.n_amcl))
            print('  正常情况走 TF, 车静止也有值; 若两者都没有, 检查 AMCL 是否起来了')
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
        print('  真值 %d 条, 成功配对 %d 条, 跨度 %.1f s   (定位源: TF %d / /amcl_pose %d)'
              % (self.n_truth, len(use_d), span,
                 self.n_tf, len(self.samples) - self.n_tf))
        print('  采样率 %.2f Hz' % (len(use_d) / span if span > 0 else 0))
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
    print('开始测量 ...')
    # ★ issue #3 兜底 (现在走 TF, 基本用不到): 万一 TF 也拿不到 (AMCL 没起来),
    #   等 6 秒还没样本就自己原地轻转一下, 逼 AMCL 动起来。
    nudge = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    need_nudge = True
    try:
        if a.duration > 0:
            t0 = time.time()
            nudged = False
            while time.time() - t0 < a.duration:
                rospy.sleep(0.2)
                if (not nudged) and (time.time() - t0 > 6.0) and len(m.samples) == 0:
                    print('  6 秒没收到 /amcl_pose —— 原地轻转一下触发 AMCL 更新')
                    print('  (AMCL 的 update_min_d/a 决定"车不动就不更新"; 见 amcl_params.yaml)')
                    tw = Twist()
                    tw.angular.z = 0.25
                    for _ in range(10):                 # 约 1 秒
                        nudge.publish(tw)
                        rospy.sleep(0.1)
                    nudge.publish(Twist())
                    nudged = True
        else:
            while not rospy.is_shutdown():
                rospy.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        nudge.publish(Twist())
    print('=== 定位误差 (AMCL vs 真值) ===')
    m.report()
    if m.csv:
        m.csv.close()
        print('逐帧数据 ->', a.csv)


if __name__ == '__main__':
    sys.exit(main())
