#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sim_wheel_encoders.py —— **仅仿真用**: 把 /cmd_vel 变成四个轮子的关节状态。

仿真里机器人由 planar_move 直接驱动, 轮子大多在打滑, Gazebo 自己的关节角度
不能当作编码器用。所以这里用和真机**同一套**麦克纳姆逆运动学, 把速度指令
积分成四个轮子的转角, 发成 /joint_states —— 相当于一个"理想编码器"。

真机上这个节点不存在: 底层驱动读真实编码器, 发到同一个 /joint_states。
下游的 mecanum_odometry.py 两边通用。

参数:
    ~rate       发布频率, 默认 50 Hz (要和真机编码器上报频率对齐)
    ~quantize   是否按编码器线数做量化, 默认 false
    ~noise_std  转角噪声标准差 (rad), 默认 0
"""

from __future__ import annotations

import math
import os
import random
import sys

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mecanum import MecanumKinematics, from_params  # noqa: E402


class SimWheelEncoders(object):
    def __init__(self):
        params = rospy.get_param('/robot_params')
        m = params['mecanum']
        enc = params.get('encoder', {})
        self.kin = from_params(params)

        self.rate = float(rospy.get_param('~rate', 50.0))
        # 和 planar_move 的 <cmdTimeout> 保持一致: 指令停了, 轮子也要停。
        # 否则上一条指令的速度会被一直积分下去, 里程计会越跑越偏。
        self.cmd_timeout = float(rospy.get_param('~cmd_timeout', 0.5))
        self.last_cmd = None
        self.quantize = bool(rospy.get_param('~quantize', False))
        self.noise_std = float(rospy.get_param('~noise_std', 0.0))

        self.cpr = MecanumKinematics.counts_per_wheel_rev(
            enc.get('counts_per_rev', 0), enc.get('gear_ratio', 0),
            enc.get('quadrature', 1))

        self.vel = (0.0, 0.0, 0.0)
        self.angle = [0.0] * 4
        self.names = ['%s_wheel_joint' % t for t in self.kin.WHEELS]

        # 速度来源:
        #   groundtruth (默认) = 用 /odom_groundtruth 里 planar_move 实测的车体速度。
        #       这样编码器反映的是车"实际"怎么动的, 里程计和真值一致。
        #       重要: planar_move 实际达到的角速度只有指令的约 0.637 倍(见 README),
        #       所以这里必须用实测值, 用指令值的话里程计会偏大 1/0.637 倍。
        #   cmd_vel = 直接用指令 (理想编码器), 适合单独测里程计积分。
        self.source = rospy.get_param('~source', 'groundtruth')
        self.pub = rospy.Publisher('joint_states', JointState, queue_size=10)
        self.sub = rospy.Subscriber('cmd_vel', Twist, self.cb, queue_size=10)
        self.sub_gt = rospy.Subscriber('odom_groundtruth', Odometry, self.cb_gt,
                                       queue_size=10)
        self.gt_vel = None
        self.last_gt = None
        rospy.loginfo('sim_wheel_encoders: %.0f Hz, 量化=%s, 车轮一圈 %d 计数'
                      % (self.rate, self.quantize, round(self.cpr)))

    def cb(self, msg):
        self.vel = (msg.linear.x, msg.linear.y, msg.angular.z)
        self.last_cmd = rospy.Time.now()

    def cb_gt(self, msg):
        self.gt_vel = (msg.twist.twist.linear.x, msg.twist.twist.linear.y,
                       msg.twist.twist.angular.z)
        self.last_gt = rospy.Time.now()

    def spin(self):
        r = rospy.Rate(self.rate)
        last = rospy.Time.now()
        last_pub = None
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            dt = (now - last).to_sec()
            last = now
            if dt <= 0 or dt > 0.5:
                r.sleep()
                continue

            if self.source == 'groundtruth':
                # 用实测速度; 实测也停了就停
                fresh = (self.last_gt is not None and
                         (now - self.last_gt).to_sec() <= self.cmd_timeout)
                v = self.gt_vel if (fresh and self.gt_vel) else (0.0, 0.0, 0.0)
            else:
                # 指令超时 -> 当作停车, 和 planar_move 的行为对齐
                stale = (self.last_cmd is None or
                         (now - self.last_cmd).to_sec() > self.cmd_timeout)
                v = (0.0, 0.0, 0.0) if stale else self.vel
            w = self.kin.inverse(*v)
            for i in range(4):
                self.angle[i] += w[i] * dt

            pos = list(self.angle)
            if self.quantize and self.cpr > 0:
                step = 2.0 * math.pi / self.cpr
                pos = [round(a / step) * step for a in pos]
            if self.noise_std > 0.0:
                pos = [a + random.gauss(0.0, self.noise_std) for a in pos]

            # 仿真时间没推进就不发 —— 否则会出现同一时间戳的重复消息,
            # 下游 TF 会报 TF_REPEATED_DATA
            if last_pub is not None and now == last_pub:
                r.sleep()
                continue
            last_pub = now

            js = JointState()
            js.header.stamp = now
            js.name = list(self.names)
            js.position = pos
            js.velocity = list(w)
            js.effort = []
            self.pub.publish(js)
            r.sleep()


def main():
    rospy.init_node('sim_wheel_encoders')
    SimWheelEncoders().spin()


if __name__ == '__main__':
    main()
