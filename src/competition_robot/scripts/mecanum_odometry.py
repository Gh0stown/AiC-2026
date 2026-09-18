#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mecanum_odometry.py —— 麦轮轮式里程计节点。**仿真和真机共用同一个节点。**

订阅  /joint_states   (四个轮子的转角, rad)
发布  /odom           (nav_msgs/Odometry)
发布  TF odom -> base_footprint

真机上: 底层驱动把编码器计数换算成关节转角, 发到同一个 /joint_states,
本节点原封不动复用 —— 这正是 sim2ros 想达到的效果:
仿真里调好的定位/导航参数, 搬到真车不需要改。

参数从 /robot_params (config/robot_params.yaml) 读取。
"""

from __future__ import annotations

import math
import os
import sys

import rospy
import tf2_ros
import yaml
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty, EmptyResponse

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mecanum import MecanumKinematics, from_params  # noqa: E402


def yaw_to_quaternion(yaw):
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class MecanumOdometry(object):
    def __init__(self):
        params = rospy.get_param('/robot_params')
        self.kin = from_params(params)
        F = params['robot']['frames']
        self.odom_frame = rospy.get_param('~odom_frame', F['odom'])
        self.base_frame = rospy.get_param('~base_frame', F['base_footprint'])
        self.wheel_joints = ['%s_wheel_joint' % t for t in self.kin.WHEELS]

        # ★ 初值必须和车的出生位姿一致!
        #   里程计是从这里开始积分的, 如果出生在别处而这里写死 0,
        #   /odom 就会整体偏出去, 建图/定位全废。
        #   launch 里会把 x/y/yaw 参数传进来。
        init = rospy.get_param('~initial_pose', [0.0, 0.0, 0.0])
        self.init = init
        self.x = float(init[0])
        self.y = float(init[1])
        self.th = float(init[2])
        rospy.loginfo('mecanum_odometry: 初始位姿 (%.3f, %.3f, %.3f rad)'
                      % (self.x, self.y, self.th))
        self.last_pos = None
        self.last_time = None

        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.sub = rospy.Subscriber('joint_states', JointState, self.cb, queue_size=20)
        # 复位里程计 (仿真里重置模型后调一下; 真机上开机/重新定位时也用得上)
        self.reset_srv = rospy.Service('~reset', Empty, self.reset)
        rospy.loginfo('mecanum_odometry: 轮径 %.4f 轴距 %.4f 轮距 %.4f, %s -> %s'
                      % (self.kin.r, self.kin.wheelbase, self.kin.track,
                         self.odom_frame, self.base_frame))

    def reset(self, _req):
        self.x = float(self.init[0])
        self.y = float(self.init[1])
        self.th = float(self.init[2])
        self.last_pos = None
        self.last_time = None
        rospy.loginfo('mecanum_odometry: 里程计已复位')
        return EmptyResponse()

    def _index(self, msg):
        """把 joint_states 里的四个轮子取出来, 缺一个就返回 None。"""
        try:
            return [msg.position[msg.name.index(j)] for j in self.wheel_joints]
        except (ValueError, IndexError):
            return None

    def cb(self, msg):
        pos = self._index(msg)
        if pos is None:
            return
        t = msg.header.stamp if not msg.header.stamp.is_zero() else rospy.Time.now()

        if self.last_pos is None:
            self.last_pos, self.last_time = pos, t
            return

        dt = (t - self.last_time).to_sec()
        if dt <= 0.0:
            return
        if dt > 0.5:                       # 时间跳变 (仿真重置 / 卡顿), 丢掉这一帧
            self.last_pos, self.last_time = pos, t
            return

        # 关节转角增量 -> 轮角速度
        w = [(pos[i] - self.last_pos[i]) / dt for i in range(4)]
        vx, vy, wz = self.kin.forward(*w)
        self.last_pos, self.last_time = pos, t

        # 用中点法积分 (比欧拉法准, 尤其是边走边转的时候)
        dth = wz * dt
        th_mid = self.th + dth / 2.0
        self.x += (vx * math.cos(th_mid) - vy * math.sin(th_mid)) * dt
        self.y += (vx * math.sin(th_mid) + vy * math.cos(th_mid)) * dt
        self.th = math.atan2(math.sin(self.th + dth), math.cos(self.th + dth))

        self.publish(t, vx, vy, wz)

    def publish(self, t, vx, vy, wz):
        q = yaw_to_quaternion(self.th)

        odom = Odometry()
        odom.header.stamp = t
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = q
        # 麦轮是全向的, 协方差按 vx/vy/wz 三个方向都给
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        for i in (0, 7, 14, 21, 28, 35):
            odom.pose.covariance[i] = 1e-3
            odom.twist.covariance[i] = 1e-3
        self.odom_pub.publish(odom)

        tfm = TransformStamped()
        tfm.header.stamp = t
        tfm.header.frame_id = self.odom_frame
        tfm.child_frame_id = self.base_frame
        tfm.transform.translation.x = self.x
        tfm.transform.translation.y = self.y
        tfm.transform.translation.z = 0.0
        tfm.transform.rotation = q
        self.tf_broadcaster.sendTransform(tfm)


def main():
    rospy.init_node('mecanum_odometry')
    if not rospy.has_param('/robot_params'):
        # 方便单机调试: 直接从包里读 YAML
        p = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         '..', 'config', 'robot_params.yaml')
        with open(p) as fp:
            rospy.set_param('/robot_params', yaml.safe_load(fp))
        rospy.logwarn('参数服务器没有 /robot_params, 已从 %s 载入' % p)
    MecanumOdometry()
    rospy.spin()


if __name__ == '__main__':
    main()
