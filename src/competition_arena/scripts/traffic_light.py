#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""临时红绿灯（物理切换版, A 方案）—— 等比赛方物料来了只换材质/尺寸。

结构与原理
----------
每盏灯 = **一个 URDF 模型**: 世界锚定 joint + 灯箱/立柱/透镜 + 3 个灯珠 link,
灯珠挂在**平移关节**上, 在两个位置之间移动:
    ON  = +0.05 m  (伸出灯箱正面透镜外 → 亮着, 看得见)
    OFF = -0.03 m  (缩进灯箱内部 → 被不透光灯箱挡住, 相机也看不见)
切换 = 调 /gazebo/set_model_configuration 设 3 个关节位置。

为什么这么做（三种做法都实测过）:
  1) 发 Gazebo 的 `~/visual` 改材质: 无 GUI 时该话题**没有订阅者**, 只影响 RViz,
     **服务器端相机看不到** ✗ (红绿灯要能被相机识别, 所以不能用)
  2) `<static>true</static>` 的灯珠用 SetModelState 挪: 位姿变了但**渲染不跟随** ✗
  3) `<kinematic>true</kinematic>`: 本机 sdformat 不认, 灯珠直接掉地上 ✗
  => 只有"关节运动"是 Gazebo 一定会正确渲染的机制 ✓ (robustify/gazebo_traffic_light
     那个插件用的也是 joint->SetPosition)

高度/距离按我们相机算过: 相机高 0.20 m、竖直半视场 ±22.6°、近裁剪 0.6 m,
灯心 0.55 m => 距离 ≥0.84 m 能看见、<0.6 m 完全看不到 => 停车线放 1.2~1.5 m 正合适。

话题
----
    ~/state     (std_msgs/String)  当前状态: red / yellow / green / off
    ~/command   (std_msgs/String)  手动覆盖: auto / red / yellow / green / off
参数
----
    ~lights    [{name, x, y, z, yaw, layout}]   layout = vertical | horizontal
    ~sequence  [{color, duration}, ...]         默认 绿15s -> 黄3s -> 红10s (与 config/traffic_lights.yaml 一致)
"""
from __future__ import annotations

import os
import threading

import rospy
import yaml
from gazebo_msgs.srv import SetModelConfiguration
from std_msgs.msg import String

# ---- 尺寸 (m) —— 与 tools/setup_traffic_lights.py 保持一致 (官方尺寸) ----
LAMP_R = 0.0425        # 灯珠半径 (直径 8.5 cm, 官方尺寸图量得)
LAMP_T = 0.008
LENS_R = 0.0425        # 常驻暗透镜半径 (与灯珠同径)
BOX_D = 0.080          # 灯箱厚度 (x, 正面方向)
BOX_W = 0.640          # 灯箱宽 (官方 64 cm)
BOX_H = 0.140          # 灯箱高 (官方 14 cm)
# 关节位置: 亮 = 把发光灯珠推到常驻透镜前方(看得见)
#           灭 = 缩回灯箱内部(被不透光灯箱挡住)
# 行程刻意做得很小(4cm), 所以从画面上看"灯珠并没有明显伸缩" —— 只是亮/暗与
# 颜色变化, 接近真实红绿灯。这也是为了不让识别模型学到"位置"这种伪特征。
ON, OFF = 0.050, 0.010


class Node(object):
    def __init__(self):
        cfg_path = rospy.get_param('~config', '')
        if cfg_path and os.path.exists(cfg_path):
            cfg = yaml.safe_load(open(cfg_path))
        else:
            rospy.logwarn('没给 ~config, 用内置默认')
            # 兜底默认 —— 与 config/traffic_lights.yaml 保持一致 (绿15/黄3/红10)
            cfg = {'lights': [{'name': 'tl_top'}, {'name': 'tl_bot'}],
                   'sequence': [{'color': 'green', 'duration': 15.0},
                                {'color': 'yellow', 'duration': 3.0},
                                {'color': 'red', 'duration': 10.0}]}
        self.lights = [L['name'] for L in cfg['lights']]
        self.seq = [(e['color'], float(e['duration'])) for e in cfg['sequence']]
        self.cycle = sum(d for _, d in self.seq)
        self.override = 'auto'
        self.lock = threading.Lock()
        self.cur = None
        self.t0 = None

        self.pub = rospy.Publisher('~state', String, queue_size=1, latch=True)
        rospy.Subscriber('~command', String, self.cb_cmd, queue_size=1)
        self.setcfg = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)
        rospy.loginfo('等待 /gazebo/set_model_configuration ...')
        self.setcfg.wait_for_service(timeout=90.0)
        rospy.loginfo('  灯: %s (模型随 world 一起加载, 运行时只切关节)' % ', '.join(self.lights))
        self.t0 = rospy.Time.now()
        rospy.Timer(rospy.Duration(0.2), self.tick)
        rospy.Timer(rospy.Duration(0.2), self.pub_state)

    def switch(self, color):
        for name in self.lights:
            names = ['j_red', 'j_yellow', 'j_green']
            pos = [ON if c == color else OFF for c in ('red', 'yellow', 'green')]
            try:
                self.setcfg(model_name=name, urdf_param_name='',
                            joint_names=names, joint_positions=pos)
            except Exception as e:
                rospy.logwarn_throttle(5.0, '切换 %s 失败: %s' % (name, e))
            rospy.sleep(0.05)

    def auto_color(self):
        if self.t0 is None:
            return self.seq[0][0]
        t = (rospy.Time.now() - self.t0).to_sec() % self.cycle
        acc = 0.0
        for color, dur in self.seq:
            acc += dur
            if t < acc:
                return color
        return self.seq[-1][0]

    def cur_color(self):
        with self.lock:
            ov = self.override
        return ov if (ov in ('red', 'yellow', 'green') or ov == 'off') else self.auto_color()

    def cb_cmd(self, msg):
        with self.lock:
            self.override = msg.data.strip().lower()
        rospy.loginfo('覆盖命令 -> %s' % self.override)

    def tick(self, _ev):
        c = self.cur_color()
        if c != self.cur:
            self.cur = c
            self.switch(c)
            rospy.loginfo('红绿灯 -> %s' % c)

    def pub_state(self, _ev):
        self.pub.publish(String(data=self.cur_color()))


def main():
    rospy.init_node('traffic_light')
    Node()
    rospy.spin()


if __name__ == '__main__':
    main()
