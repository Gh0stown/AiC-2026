#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""验证红绿灯切换 —— 量灯珠沿"灯箱正面方向"的伸出量, **不依赖相机渲染**。

用法:
    roslaunch competition_robot robot_gazebo.launch      # 另开终端
    tools/verify_traffic_light.py

为什么需要它:
    本项目的沙箱是软件渲染, **相机画面从启动起就冻住** —— 实测让车往前走 1 m,
    画面 md5 都不变, 所以"三张状态图"根本看不出区别(曾经因此误判过一次)。
    这个工具直接查 Gazebo 里灯珠 link 的位姿, 换算成"伸出量":
        0.010 m = 缩进灯箱内部(灭, 被不透光灯箱挡住)
        0.050 m = 推到常驻透镜前方(亮, 看得见)
    这样任何机器上都能验证切换机制是否正确, 不受渲染影响。

    注意: 模型是"落地"的(灯箱受重力), 启动后会有约 3 cm 的沉降, 所以量到的
    绝对值会比指令值大约 0.03~0.04; 但只要**三颗之间的相对关系**正确就没问题
    (指令值差 0.04, 量出来也差 0.04)。
"""
from __future__ import annotations

import math
import os
import sys
import time

import rospy
import yaml
from gazebo_msgs.srv import GetLinkState, SetModelConfiguration

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CFG = os.path.join(WS, 'src', 'competition_arena', 'config', 'traffic_lights.yaml')
JOINTS = ['j_red', 'j_yellow', 'j_green']
IDX = {'red': 0, 'yellow': 1, 'green': 2}
ON, OFF = 0.050, 0.010


def main():
    rospy.init_node('verify_traffic_light', anonymous=True)
    setc = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)
    getl = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
    try:
        rospy.wait_for_service('/gazebo/set_model_configuration', timeout=20)
        rospy.wait_for_service('/gazebo/get_link_state', timeout=20)
    except rospy.ROSException:
        print('找不到 Gazebo 服务 —— 先把仿真起起来:')
        print('  roslaunch competition_robot robot_gazebo.launch')
        return 1

    lights = yaml.safe_load(open(CFG))['lights']
    print('红绿灯切换验证 (按灯珠位姿, 不依赖相机)\n')
    print('%-8s %-8s %-36s %s' % ('灯', '指令', '三颗灯珠伸出量 (米)', '判定'))
    print('-' * 94)

    bad = 0
    for L in lights:
        name = L['name']
        mx, my, yaw = float(L['x']), float(L['y']), float(L.get('yaw', 0.0))
        for st in ('red', 'yellow', 'green'):
            setc(model_name=name, urdf_param_name='', joint_names=JOINTS,
                 joint_positions=[ON if i == IDX[st] else OFF for i in range(3)])
            time.sleep(0.7)
            d = {}
            for c in ('red', 'yellow', 'green'):
                r = getl(link_name='%s::lamp_%s' % (name, c), reference_frame='')
                p = r.link_state.pose.position
                # 把 (灯珠 - 模型原点) 投影到模型局部 +x 轴 = 灯箱正面方向
                d[c] = (p.x - mx) * math.cos(yaw) + (p.y - my) * math.sin(yaw)
            lit = max(d, key=d.get)
            ok = (lit == st)
            bad += 0 if ok else 1
            print('%-8s %-8s %-36s %s'
                  % (name, st,
                     '  '.join('%s=%.4f' % (c, d[c]) for c in ('red', 'yellow', 'green')),
                     ('✓ 亮的正是 %s' % lit) if ok else ('✗ 实际亮的是 %s' % lit)))
        print()
    n = len(lights) * 3
    print('结论: %s' % ('✓ %d/%d 全部正确' % (n, n) if bad == 0 else '✗ %d/%d 不对' % (bad, n)))
    return 0 if bad == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
