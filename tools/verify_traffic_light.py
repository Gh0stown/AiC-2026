#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""验证红绿灯切换 —— 量灯珠沿"灯箱正面方向"的伸出量, **不依赖相机渲染**。

用法:
    roslaunch competition_robot navigation.launch     # 另开终端 (或 robot_gazebo.launch)
    tools/verify_traffic_light.py

为什么量位姿而不是看图:
    沙箱里相机是软件渲染, 画面可能冻住 —— "三张状态图"根本看不出区别。
    这个工具直接查 Gazebo 里灯珠 link 的位姿, 换算成"伸出量":
        0.010 m = 缩进灯箱内部(灭, 被不透光灯箱挡住)
        0.050 m = 推到常驻透镜前方(亮, 看得见)
    这样任何机器上都能验证, 不受渲染影响。

    注: 灯箱关重力悬放 2mm, 量到的绝对值可能整体偏一点; 只要**三颗之间的相对关系**
    正确就没问题 (指令差 0.04, 量出来也差 0.04)。

★ issue #2 的修法: 优先**通过切换节点**驱动 (发 /traffic_light/command),
  而不是自己直接写 set_model_configuration —— 否则本脚本和 traffic_light.py
  会有两个写入者周期性互相覆盖, 验收第 2 项会间歇性失败。
  节点不在时才退回直接写 (并提示风险)。测完会把状态交还 'auto'。
"""
from __future__ import annotations

import math
import os
import sys
import time

import rospy
import yaml
from gazebo_msgs.srv import GetLinkState, SetModelConfiguration
from std_msgs.msg import String

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CFG = os.path.join(WS, 'src', 'competition_arena', 'config', 'traffic_lights.yaml')
JOINTS = ['j_red', 'j_yellow', 'j_green']
IDX = {'red': 0, 'yellow': 1, 'green': 2}
ON, OFF = 0.050, 0.010
COLORS = ('red', 'yellow', 'green')


def main():
    rospy.init_node('verify_traffic_light', anonymous=True)
    setc = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)
    getl = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
    try:
        rospy.wait_for_service('/gazebo/set_model_configuration', timeout=20)
        rospy.wait_for_service('/gazebo/get_link_state', timeout=20)
    except rospy.ROSException:
        print('找不到 Gazebo 服务 —— 先把仿真起起来:')
        print('  roslaunch competition_robot navigation.launch')
        return 1

    # ---- 决定驱动方式 (优先走切换节点, 避免两个写入者) ----
    cmd = rospy.Publisher('/traffic_light/command', String, queue_size=1)
    t0 = time.time()
    while cmd.get_num_connections() == 0 and time.time() - t0 < 3.0:
        rospy.sleep(0.1)
    use_node = cmd.get_num_connections() > 0

    def read_d(L):
        """三颗灯珠沿"灯箱正面"(模型局部 +x) 的伸出量"""
        mx, my = float(L['x']), float(L['y'])
        yaw = float(L.get('yaw', 0.0))
        out = {}
        for c in COLORS:
            r = getl(link_name='%s::lamp_%s' % (L['name'], c), reference_frame='')
            p = r.link_state.pose.position
            out[c] = (p.x - mx) * math.cos(yaw) + (p.y - my) * math.sin(yaw)
        return out

    def settle(L, timeout=3.0):
        """等三颗灯珠的伸出量稳定 (比固定 sleep 可靠; 节点是 0.2s 一个 tick)"""
        last, stable, t1 = None, 0, time.time()
        while time.time() - t1 < timeout:
            rospy.sleep(0.15)
            cur = tuple(round(read_d(L)[c], 4) for c in COLORS)
            if cur == last:
                stable += 1
                if stable >= 2:
                    return True
            else:
                stable = 0
            last = cur
        return False

    lights = yaml.safe_load(open(CFG))['lights']
    print('红绿灯切换验证 (按灯珠位姿, 不依赖相机)')
    if use_node:
        print('  驱动方式: /traffic_light/command 命令切换节点 (唯一写入者) ✓')
    else:
        print('  ⚠ 没发现切换节点 —— 直接写 Gazebo 关节。')
        print('    若同时跑着 traffic_light.py, 两者会互相覆盖, 结果可能间歇性出错;')
        print('    建议先 roslaunch competition_robot navigation.launch')
    print()
    print('%-8s %-8s %-40s %s' % ('灯', '指令', '三颗灯珠伸出量 (米)', '判定'))
    print('-' * 98)

    bad = 0
    for L in lights:
        name = L['name']
        for st in COLORS:
            if use_node:
                cmd.publish(String(data=st))
            else:
                setc(model_name=name, urdf_param_name='', joint_names=JOINTS,
                     joint_positions=[ON if i == IDX[st] else OFF for i in range(3)])
            settle(L)
            time.sleep(0.3)
            d = read_d(L)
            lit = max(d, key=d.get)
            ok = (lit == st)
            bad += 0 if ok else 1
            print('%-8s %-8s %-40s %s'
                  % (name, st, '  '.join('%s=%.4f' % (c, d[c]) for c in COLORS),
                     ('✓ 亮的正是 %s' % lit) if ok else ('✗ 实际亮的是 %s' % lit)))
        print()

    # ★ issue #2 的 B 方案: 上面验的是"手动命令 -> 灯珠位姿"这条链;
    #   再顺带确认**循环节点自己在切** (否则把节点关了也看不出来)。
    cycle_bad = 0
    if use_node:
        seen = []
        sub = rospy.Subscriber('/traffic_light/state', String,
                               lambda m: seen.append(m.data.strip().lower()), queue_size=10)
        cmd.publish(String(data='auto'))
        t1 = time.time()
        while time.time() - t1 < 22.0 and len(set(seen)) < 3:
            rospy.sleep(0.3)
        sub.unregister()
        got = [c for c in ('green', 'yellow', 'red') if c in set(seen)]
        print('自动循环检查: 22 秒内观察到 %s' % (', '.join(got) if got else '(什么都没收到)'))
        if len(got) >= 2:
            print('  ✓ 循环节点在工作 (时序见 config/traffic_lights.yaml: 绿15s->黄3s->红10s)')
        else:
            print('  ✗ 只看到 %d 种状态 —— 检查 traffic_light 节点是否在跑' % len(got))
            cycle_bad = 1
        print()

    n = len(lights) * 3
    print('结论: %s' % ('✓ %d/%d 全部正确' % (n, n) if bad == 0 else '✗ %d/%d 不对' % (bad, n)))
    return 0 if (bad == 0 and cycle_bad == 0) else 1


if __name__ == '__main__':
    sys.exit(main())
