#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""验证「激光对墙定位」的精度: 把 laser_wall_pose() 和 Gazebo 真值逐帧对比。

用法:
    tools/check_laser_pose.py                     # 测 12 次, 每次间隔 1 秒
    tools/check_laser_pose.py --n 20 --period 0.5

前提: 仿真在跑 (roslaunch competition_robot navigation.launch),
      并且车**停在靠墙/角落**的位置 —— 激光对墙只在雷达能看到墙时有效
      (本场地四面都是墙, 所以基本哪都行; 但离墙太远时可用点会变少)。

为什么要单独测这个: laser_wall_pose() 是倒车入库的位姿来源, 它的精度直接
决定入位精度。它是"用地图已知的墙位置 + 激光测距"反推绝对位姿, 不经过
AMCL 粒子滤波, 也不受里程计漂移影响。

参考值 (2026-09-18, 车停在出生点 (1.772, 1.782)):
    激光对墙   误差 2 ~ 5 mm
    AMCL       误差 13.4 mm
    => 倒车入库终点真值误差 1.0 cm (跑 2 遍: 1.03 / 1.08 cm)
"""
from __future__ import annotations

import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rospy                                                # noqa: E402
import patrol as P                                          # noqa: E402


def pct(v, p):
    if not v:
        return float('nan')
    s = sorted(v)
    return s[min(len(s) - 1, max(0, int(round(p / 100.0 * (len(s) - 1)))))]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--n', type=int, default=12, help='采样次数, 默认 12')
    ap.add_argument('--period', type=float, default=1.0, help='采样间隔秒, 默认 1.0')
    ap.add_argument('--csv', help='把逐次结果存成 csv')
    a = ap.parse_args()

    rospy.init_node('check_laser_pose', anonymous=True)
    # 只需要它的订阅和 laser_wall_pose(), 这些参数随便给
    dummy = type('A', (), {'pre_rotate': True, 'max_w': 1.0, 'k_w': 1.2,
                           'timeout': 60.0, 'goal_includes_yaw': False,
                           'park_pos_tol': 0.012, 'park_yaw_tol': 0.02})()
    p = P.Patrol(dummy)

    print('等数据 (雷达 + TF + AMCL) ...')
    rospy.sleep(4.0)

    csv = open(a.csv, 'w') if a.csv else None
    if csv:
        csv.write('gt_x,gt_y,lw_x,lw_y,amcl_x,amcl_y,lw_err,amcl_err\n')

    print()
    print('%-22s %-22s %-22s %s' % ('真值位置', '激光对墙', 'AMCL', '激光误差 / AMCL误差'))
    print('-' * 88)
    lw_e, am_e, miss = [], [], 0
    for _ in range(a.n):
        lw, am, gt = p.laser_wall_pose(), p.pose(), p.gt
        if lw and am and gt:
            e1 = math.hypot(lw[0] - gt[0], lw[1] - gt[1])
            e2 = math.hypot(am[0] - gt[0], am[1] - gt[1])
            lw_e.append(e1)
            am_e.append(e2)
            print('(%+.4f,%+.4f)  (%+.4f,%+.4f)  (%+.4f,%+.4f)   %6.1f mm / %6.1f mm'
                  % (gt[0], gt[1], lw[0], lw[1], am[0], am[1], e1 * 1000, e2 * 1000))
            if csv:
                csv.write('%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.5f,%.5f\n'
                          % (gt[0], gt[1], lw[0], lw[1], am[0], am[1], e1, e2))
        else:
            miss += 1
            print('  取不到位姿 (激光对墙返回 None? 离墙太远或没雷达数据)')
        rospy.sleep(a.period)

    print('-' * 88)
    if lw_e:
        print('激光对墙  中位 %.1f mm  均值 %.1f mm  最大 %.1f mm  (%d 次有效, %d 次失败)'
              % (pct(lw_e, 50) * 1000, sum(lw_e) / len(lw_e) * 1000,
                 max(lw_e) * 1000, len(lw_e), miss))
        print('AMCL      中位 %.1f mm  均值 %.1f mm  最大 %.1f mm'
              % (pct(am_e, 50) * 1000, sum(am_e) / len(am_e) * 1000, max(am_e) * 1000))
        ratio = (pct(am_e, 50) / pct(lw_e, 50)) if pct(lw_e, 50) > 0 else 0
        print('=> 激光对墙比 AMCL 准 %.1f 倍' % ratio)
    else:
        print('没有有效样本')
    if csv:
        csv.close()
        print('逐次数据 ->', a.csv)
    return 0


if __name__ == '__main__':
    sys.exit(main())
