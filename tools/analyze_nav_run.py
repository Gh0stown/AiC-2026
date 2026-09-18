#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""分析一次导航跑得"直不直、丝不丝滑"。由 tools/bench_nav.sh 调用。

输入: 一个包含 真值轨迹 (x, y, yaw, t) 和 目标 的 pickle。
输出:
  横向偏差   —— 轨迹偏离起点-终点直线的最大值 (走直线最核心的指标)
  蛇形指数   —— 横向速度符号翻转次数 / 100 个点 (越小越直)
  转向平滑度 —— 角速度变化率的 RMS (越小越丝滑; 原地猛转会很大)
  到点前原地转 —— 最后 1 m 内, 只有转没有走的时长 (你现在遇到的问题)
"""
from __future__ import annotations

import math
import pickle
import sys

import numpy as np


def main():
    d = pickle.load(open(sys.argv[1], 'rb'))
    tr = np.array([(r[0], r[1], r[2], r[3]) for r in d['gt']])   # t,x,y,yaw
    gx, gy = d['goal'][0], d['goal'][1]
    if len(tr) < 50:
        print('  轨迹太短, 没法分析'); return 1
    # 起点取"开始动"的点
    x0, y0 = tr[0, 1], tr[0, 2]
    for r in tr:
        if math.hypot(r[1] - x0, r[2] - y0) > 0.05:
            sx, sy = r[1], r[2]
            break
    else:
        sx, sy = x0, y0
    # 单位方向 + 法向
    dx, dy = gx - sx, gy - sy
    L = math.hypot(dx, dy)
    ux, uy = dx / L, dy / L
    nx, ny = -uy, ux
    lat = (tr[:, 1] - sx) * nx + (tr[:, 2] - sy) * ny      # 横向偏差(带符号)
    lon = (tr[:, 1] - sx) * ux + (tr[:, 2] - sy) * uy      # 纵向进度

    # 速度/角速度 (用真值差分, 单位: 仿真秒)
    dt = np.diff(tr[:, 0]); dt[dt <= 1e-6] = 1e-6
    vx = np.diff(tr[:, 1]) / dt
    vy = np.diff(tr[:, 2]) / dt
    w = np.diff(np.unwrap(tr[:, 3])) / dt

    def sign_flips(a, thr=0.02):
        s = np.sign(a[np.abs(a) > thr])
        return int((s[1:] * s[:-1] < 0).sum()) if len(s) > 1 else 0

    # 车体系横向速度 (麦轮横移会让它明显非零) —— "看着像不像在开车"
    yaw = tr[:-1, 3]
    vy_body = -np.sin(yaw) * vx + np.cos(yaw) * vy      # 车体左右方向的速度
    strafe_frac = float(np.mean(np.abs(vy_body) > 0.05))

    # 到点前原地转: 纵向进度 > L-1.0 之后, |v|<0.03 但 |w|>0.2 的时间
    tail = lon > max(0.0, L - 1.0)
    spin = float(np.sum(dt[tail[:-1] & (np.hypot(vx, vy) < 0.03) & (np.abs(w) > 0.2)]))

    print('  总时长            %.1f s' % (tr[-1, 0] - tr[0, 0]))
    print('  横向偏差 (最大)    %.3f m     <- 走直线最核心, 越小越直' % np.abs(lat).max())
    print('  横向偏差 (RMS)     %.3f m' % np.sqrt((lat ** 2).mean()))
    print('  蛇形指数          %d 次方向翻转 / %d 点  (%.1f 次每百点)'
          % (sign_flips(vy), len(vy), 100.0 * sign_flips(vy) / max(len(vy), 1)))
    print('  角速度变化率 RMS   %.2f rad/s²  <- 越小越丝滑' % np.sqrt(np.mean(np.diff(w) ** 2)))
    print('  横移(车体系)       RMS %.3f m/s, 最大 %.3f m/s, 占比 %.0f%%  <- 越大越"不像在开车"'
          % (np.sqrt((vy_body ** 2).mean()), np.abs(vy_body).max(), 100 * strafe_frac))
    tail1 = lon > max(0.0, L - 1.0)
    tail05 = lon > max(0.0, L - 0.5)
    print('  末端 1 m 横向偏差   %.3f m     <- 快到终点时有没有偏出去' %
          (np.abs(lat[tail1]).max() if tail1.any() else 0.0))
    print('  末端 0.5 m 横向偏差 %.3f m' %
          (np.abs(lat[tail05]).max() if tail05.any() else 0.0))
    print('  到点前原地转       %.1f s       <- 你遇到的"原地转圈找方向"' % spin)
    print('  最终位置误差       %.3f m (真值 vs 目标; 含地图系偏差)'
          % math.hypot(tr[-1, 1] - gx, tr[-1, 2] - gy))
    # 定位偏差: AMCL 估计 vs 真值 (决定"离目标还差多少就宣布到点")
    am = d.get('amcl') or []
    if am:
        e = []
        for r in am:
            g = tr[np.argmin(np.abs(tr[:, 0] - r[0]))]
            e.append(math.hypot(r[1] - g[1], r[2] - g[2]))
        e = np.array(e)
        print('  定位偏差(AMCL-真值) 中位 %.3f m, 最大 %.3f m' % (np.median(e), e.max()))

    # 全局路径本身直不直? (如果它也弯, 那就是全局规划器的问题, 不怪局部规划器)
    pl = d.get('plan') or []
    if len(pl) > 5:
        px = np.array([p[0] for p in pl]); py = np.array([p[1] for p in pl])
        lat_p = (px - sx) * nx + (py - sy) * ny
        print('  ── 全局路径 (%s, %d 点): 横向偏差最大 %.3f m, RMS %.3f m'
              % (d.get('plan_topic', '?'), len(pl), np.abs(lat_p).max(),
                 np.sqrt((lat_p ** 2).mean())))
        if np.abs(lat_p).max() > 0.03 and np.abs(lat_p).max() > 2 * np.abs(lat).max():
            print('     ^ 路径比机器人走的还弯 -> 可以试试换全局规划器: gplan:=global')
        elif np.abs(lat_p).max() > 0.03:
            print('     (参考线是"起点-终点直线"; 要转弯的场景本来就该偏, 正常)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
