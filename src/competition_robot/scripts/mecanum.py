#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
麦克纳姆轮运动学 —— 仿真和真机**共用同一份实现**。

这是 sim2ros 的关键: 仿真里算轮速用的公式, 和真机上驱动节点算的是同一段代码,
所以仿真里调好的控制参数搬到真车不用重标。

轮序固定为  FL, FR, RL, RR  (左前, 右前, 左后, 右后)
坐标约定    +x 朝前, +y 朝左, +z 朝上, wz 逆时针为正

★ 支持**前后轮距不相等**的布局。
  实车 mowen2 就是这种情况: 前轮在 x=+0.1272, 后轮在 x=-0.0798
  (轮子整体前移了 2.4cm), 前后自转系数分别是 0.2726 和 0.2252, 差 21%。
  按对称轮距算的话, 自转指令会偏 10% 左右。

布局 (X 型, 俯视):

        FL ●────────● FR           +x
           │        │              ↑
           │        │              │
        RL ●────────● RR           └──→ +y
        |<-- xf -->|<-- xr -->|    (都在 base 原点两侧, 可以不等)
"""

from __future__ import annotations

import math


class MecanumKinematics(object):
    """四轮麦克纳姆底盘的正/逆运动学 (支持前后不对称)。"""

    WHEELS = ('fl', 'fr', 'rl', 'rr')

    def __init__(self, wheel_radius, wheelbase=None, track=0.0,
                 roller_angle_deg=45.0, front_axle_x=None, rear_axle_x=None):
        """两种构造方式:

        * 对称布局: MecanumKinematics(r, wheelbase, track)
        * 实测布局: MecanumKinematics(r, track=..., front_axle_x=..., rear_axle_x=...)
          (front_axle_x / rear_axle_x 是前后轮中心的 x, 相对 base 原点, 带符号)
        """
        if wheel_radius <= 0:
            raise ValueError('wheel_radius 必须为正')
        self.r = float(wheel_radius)
        self.roller_angle = math.radians(float(roller_angle_deg))
        self.track = float(track)
        self.ly = self.track / 2.0

        if front_axle_x is None or rear_axle_x is None:
            if wheelbase is None:
                raise ValueError('要么给 wheelbase, 要么给 front_axle_x/rear_axle_x')
            half = float(wheelbase) / 2.0
            front_axle_x, rear_axle_x = +half, -half
        self.xf = float(front_axle_x)          # 前轮 x (一般 > 0)
        self.xr = float(rear_axle_x)           # 后轮 x (一般 < 0)
        self.wheelbase = self.xf - self.xr

        # 每个轮子的自转系数 = |x_i| + ly   (关键: 前后可以不同)
        self.k_front = abs(self.xf) + self.ly
        self.k_rear = abs(self.xr) + self.ly
        # 兼容旧接口: 对称时的单一系数
        self.k = (self.k_front + self.k_rear) / 2.0

    # ----------------------------------------------------------------- 逆运动学
    def inverse(self, vx, vy, wz):
        """车体速度 -> 四个轮子的角速度 [rad/s]。返回 (w_fl, w_fr, w_rl, w_rr)。"""
        r, kf, kr = self.r, self.k_front, self.k_rear
        return (
            (vx - vy - kf * wz) / r,      # FL
            (vx + vy + kf * wz) / r,      # FR
            (vx + vy - kr * wz) / r,      # RL
            (vx - vy + kr * wz) / r,      # RR
        )

    # ----------------------------------------------------------------- 正运动学
    def forward(self, w_fl, w_fr, w_rl, w_rr):
        """四个轮子的角速度 [rad/s] -> 车体速度 (vx, vy, wz)。

        ★ 前后不对称(k_front != k_rear)时, vy 和 wz 是**耦合**的,
          不能像对称布局那样用两条独立公式。这里解的是
          [vx,vy,wz] = r * pinv(M) * w  (M 是逆运动学矩阵),
          对称时自动退化成常见的那组公式。
        """
        r = self.r
        kf, kr = self.k_front, self.k_rear

        vx = r * (w_fl + w_fr + w_rl + w_rr) / 4.0

        d = kf - kr
        s = kf * kf + kr * kr
        det = 4.0 * (2.0 * s - d * d)
        if abs(det) < 1e-15:                     # 退化保护
            return vx, 0.0, 0.0
        b1 = -w_fl + w_fr + w_rl - w_rr
        b2 = -kf * w_fl + kf * w_fr - kr * w_rl + kr * w_rr
        vy = r * (2.0 * s * b1 - 2.0 * d * b2) / det
        wz = r * (-2.0 * d * b1 + 4.0 * b2) / det
        return vx, vy, wz

    # ----------------------------------------------------------------- 限幅
    def limit(self, vx, vy, wz, max_vx, max_vy, max_wz):
        """按比例整体缩放, 保持运动方向不变 (比逐轴截断更平滑)。"""
        s = 1.0
        for val, lim in ((vx, max_vx), (vy, max_vy), (wz, max_wz)):
            if lim > 0 and abs(val) > lim:
                s = min(s, lim / abs(val))
        return vx * s, vy * s, wz * s

    def max_wheel_speed(self, vx, vy, wz):
        return max(abs(w) for w in self.inverse(vx, vy, wz))

    # ----------------------------------------------------------------- 轮子位置
    def wheel_positions(self):
        """四个轮子中心相对 base 原点的 (x, y)。"""
        return {
            'fl': (+self.xf, +self.ly),
            'fr': (+self.xf, -self.ly),
            'rl': (self.xr, +self.ly),
            'rr': (self.xr, -self.ly),
        }

    # ----------------------------------------------------------------- 编码器
    @staticmethod
    def counts_per_wheel_rev(counts_per_rev, gear_ratio, quadrature):
        """车轮转一整圈对应的编码器计数。"""
        return float(counts_per_rev) * float(gear_ratio) * float(quadrature)

    @staticmethod
    def wheel_angle_to_counts(angle_rad, counts_per_wheel_rev):
        return angle_rad / (2.0 * math.pi) * counts_per_wheel_rev

    @staticmethod
    def wheel_speed_to_rpm(w_rad_s):
        return w_rad_s * 60.0 / (2.0 * math.pi)

    @staticmethod
    def rpm_to_wheel_speed(rpm):
        return float(rpm) * 2.0 * math.pi / 60.0


def from_params(params):
    """从 robot_params.yaml 的字典直接构造。"""
    m = params['mecanum']
    return MecanumKinematics(
        m['wheel_radius'],
        wheelbase=m.get('wheelbase'),
        track=m['track'],
        roller_angle_deg=m.get('roller_angle', 45.0),
        front_axle_x=m.get('front_axle_x'),
        rear_axle_x=m.get('rear_axle_x'))


# --------------------------------------------------------------------------- CLI
def _demo():
    import os

    import yaml
    here = os.path.dirname(os.path.abspath(__file__))
    cfg = os.path.join(here, '..', 'config', 'robot_params.yaml')
    with open(cfg) as f:
        params = yaml.safe_load(f)
    k = from_params(params)

    print('轮径 %.4f m  轮宽 %.4f m'
          % (k.r, params['mecanum'].get('wheel_width', 0)))
    print('前轮 x=%+.4f  后轮 x=%+.4f  -> 轴距 %.4f m' % (k.xf, k.xr, k.wheelbase))
    print('轮距 %.4f m (ly=%.4f)' % (k.track, k.ly))
    print('自转系数  前 %.4f / 后 %.4f   %s'
          % (k.k_front, k.k_rear,
             '(对称)' if abs(k.k_front - k.k_rear) < 1e-9 else
             '(不对称! 差 %.1f%%)' % (100 * abs(k.k_front - k.k_rear) / k.k_front)))
    print()
    print('%-28s %-34s %s' % ('车体速度 (vx, vy, wz)', '四轮角速度 rad/s (FL FR RL RR)', '反解校验'))
    cases = [
        ('前进', (0.5, 0.0, 0.0)),
        ('后退', (-0.5, 0.0, 0.0)),
        ('左横移', (0.0, 0.5, 0.0)),
        ('右横移', (0.0, -0.5, 0.0)),
        ('原地左转', (0.0, 0.0, 1.0)),
        ('左前斜行', (0.4, 0.4, 0.0)),
        ('边走边转', (0.4, 0.0, 0.8)),
    ]
    ok = True
    for name, (vx, vy, wz) in cases:
        w = k.inverse(vx, vy, wz)
        bvx, bvy, bwz = k.forward(*w)
        err = max(abs(bvx - vx), abs(bvy - vy), abs(bwz - wz))
        ok = ok and err < 1e-9
        print('%-12s (%5.2f,%5.2f,%5.2f)  [%7.2f %7.2f %7.2f %7.2f]  err=%.1e'
              % (name, vx, vy, wz, w[0], w[1], w[2], w[3], err))
    print()
    print('正/逆运动学自洽:', 'PASS' if ok else 'FAIL')
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(_demo())
