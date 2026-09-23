#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把采集世界的布局画成俯视图 (给人和文档看)。

用法:
    python3 tools/draw_collect_layout.py            # -> docs/collect_layout.png
"""
from __future__ import annotations

import argparse
import math
import os
import sys

import numpy as np
import yaml
from PIL import Image, ImageDraw

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from cnfont import load as _load_font, pick as _pick       # noqa: E402

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LAYOUT = os.path.join(WS, 'src', 'competition_arena', 'config', 'collect_layout.yaml')

C_STANDEE = (90, 200, 255)
C_NON = (255, 170, 60)
C_LIGHT = (120, 255, 120)
C_PLATE = (255, 120, 120)
C_RIG = (255, 240, 90)
C_FLOOR = (36, 36, 40)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--layout', default=LAYOUT)
    ap.add_argument('--out', default=os.path.join(WS, 'docs', 'collect_layout.png'))
    ap.add_argument('--px-per-m', type=float, default=62.0)
    a = ap.parse_args()

    L = yaml.safe_load(open(a.layout))
    objs = L['objects']
    rigs = L.get('rigs', [])
    S = a.px_per_m
    EXT = 7.0                                   # 画 ±7 m
    W = H = int(2 * EXT * S)
    f, CJK = _load_font(15)
    fb, _ = _load_font(19)

    def P(x, y):                                # 世界 -> 像素 (y 轴翻转)
        return (W / 2 + x * S, H / 2 - y * S)

    im = Image.new('RGB', (W, H), (20, 20, 24))
    d = ImageDraw.Draw(im)

    # 地面 (3x3 块)
    for i in range(3):
        for j in range(3):
            cx, cy = (i - 1) * 4.4, (j - 1) * 4.4
            x0, y0 = P(cx - 2.2, cy + 2.2)
            x1, y1 = P(cx + 2.2, cy - 2.2)
            d.rectangle([x0, y0, x1, y1], fill=C_FLOOR, outline=(70, 70, 76))

    # 立牌圈
    ring = L['ring']
    rc = P(ring['center'][0], ring['center'][1])
    rr = ring['radius'] * S
    d.ellipse([rc[0] - rr, rc[1] - rr, rc[0] + rr, rc[1] + rr], outline=(80, 120, 160))

    for o in objs:
        cls = o['cls']
        if cls == 'floor':
            continue
        x, y, yaw = float(o['x']), float(o['y']), float(o['yaw'])
        px, py = P(x, y)
        col = {'standee': C_STANDEE, 'non_community': C_NON,
               'traffic_light': C_LIGHT, 'plate': C_PLATE}.get(cls, (200, 200, 200))
        # 物体本体 (朝向: 立牌正面=局部 -x; 灯/车牌照=局部 +x)
        if cls in ('standee', 'non_community'):
            fx, fy = -math.cos(yaw), -math.sin(yaw)
            w, h = 3.0, 9.0
        else:
            fx, fy = math.cos(yaw), math.sin(yaw)
            w, h = (26.0, 7.0) if cls == 'traffic_light' else (16.0, 7.0)
        ax, ay = math.cos(yaw), math.sin(yaw)
        bx, by = -math.sin(yaw), math.cos(yaw)
        pts = []
        for sx, sy in ((+1, +1), (+1, -1), (-1, -1), (-1, +1)):
            pts.append(P(x + ax * sx * h / S / 2 + bx * sy * w / S / 2,
                         y + ay * sx * h / S / 2 + by * sy * w / S / 2))
        d.polygon(pts, fill=col, outline=(0, 0, 0))
        # 朝向箭头 (指向"正面")
        L0 = 0.42 if cls in ('standee', 'non_community') else 0.55
        d.line([px, py, px + fx * L0 * S, py - fy * L0 * S], fill=col, width=2)
        d.ellipse([px + fx * L0 * S - 2, py - fy * L0 * S - 2,
                   px + fx * L0 * S + 2, py - fy * L0 * S + 2], fill=col)

    # 三台相机小车 + 它们的拍摄范围
    stations = [('rig1', 0.0, 0.0, 'ring'), ('rig2', -1.0, 1.45, 'light'),
                ('rig3', 4.2, 0.10, 'plate')]
    for nm, x, y, ph in stations:
        px, py = P(x, y)
        d.ellipse([px - 7, py - 7, px + 7, py + 7], outline=C_RIG, width=3)
        d.text((px + 10, py - 8), nm, font=f, fill=C_RIG)
        if ph == 'ring':
            d.ellipse([px - 0.25 * S, py - 0.25 * S, px + 0.25 * S, py + 0.25 * S],
                      outline=(120, 110, 40))
        if ph == 'light':
            for dd in (0.8, 1.2, 1.6):
                d.ellipse([px - dd * S, py - dd * S, px + dd * S, py + dd * S],
                          outline=(60, 110, 60))
        if ph == 'plate':
            for dd in (0.6, 0.9, 1.2):
                d.ellipse([px - dd * S, py - dd * S, px + dd * S, py + dd * S],
                          outline=(110, 60, 60))

    # 图例 (多行, 按实测文字宽度排版, 避免挤在一起)
    def T(zh, en):
        return _pick(zh, en, CJK)

    def tw(txt):                                # 老 Pillow 没有 textlength
        try:
            return d.textlength(txt, font=f)
        except AttributeError:
            return d.textsize(txt, font=f)[0]

    legend = [(T('社区立牌 x8 (正面朝圈心)', 'community x8 (face center)'), C_STANDEE),
              (T('非社区立牌 x2', 'non-community x2'), C_NON),
              (T('红绿灯 x2 (正面朝 +x)', 'traffic light x2'), C_LIGHT),
              (T('车牌 x3 (朝 +x)', 'plate x3'), C_PLATE),
              (T('相机小车 x3', 'rig x3'), C_RIG),
              (T('圈 = 拍摄距离', 'circles = shot distance'), (80, 100, 80))]
    rows, cur, curw = [], [], 0
    for txt, col in legend:
        w = 22 + int(tw(txt))
        if curw + w > W - 16 and cur:
            rows.append(cur)
            cur, curw = [], 0
        cur.append((txt, col, w))
        curw += w
    rows.append(cur)
    LEG_H = 24 * len(rows) + 8
    im2 = Image.new('RGB', (W, H + LEG_H), (20, 20, 24))
    im2.paste(im.crop((0, 0, W, H)), (0, 0))
    d2 = ImageDraw.Draw(im2)
    for r_i, row in enumerate(rows):
        x = 10
        yy = H + 6 + r_i * 24
        for txt, col, w in row:
            d2.rectangle([x, yy + 4, x + 12, yy + 16], fill=col)
            d2.text((x + 17, yy + 3), txt, font=f, fill=(220, 220, 220))
            x += w
    im = im2

    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    im.save(a.out)
    print('俯视图 -> %s  (%dx%d)' % (a.out, im.size[0], im.size[1]))
    print('  立牌圈: %d 个, 半径 %.2f m' % (len(ring['models']), ring['radius']))
    print('  红绿灯: %d, 车牌: %d, 相机小车: %d'
          % (len([o for o in objs if o['cls'] == 'traffic_light']),
             len([o for o in objs if o['cls'] == 'plate']), len(rigs)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
