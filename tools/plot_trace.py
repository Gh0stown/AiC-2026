#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把巡航轨迹画到原图上 (和用户标注的位置直接对照)。

用法:
    python3 tools/plot_trace.py --trace trace.csv --waypoints config/waypoints.yaml \
                                --out src/competition_arena/docs/patrol_trace.png

trace csv 格式: t,x,y,yaw (由 tools/patrol.py --save-trace 生成)
"""
from __future__ import annotations

import argparse
import csv
import os
import sys

from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from map_pixels import SRC, x2px, y2py, px2x, px2y   # noqa: E402


def font(sz=16):
    for p in ('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',
              '/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf'):
        if os.path.exists(p):
            return ImageFont.truetype(p, sz)
    return ImageFont.load_default()


def load_trace(path):
    with open(path) as f:
        return [(float(r['x']), float(r['y']), float(r['yaw']))
                for r in csv.DictReader(f)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--trace', required=True)
    ap.add_argument('--waypoints', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--scale', type=float, default=1.0)
    a = ap.parse_args()

    import yaml
    wps, start = [], None
    cfg = yaml.safe_load(open(a.waypoints))
    for i, w in enumerate(cfg.get('waypoints', [])):
        if 'world' in w:
            x, y = w['world']
        else:
            x, y = px2x(w['pixel'][0]), px2y(w['pixel'][1])
        wps.append((w.get('name', 'P%d' % (i + 1)), x, y))
    if cfg.get('start'):
        start = cfg['start']

    tr = load_trace(a.trace)
    img = Image.open(SRC).convert('RGB')
    if a.scale != 1.0:
        img = img.resize((int(img.width * a.scale), int(img.height * a.scale)))
    d = ImageDraw.Draw(img, 'RGBA')
    f = font(20)
    fs = font(16)

    # 轨迹
    pts = [(x2px(x) * a.scale, y2py(y) * a.scale) for x, y, _ in tr]
    if len(pts) > 1:
        d.line(pts, fill=(255, 40, 40, 230), width=4, joint='curve')
    # 起点
    if start:
        sx, sy = x2px(start[0]) * a.scale, y2py(start[1]) * a.scale
        d.ellipse([sx - 12, sy - 12, sx + 12, sy + 12], fill=(0, 90, 255, 255))
        d.text((sx + 16, sy - 10), 'START', fill=(0, 90, 255), font=f)
    # 航点
    for i, (nm, x, y) in enumerate(wps, 1):
        px, py = x2px(x) * a.scale, y2py(y) * a.scale
        r = 16
        click = (nm == 'ring')
        d.ellipse([px - r, py - r, px + r, py + r],
                  outline=(255, 0, 255, 255) if click else (0, 200, 0, 255), width=4)
        if click:
            d.ellipse([px - r - 6, py - r - 6, px + r + 6, py + r + 6],
                      outline=(255, 0, 255, 255), width=3)
        d.text((px + r + 4, py - 12), '%d' % i, fill=(0, 160, 0) if not click else (255, 0, 255), font=f)
    d.text((12, 12), '红线=实际轨迹 蓝点=起点  绿圈=航点(1..%d)  紫圈=打圈的点'
           % len(wps), fill=(255, 255, 0), font=fs)
    img.save(a.out)
    print('轨迹图 -> %s   (%d 个轨迹点, %d 个航点)' % (a.out, len(tr), len(wps)))


if __name__ == '__main__':
    main()
