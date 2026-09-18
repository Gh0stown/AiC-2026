#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""图上标点 <-> 场地坐标 的换算工具。

场地和原图的关系 (和 scripts/build_arena.py 里 Frame 类完全一致, 单一真值源):
    原图 tools/map_source.jpg 1280x1280, 场地(4.2x4.2)占像素 x[29,1252) y[31,1254)
    比例尺 3.4342 mm/px
    世界 +x = 图的**右**, 世界 +y = 图的**上**
    x = (px - 29) * scale - 2.1
    y = 2.1 - (py - 31) * scale
  校验: 出生点 (1.772, 1.782) -> 像素 (1156, 124), 正好在图右上角那块小方块里 ✓

用法:
    python3 tools/map_pixels.py --grid                    # 生成两张带网格的图, 用来标点
    python3 tools/map_pixels.py --check                   # 生成标了地标的图 (自检用)
    python3 tools/map_pixels.py --px 640 640              # 像素 -> 世界坐标
    python3 tools/map_pixels.py --world 0 0               # 世界坐标 -> 像素
    python3 tools/map_pixels.py --file config/waypoints.yaml   # 检查航点文件

标点建议: 打开 src/competition_arena/docs/coord_world_grid.png,
         直接读世界坐标 (每格 0.5 m, 图上标了数值); 绿色框内是车能停到的范围。
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np
from PIL import Image, ImageDraw, ImageFont

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(WS, 'src', 'competition_arena', 'tools', 'map_source.jpg')
DOCS = os.path.join(WS, 'src', 'competition_arena', 'docs')

# ★ 和 build_arena.py 的 ARENA_PX / --arena 保持一致
ARENA_PX = (29, 31, 1252, 1254)
ARENA_M = 4.2
X0, Y0, X1, Y1 = ARENA_PX
SCALE = ARENA_M / (X1 - X0)          # m / px
HALF = ARENA_M / 2.0
INNER = 2.076                        # 场地内沿
GOAL_LIMIT = 1.75                    # 车能停到的范围 (|x|,|y| <= 这个值)


def px2x(px):
    return (px - X0) * SCALE - HALF


def px2y(py):
    return HALF - (py - Y0) * SCALE


def x2px(x):
    return X0 + (x + HALF) / SCALE


def y2py(y):
    return Y0 + (HALF - y) / SCALE


def _font(size=16):
    for p in ('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',
              '/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf'):
        if os.path.exists(p):
            return ImageFont.truetype(p, size)
    return ImageFont.load_default()


def make_grid(step_m=0.5, px_step=100, out_px=None, out_world=None):
    img = Image.open(SRC).convert('RGB')
    d = ImageDraw.Draw(img, 'RGBA')
    f = _font(14)
    # 场地外框 + 内沿 + 可停范围
    for lim, color, w in ((HALF, (255, 0, 0, 220), 3), (INNER, (255, 120, 0, 200), 2)):
        d.rectangle([x2px(-lim), y2py(lim), x2px(lim), y2py(-lim)], outline=color, width=w)
    d.rectangle([x2px(-GOAL_LIMIT), y2py(GOAL_LIMIT),
                 x2px(GOAL_LIMIT), y2py(-GOAL_LIMIT)], outline=(0, 200, 0, 220), width=2)
    # 世界坐标网格
    xs = np.arange(-2.0, 2.01, step_m)
    for v in xs:
        d.line([x2px(v), y2py(2.1), x2px(v), y2py(-2.1)], fill=(0, 120, 255, 90), width=1)
        d.line([x2px(-2.1), y2py(v), x2px(2.1), y2py(v)], fill=(0, 120, 255, 90), width=1)
    if out_world:
        for v in xs:
            d.text((x2px(v) + 2, y2py(2.1) + 2), '%+.1f' % v, fill=(0, 80, 255), font=f)
            d.text((x2px(-2.1) + 2, y2py(v) + 2), '%+.1f' % v, fill=(0, 80, 255), font=f)
        d.text((x2px(-GOAL_LIMIT) + 4, y2py(-GOAL_LIMIT) - 22),
               'green = 车能停到 (|x|,|y|<=%.2f)' % GOAL_LIMIT, fill=(0, 160, 0), font=f)
        img.save(out_world)
        print('  世界坐标网格 ->', out_world)
    if out_px:
        img2 = Image.open(SRC).convert('RGB')
        d2 = ImageDraw.Draw(img2, 'RGBA')
        f2 = _font(14)
        for p in range(0, 1281, px_step):
            d2.line([(p, 0), (p, 1280)], fill=(255, 0, 255, 80), width=1)
            d2.line([(0, p), (1280, p)], fill=(255, 0, 255, 80), width=1)
            d2.text((p + 2, 2), str(p), fill=(200, 0, 200), font=f2)
            d2.text((2, p + 2), str(p), fill=(200, 0, 200), font=f2)
        d2.rectangle([x2px(-INNER), y2py(INNER), x2px(INNER), y2py(-INNER)],
                     outline=(255, 120, 0, 220), width=2)
        img2.save(out_px)
        print('  像素坐标网格 ->', out_px)


def make_check(out):
    pts = [('spawn(1.772,1.782)', 1.772, 1.782, (255, 0, 0)),
           ('center(0,0)', 0.0, 0.0, (0, 160, 0)),
           ('inner(-2.076,-2.076)', -INNER, -INNER, (0, 0, 255)),
           ('inner(+2.076,+2.076)', INNER, INNER, (0, 0, 255)),
           ('goal(1.5,1.5)', 1.5, 1.5, (255, 140, 0)),
           ('goal(-1.5,-1.5)', -1.5, -1.5, (255, 140, 0))]
    img = Image.open(SRC).convert('RGB')
    d = ImageDraw.Draw(img)
    f = _font(18)
    for name, x, y, c in pts:
        px, py = x2px(x), y2py(y)
        d.ellipse([px - 7, py - 7, px + 7, py + 7], outline=c, width=3)
        d.text((px + 10, py - 8), '%s' % name, fill=c, font=f)
    img.save(out)
    print('  地标图 ->', out)
    for name, x, y, _ in pts:
        print('    %-22s (%.3f, %.3f) -> px (%.0f, %.0f)' % (name, x, y, x2px(x), y2py(y)))


def check_file(path):
    import yaml
    cfg = yaml.safe_load(open(path))
    wps = cfg.get('waypoints') if isinstance(cfg, dict) else cfg
    print('  航点文件 %s: %d 个点' % (path, len(wps)))
    for i, w in enumerate(wps):
        name = w.get('name', '#%d' % (i + 1))
        if 'world' in w:                 # 和 patrol.py 一致: world 优先
            x, y = w['world']
            px, py = x2px(x), y2py(y)
            src = 'world'
        else:
            px, py = w['pixel']
            x, y = px2x(px), px2y(py)
            src = 'pixel'
        ok = max(abs(x), abs(y)) <= GOAL_LIMIT
        print('    %-12s %-5s px(%4.0f,%4.0f) -> world(%+.3f, %+.3f)  %s'
              % (name, src, px, py, x, y, '✓ 可停' if ok else '⚠ 太靠墙(车可能停不到)'))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--px', type=float, nargs=2, help='像素 -> 世界: --px PX PY')
    ap.add_argument('--world', type=float, nargs=2, help='世界 -> 像素: --world X Y')
    ap.add_argument('--grid', action='store_true', help='生成带网格的图 (标点用)')
    ap.add_argument('--check', action='store_true', help='生成地标图 (自检)')
    ap.add_argument('--file', help='检查航点 yaml')
    a = ap.parse_args()
    did = False
    if a.px:
        print('  像素 (%.0f, %.0f) -> 世界 (%.3f, %.3f)' % (a.px[0], a.px[1],
                                                          px2x(a.px[0]), px2y(a.px[1])))
        did = True
    if a.world:
        print('  世界 (%.3f, %.3f) -> 像素 (%.0f, %.0f)  %s' % (
            a.world[0], a.world[1], x2px(a.world[0]), y2py(a.world[1]),
            '✓ 可停' if max(abs(v) for v in a.world) <= GOAL_LIMIT else '⚠ 太靠墙'))
        did = True
    if a.file:
        check_file(a.file)
        did = True
    if a.check:
        os.makedirs(DOCS, exist_ok=True)
        make_check(os.path.join(DOCS, 'coord_check.png'))
        did = True
    if a.grid:
        os.makedirs(DOCS, exist_ok=True)
        make_grid(out_px=os.path.join(DOCS, 'coord_px_grid.png'),
                  out_world=os.path.join(DOCS, 'coord_world_grid.png'))
        did = True
    if not did:
        print(__doc__)


if __name__ == '__main__':
    main()
