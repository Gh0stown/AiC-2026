#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""从"在图上画了记号"的图片里自动找出记号, 换算成场地坐标。

用法:
    python3 tools/detect_marks.py 图片 [--out config/waypoints.yaml] [--order auto]

识别规则: 找**纯红色**的团块 (用户的记号), 自动排除我网格图里那圈红色外框(太大)。
圆环和叉叉都认, 并标出哪个是"圈起来的"(中心是空的)。
"""
from __future__ import annotations

import argparse
import os
import sys
from collections import deque

import numpy as np
from PIL import Image

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from map_pixels import px2x, px2y, x2px, y2py, GOAL_LIMIT   # noqa: E402


def red_mask(img):
    a = np.asarray(img.convert('RGB')).astype(np.int16)
    r, g, b = a[:, :, 0], a[:, :, 1], a[:, :, 2]
    return (r > 150) & (g < 90) & (b < 90)


def blobs(mask, min_px=30, max_bbox=120):
    h, w = mask.shape
    seen = np.zeros_like(mask, dtype=bool)
    out = []
    ys, xs = np.nonzero(mask)
    for y0, x0 in zip(ys, xs):
        if seen[y0, x0]:
            continue
        q = deque([(y0, x0)])
        seen[y0, x0] = True
        pts = []
        while q:
            y, x = q.popleft()
            pts.append((y, x))
            for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                ny, nx = y + dy, x + dx
                if 0 <= ny < h and 0 <= nx < w and mask[ny, nx] and not seen[ny, nx]:
                    seen[ny, nx] = True
                    q.append((ny, nx))
        if len(pts) < min_px:
            continue
        ys_ = [p[0] for p in pts]
        xs_ = [p[1] for p in pts]
        bh, bw = max(ys_) - min(ys_) + 1, max(xs_) - min(xs_) + 1
        if bw > max_bbox or bh > max_bbox:      # 我的红色外框 -> 跳过
            continue
        cy, cx = int(np.mean(ys_)), int(np.mean(xs_))
        hollow = not mask[cy, cx]              # 中心是空的 -> 圆环
        out.append(dict(n=len(pts), cy=cy, cx=cx, bw=bw, bh=bh, hollow=hollow))
    out.sort(key=lambda b: (b['cy'], b['cx']))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('image')
    ap.add_argument('--out', help='顺便写出航点 yaml')
    ap.add_argument('--order', default='auto',
                    help='auto=按"左上->右上->左下->右下"排; given=按图上顺序')
    a = ap.parse_args()

    img = Image.open(a.image)
    bs = blobs(red_mask(img))
    print('找到 %d 个记号:' % len(bs))
    print('  #   像素(px,py)      场地(x,y)      形状   可停?')
    marks = []
    for i, b in enumerate(bs, 1):
        x, y = px2x(b['cx']), px2y(b['cy'])
        ok = max(abs(x), abs(y)) <= GOAL_LIMIT
        shape = '圆环' if b['hollow'] else '叉'
        print('  %-3d (%4d,%4d)   (%+.3f,%+.3f)  %-5s  %s'
              % (i, b['cx'], b['cy'], x, y, shape, '✓' if ok else '✗ 太靠墙'))
        marks.append(dict(idx=i, px=(b['cx'], b['cy']), world=(x, y),
                          hollow=b['hollow'], ok=ok))

    if a.out:
        # 太靠墙的点自动挪到可停边界上 (并打印出来)
        lines = ['# 由 tools/detect_marks.py 从标注图自动生成',
                 '# pixel = 在 coord_world_grid.png 上的像素; world = 场地坐标(m)',
                 '# 起点 = 出生点 (1.772, 1.782), 由脚本自动作为第 0 个点',
                 'start: [1.772, 1.782]',
                 'waypoints:']
        for m in marks:
            x, y = m['world']
            note = ''
            if not m['ok']:
                x = max(-GOAL_LIMIT, min(GOAL_LIMIT, x))
                y = max(-GOAL_LIMIT, min(GOAL_LIMIT, y))
                note = '    # ★ 原标点太靠墙, 已挪到 %.2f 以内' % GOAL_LIMIT
            nm = 'ring' if m['hollow'] else 'P%d' % m['idx']
            lines.append('  - {name: %-6s pixel: [%4d, %4d], world: [%+.3f, %+.3f]}%s'
                         % (nm, m['px'][0], m['px'][1], x, y, note))
        os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
        open(a.out, 'w').write('\n'.join(lines) + '\n')
        print('  已写出航点文件 ->', a.out)


if __name__ == '__main__':
    main()
