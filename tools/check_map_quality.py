#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""地图质量打分 —— 用来横向比较不同建图算法 (gmapping / karto / cartographer ...)。

用法:
    python3 tools/check_map_quality.py maps/xxx.pgm [maps/yyy.pgm ...]

看什么:
  1. 房间尺寸      (应该 ≈ 4.2 x 4.2 m, 内沿)
  2. 墙直不直      (墙那一列/行的抖动, 越小越好)
  3. 空闲面积      (房间内部 ≈ 17 m²)
  4. ★ 墙外的占用  (地图外面糊出来的"扇形"假墙 —— 越少越好, 目标是 0)
  5. 空闲区域个数  (一块 = 正常; 出现第二块大的说明地图里长出了别的东西)
"""
from __future__ import annotations

import os
import sys
from collections import deque

import numpy as np


def read_pgm(path):
    with open(path, 'rb') as f:
        magic = f.readline().strip()
        if magic != b'P5':
            raise ValueError('%s: 只支持 P5 二进制 PGM (%s)' % (path, magic))
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()
        w, h = (int(v) for v in line.split())
        maxv = int(f.readline())
        data = np.frombuffer(f.read(w * h), dtype=np.uint8).reshape(h, w)
    return data, w, h, maxv


def read_yaml(path):
    cfg = {}
    if os.path.exists(path):
        for line in open(path):
            if ':' in line:
                k, v = line.split(':', 1)
                cfg[k.strip()] = v.strip()
    return cfg


def flood(mask, start, visited):
    """从 start 出发 4 邻域连通块 (迭代, 不爆栈)"""
    h, w = mask.shape
    q = deque([start])
    visited[start] = True
    cells = []
    while q:
        y, x = q.popleft()
        cells.append((y, x))
        for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            ny, nx = y + dy, x + dx
            if 0 <= ny < h and 0 <= nx < w and mask[ny, nx] and not visited[ny, nx]:
                visited[ny, nx] = True
                q.append((ny, nx))
    return cells


def components(mask):
    visited = np.zeros_like(mask, dtype=bool)
    ys, xs = np.nonzero(mask)
    out = []
    for y, x in zip(ys, xs):
        if not visited[y, x]:
            out.append(flood(mask, (y, x), visited))
    return out


def report(path):
    img, w, h, _ = read_pgm(path)
    cfg = read_yaml(os.path.splitext(path)[0] + '.yaml')
    res = float(cfg.get('resolution', 0.05))
    o = cfg.get('origin', '[-10, -10, 0]').strip('[]').split(',')
    ox, oy = float(o[0]), float(o[1])

    occ = (img == 0)
    free = (img == 254)
    unk = (img != 0) & (img != 254)

    print('=' * 72)
    print('%s   %dx%d @ %.3f m/格  = %.1f x %.1f m' % (path, w, h, res, w * res, h * res))
    print('  占用 %.2f m²   空闲 %.2f m²   未知 %.2f m²'
          % (occ.sum() * res * res, free.sum() * res * res, unk.sum() * res * res))
    if free.sum() == 0:
        print('  ✗ 没有空闲区域, 地图基本是空的')
        return
    comps = sorted(components(free), key=len, reverse=True)
    big = [c for c in comps if len(c) * res * res > 1.0]
    ys = [y for y, _ in comps[0]]
    xs = [x for _, x in comps[0]]
    bx0, bx1 = min(xs) * res + ox, (max(xs) + 1) * res + ox
    by0, by1 = min(ys) * res + oy, (max(ys) + 1) * res + oy
    span = max(bx1 - bx0, by1 - by0)
    farea = free.sum() * res * res
    print('  主空闲区: %.2f m², 外框 %.2f x %.2f m   (房间内沿应该是 4.15 x 4.15)'
          % (len(comps[0]) * res * res, bx1 - bx0, by1 - by0))

    # 墙外散点: 离主空闲区外框 0.5 m 以外的占用格
    pad = int(round(0.5 / res))
    y0, y1 = max(0, min(ys) - pad), min(h, max(ys) + pad)
    x0, x1 = max(0, min(xs) - pad), min(w, max(xs) + pad)
    outside = occ.copy()
    outside[y0:y1, x0:x1] = False
    print('  墙外占用: %d 格 (%.2f m²)' % (outside.sum(), outside.sum() * res * res))

    # 墙直不直: 取主空闲区四条边附近 0.15 m 内的占用格, 看位置抖动
    oy_, ox_ = np.nonzero(occ)

    def straight(axis):
        d = []
        for e in ((min(ys), max(ys)) if axis == 0 else (min(xs), max(xs))):
            cols = {}
            for y, x in zip(oy_, ox_):
                v = y if axis == 0 else x
                if abs(v - e) <= int(0.15 / res):
                    key = x if axis == 0 else y
                    cols.setdefault(key, []).append(v)
            pos = [np.mean(v) * res for v in cols.values() if len(v) >= 2]
            if pos:
                d.append(np.std(pos))
        return np.mean(d) if d else float('nan')

    js, jv = straight(0), straight(1)
    print('  墙的位置抖动(越小越直): 横墙 %.3f m, 竖墙 %.3f m' % (js, jv))

    # ---- 结论 ----
    flags = []
    if not (3.8 <= span <= 4.6):
        flags.append('空闲区外框 %.2f m ≠ 4.15 m (地图被拉大/多出一块)' % span)
    if not (13.0 <= farea <= 20.0):
        flags.append('空闲面积 %.1f m² ≠ 17 m² 左右' % farea)
    if len(big) > 1:
        flags.append('有 %d 块 >1 m² 的空闲区' % len(big))
    if outside.sum() * res * res >= 0.15:
        flags.append('墙外有 %.2f m² 假墙' % (outside.sum() * res * res))
    if max(js, jv) > 0.08:
        flags.append('墙不直 (抖动 %.3f m)' % max(js, jv))
    print('  ' + ('✓ 地图正常' if not flags else '⚠ 有问题: ' + '; '.join(flags)))


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    for p in sys.argv[1:]:
        report(p)
