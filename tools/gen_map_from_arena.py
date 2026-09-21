#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""从**场地几何**生成一张干净的地图, 不依赖 SLAM。

为什么: SLAM(karto/gmapping) 的占用栅格会把 24 mm 的墙"糊"成 2 格(100 mm),
        占用还向内侵入自由区 ~60 mm; 而且地图坐标系和世界坐标系差几厘米。
        导航用的静态地图其实可以直接由几何生成 —— 边界准确、墙薄、坐标系就是世界系。

场地真值 (来自 worlds/competition_arena.world, 由 build_arena.py 生成):
    白线内沿 +-2.076 m, 围墙外移到白线外 0.10m -> **墙内沿 +-2.176 m**,
    墙厚 24 mm, 墙高 0.30 m, 白线场地 4.2 x 4.2 m (地板 4.4 x 4.4)
    (围墙外移的原因见 src/competition_arena/docs/traffic_light_model.md)

用法:
    python3 tools/gen_map_from_arena.py                     # -> maps/arena_clean.pgm/.yaml
    python3 tools/gen_map_from_arena.py --resolution 0.05
    python3 tools/gen_map_from_arena.py --obstacles obstacles.yaml
        # obstacles.yaml: [[x0,y0,x1,y1], ...] 场地内部的障碍物矩形 (比赛出物料后用)

生成后: roslaunch competition_robot navigation.launch map:=<生成的 yaml>
"""
from __future__ import annotations

import argparse
import os
import shutil
import struct
import zlib

import numpy as np

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def write_pgm(path, img):
    """P5 二进制 PGM"""
    h, w = img.shape
    with open(path, 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (w, h))
        f.write(img.astype(np.uint8).tobytes())


def write_png(path, img):
    """顺手存一张 PNG, 方便直接看"""
    rgb = np.stack([img] * 3, axis=-1).astype(np.uint8)
    h, w, _ = rgb.shape
    raw = b''.join(b'\x00' + rgb[y].tobytes() for y in range(h))

    def chunk(tag, data):
        c = tag + data
        return struct.pack('>I', len(data)) + c + struct.pack('>I', zlib.crc32(c) & 0xffffffff)

    png = (b'\x89PNG\r\n\x1a\n'
           + chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 2, 0, 0, 0))
           + chunk(b'IDAT', zlib.compress(raw)) + chunk(b'IEND', b''))
    open(path, 'wb').write(png)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default=os.path.join(WS, 'maps', 'arena_clean'))
    ap.add_argument('--resolution', type=float, default=0.025, help='m/格 (默认 2.5 cm, 边界更准)')
    ap.add_argument('--inner', type=float, default=2.176,
                    help='围墙内沿 (m)。注意不是白线 2.076 —— build_arena.py '
                         '默认 --margin 0.10 把围墙退到了白线外')
    ap.add_argument('--margin', type=float, default=0.30, help='地图比场地外扩多少 (m)')
    ap.add_argument('--obstacles', default=None, help='场地内部障碍 yaml: [[x0,y0,x1,y1],...]')
    ap.add_argument('--no-install', action='store_true', help='不往包里也拷一份')
    a = ap.parse_args()

    res = a.resolution
    half = a.inner + a.margin
    n = int(round(half * 2 / res))
    origin = -half
    # 格子中心的世界坐标
    cx = origin + (np.arange(n) + 0.5) * res
    CX, CY = np.meshgrid(cx, cx)

    occ = (np.abs(CX) >= a.inner) | (np.abs(CY) >= a.inner)   # 墙 + 外面全算障碍
    if a.obstacles:
        import yaml
        for (x0, y0, x1, y1) in yaml.safe_load(open(a.obstacles)):
            occ |= (CX >= x0) & (CX <= x1) & (CY >= y0) & (CY <= y1)

    img = np.where(occ, 0, 254).astype(np.uint8)   # 0=障碍 254=空闲 (map_server 约定)

    out = a.out
    os.makedirs(os.path.dirname(out), exist_ok=True)
    write_pgm(out + '.pgm', img)
    write_png(out + '.png', img)
    with open(out + '.yaml', 'w') as f:
        f.write('image: %s.pgm\n' % os.path.basename(out))
        f.write('resolution: %.4f\n' % res)
        f.write('origin: [%.4f, %.4f, 0.0]\n' % (origin, origin))
        f.write('negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')

    free = (~occ).sum() * res * res
    print('生成 %s.pgm/.yaml/.png' % out)
    print('  %dx%d 格 @ %.3f m/格  = %.2f x %.2f m   原点 (%.2f, %.2f)'
          % (n, n, res, n * res, n * res, origin, origin))
    print('  可走面积 %.2f m²  (几何真值 %.2f m²)' % (free, (2 * a.inner) ** 2))
    print('  墙的内边界: %.4f m  (真值 %.3f)  <- 占用格不会侵入场地内部' % (a.inner, a.inner))
    print('  建议目标点安全区: |x|,|y| <= %.2f m  (再往里车体放不下)'
          % (a.inner - 0.19 - 0.02))

    if not a.no_install:
        pkg = os.path.join(WS, 'src', 'competition_robot', 'maps')
        if os.path.isdir(pkg):
            base = os.path.basename(out)
            shutil.copy(out + '.pgm', os.path.join(pkg, base + '.pgm'))
            shutil.copy(out + '.yaml', os.path.join(pkg, base + '.yaml'))
            print('  已拷到 src/competition_robot/maps/  (roslaunch 可以直接 $(find))')


if __name__ == '__main__':
    main()
