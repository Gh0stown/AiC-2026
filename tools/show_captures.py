#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把一次拍摄的图片拼成一张总览图, 快速人眼过一遍。

用法:
    python3 tools/show_captures.py captures/2026-09-21_211530
    python3 tools/show_captures.py captures/2026-09-21_211530 --cols 3 --width 420
    python3 tools/show_captures.py captures/                 # 用最新一次
输出: 在同一个文件夹里写 overview.png
"""
from __future__ import annotations

import argparse
import csv
import glob
import os
import sys

from PIL import Image, ImageDraw, ImageFont

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def font(size):
    for p in ('/usr/share/fonts/opentype/noto/NotoSansCJK-Medium.ttc',
              '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'):
        if os.path.exists(p):
            return ImageFont.truetype(p, size)
    return ImageFont.load_default()


def latest_run(root):
    d = sorted(glob.glob(os.path.join(root, '*')), key=os.path.getmtime)
    d = [x for x in d if os.path.isdir(x)]
    return d[-1] if d else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run', nargs='?', default=os.path.join(WS, 'captures'))
    ap.add_argument('--cols', type=int, default=4)
    ap.add_argument('--width', type=int, default=320, help='每张缩略图宽')
    ap.add_argument('--frame', type=int, default=0, help='每个目标取第几帧')
    ap.add_argument('--out', default=None)
    a = ap.parse_args()

    run = a.run
    if os.path.isdir(run) and not os.path.exists(os.path.join(run, 'run.yaml')):
        run = latest_run(run) or run
    if not os.path.isdir(run):
        sys.exit('找不到文件夹: %s' % a.run)
    print('查看: %s' % os.path.relpath(run, WS))

    # 优先按 index.csv 的顺序; 没有就按文件名
    items = []
    idx = os.path.join(run, 'index.csv')
    if os.path.exists(idx):
        for r in csv.DictReader(open(idx)):
            files = [f for f in r['files'].split(';') if f]
            if files:
                items.append((r['target'], r.get('task', ''), r.get('ground_truth', ''),
                              files[min(a.frame, len(files) - 1)]))
    if not items:
        for f in sorted(glob.glob(os.path.join(run, '*.png')) + glob.glob(os.path.join(run, '*.jpg'))):
            if os.path.basename(f) == 'overview.png':
                continue
            items.append((os.path.basename(f), '', '', os.path.basename(f)))
    if not items:
        sys.exit('这个文件夹里没有图片')

    tw = a.width
    th = int(tw * 960.0 / 1280.0)
    cols = a.cols
    rowsn = (len(items) + cols - 1) // cols
    pad, cap_h = 8, 34
    W = cols * (tw + pad) + pad
    H = rowsn * (th + cap_h + pad) + pad
    sheet = Image.new('RGB', (W, H), (24, 24, 28))
    d = ImageDraw.Draw(sheet)
    f = font(17)
    for i, (tgt, task, truth, fn) in enumerate(items):
        p = os.path.join(run, fn)
        if not os.path.exists(p):
            continue
        im = Image.open(p).convert('RGB').resize((tw, th), Image.LANCZOS)
        c, r = i % cols, i // cols
        x = pad + c * (tw + pad)
        y = pad + r * (th + cap_h + pad)
        sheet.paste(im, (x, y))
        cap = '%s' % tgt
        if truth:
            cap += '  真值:%s' % truth
        d.text((x + 2, y + th + 6), cap, fill=(230, 230, 120), font=f)
    out = a.out or os.path.join(run, 'overview.png')
    sheet.save(out)
    print('总览图 -> %s   (%d 张, %dx%d)' % (os.path.relpath(out, WS), len(items), W, H))


if __name__ == '__main__':
    main()
