#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""预览一个 YOLO 数据集: 把标注画回图上 + 打印统计。

用法:
    python3 tools/preview_dataset.py datasets/vision                 # 统计 + 预览图
    python3 tools/preview_dataset.py datasets/vision --n 12          # 预览 12 张
    python3 tools/preview_dataset.py datasets/vision --out p.png

看什么:
  * 框画得对不对 (和画面里的物体对齐) —— 合成数据的标注是投影出来的, 应当严丝合缝
  * 各类别样本数 / 框的像素尺寸分布 (决定模型能不能练起来)
  * 每张图的灯态 (traffic_light 的"状态"不是框, 记在 meta.jsonl 里, 图上标出来)
  * 车牌字符串 (同样在 meta.jsonl 里, 图上标出来)
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from collections import Counter

import numpy as np
from PIL import Image, ImageDraw

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from cnfont import load as _load_font, pick as _pick       # noqa: E402

# 颜色按类别固定, 方便肉眼对比
COLORS = {'standee': (255, 80, 80), 'non_community': (255, 160, 0),
          'traffic_light': (80, 255, 80), 'plate': (80, 160, 255)}


def text_w(d, s, f):
    try:
        return d.textlength(s, font=f)
    except AttributeError:
        try:
            return d.textsize(s, font=f)[0]
        except Exception:
            return 7 * len(s)


def text_box(d, s, f):
    """老 Pillow (7.0.0) 没有 textbbox —— 用 textsize 兜底"""
    try:
        return d.textbbox((0, 0), s, font=f)
    except AttributeError:
        w, h = d.textsize(s, font=f)
        return (0, 0, w, h)


def load_meta(ds):
    p = os.path.join(ds, 'meta.jsonl')
    out = {}
    if os.path.exists(p):
        for line in open(p):
            r = json.loads(line)
            out[os.path.basename(r['file'])] = r
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('ds', help='数据集目录')
    ap.add_argument('--n', type=int, default=8, help='预览几张')
    ap.add_argument('--cols', type=int, default=4)
    ap.add_argument('--out', default=None, help='预览图输出路径 (默认 <ds>/preview.png)')
    ap.add_argument('--tile-w', type=int, default=420)
    a = ap.parse_args()

    names = ['standee', 'non_community', 'traffic_light', 'plate']
    dy = os.path.join(a.ds, 'data.yaml')
    if os.path.exists(dy):
        for line in open(dy):
            if line.startswith('names:'):
                names = [s.strip().strip("'\"[]") for s in line.split(':', 1)[1].split(',')]
                break

    meta = load_meta(a.ds)
    samples = []
    for split in ('train', 'val'):
        idir = os.path.join(a.ds, 'images', split)
        if not os.path.isdir(idir):
            continue
        for fn in sorted(os.listdir(idir)):
            if fn.rsplit('.', 1)[-1].lower() not in ('jpg', 'jpeg', 'png'):
                continue
            lp = os.path.join(a.ds, 'labels', split, os.path.splitext(fn)[0] + '.txt')
            boxes = []
            if os.path.exists(lp):
                for line in open(lp):
                    v = line.split()
                    if len(v) >= 5:
                        boxes.append((int(v[0]), *[float(x) for x in v[1:5]]))
            samples.append(dict(split=split, img=os.path.join(idir, fn), fn=fn, boxes=boxes))

    if not samples:
        print('这个数据集里没有图片:', a.ds)
        return 1

    # ---------------- 统计 ----------------
    cnt = Counter()
    sizes = {i: [] for i in range(len(names))}
    for s in samples:
        for c, cx, cy, w, h in s['boxes']:
            cnt[names[c] if c < len(names) else str(c)] += 1
    split_cnt = Counter(s['split'] for s in samples)
    light = Counter(r['light'] for r in meta.values() if r.get('light'))
    plates = [o.get('plate') for r in meta.values() for o in r.get('objects', [])
              if o.get('plate')]
    # 框尺寸 (用图像尺寸换算)
    for s in samples[:200]:
        im = Image.open(s['img'])
        W, H = im.size
        for c, cx, cy, w, h in s['boxes']:
            if c < len(names):
                sizes[c].append(max(w * W, h * H))
    print('数据集: %s' % a.ds)
    print('  图片 %d 张 (train %d / val %d)'
          % (len(samples), split_cnt.get('train', 0), split_cnt.get('val', 0)))
    print('  标注框:')
    for i, nm in enumerate(names):
        s = sizes.get(i, [])
        print('    %-14s %5d 个   中位边长 %s px'
              % (nm, cnt.get(nm, 0), ('%.0f' % np.median(s)) if s else '-'))
    if light:
        print('  红绿灯状态分布 (来自 meta.jsonl): %s'
              % ', '.join('%s=%d' % (k, v) for k, v in sorted(light.items())))
    if plates:
        print('  车牌真值: %s' % ', '.join(sorted(set(plates))))

    # ---------------- 预览图 ----------------
    n = min(a.n, len(samples))
    cols = max(1, a.cols)
    rows = (n + cols - 1) // cols
    tw = a.tile_w
    # 每张图按同一宽度缩放, 高度取平均比例, 保证网格整齐
    tile_h = int(tw * 960 / 1280)
    cap_h = 46
    sheet = Image.new('RGB', (cols * tw, rows * (tile_h + cap_h)), (24, 24, 28))
    f_small, CJK = _load_font(15)
    f_cap, _ = _load_font(16)

    for k, s in enumerate(samples[:n]):
        im = Image.open(s['img']).convert('RGB')
        sw, sh = im.size
        sc = tw / sw
        im = im.resize((tw, int(sh * sc)), Image.LANCZOS)
        d = ImageDraw.Draw(im)
        for c, cx, cy, w, h in s['boxes']:
            nm = names[c] if c < len(names) else str(c)
            col = COLORS.get(nm, (255, 255, 255))
            x0 = (cx - w / 2) * tw
            x1 = (cx + w / 2) * tw
            y0 = (cy - h / 2) * im.size[1]
            y1 = (cy + h / 2) * im.size[1]
            d.rectangle([x0, y0, x1, y1], outline=col, width=3)
            tb = text_box(d, nm, f_small)
            d.rectangle([x0, y0 - (tb[3] - tb[1]) - 6, x0 + (tb[2] - tb[0]) + 6, y0],
                        fill=col)
            d.text((x0 + 3, y0 - (tb[3] - tb[1]) - 4), nm, font=f_small, fill=(0, 0, 0))

        r = meta.get(s['fn'], {})
        cap = '%s  [%s]' % (s['fn'][:22], s['split'])
        if r.get('light'):
            cap += '  ' + _pick('灯:', 'light:', CJK) + r['light']
        pl = [o['plate'] for o in r.get('objects', []) if o.get('plate')]
        if pl:
            cap += '  ' + _pick('车牌:', 'plate:', CJK) + pl[0]
        cw, ch = tw, cap_h
        sheet.paste(im, ((k % cols) * cw, (k // cols) * (tile_h + cap_h)))
        d2 = ImageDraw.Draw(sheet)
        d2.text(((k % cols) * cw + 4, (k // cols) * (tile_h + cap_h) + tile_h + 14),
                cap, font=f_cap, fill=(230, 230, 230))

    out = a.out or os.path.join(a.ds, 'preview.png')
    sheet.save(out)
    print('  预览图 -> %s  (%dx%d, %d 张)' % (out, sheet.size[0], sheet.size[1], n))
    return 0


if __name__ == '__main__':
    sys.exit(main())
