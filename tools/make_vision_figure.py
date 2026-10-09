#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把一次运行里的真帧拼成"三类识别"三联图（人偶立牌 / 车牌 / 红绿灯）。

为什么要有它
------------
报告原来用的 `docs/vision_output_example.jpg` 是**画出来的示意图**（960x720、
只有深色底、只有框和字），既不真、又宣称"人偶+绿灯+车牌同时出现"
（`A_north` 站点根本看不到车牌）。本脚本直接从 `vision_runs/<时间戳>/` 的
真帧 + `results.json` 里的检测框裁切拼图，每个面板下面印上**终端输出的那一行**，
做到"终端与图像一一对应"可核对。

用法:
    python3 tools/make_vision_figure.py                       # 取最新一次运行
    python3 tools/make_vision_figure.py --run vision_runs/2026-10-09_153354
    python3 tools/make_vision_figure.py -o docs/vision_output_3tasks.jpg --panel 720
"""
from __future__ import annotations

import argparse
import glob
import json
import os
import sys

from PIL import Image, ImageDraw, ImageFont

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FONTS = ('/mnt/c/Windows/Fonts/msyh.ttc', '/mnt/c/Windows/Fonts/msyhbd.ttc',
         '/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf')

# 三类任务各挑一个"最有代表性"的点位（框多、内容清楚）
PICK = [('standee', 'A_north', '人偶立牌（街区统计）'),
        ('plate', 'car_1', '车牌字符识别'),
        ('light', 'tl_top', '红绿灯状态')]


def font(size):
    for p in FONTS:
        if os.path.isfile(p):
            try:
                return ImageFont.truetype(p, size)
            except Exception:                                   # noqa: BLE001
                continue
    return ImageFont.load_default()


def newest_run(runs_dir):
    ds = sorted(glob.glob(os.path.join(runs_dir, '*')), key=os.path.getmtime)
    return ds[-1] if ds else None


def pick_entry(points, kind, point):
    """同一 (kind, point) 可能有多条(红绿灯会被通行闸反复读), 取框最多的那条。"""
    cand = [p for p in points if p.get('kind') == kind and p.get('point') == point]
    if not cand:
        cand = [p for p in points if p.get('kind') == kind]
    if not cand:
        return None
    return max(cand, key=lambda p: len(p.get('boxes') or []))


def crop_for(img, boxes, pad=0.28):
    W, H = img.size
    if not boxes:
        return img
    xs0 = min(b['box'][0] for b in boxes); ys0 = min(b['box'][1] for b in boxes)
    xs1 = max(b['box'][2] for b in boxes); ys1 = max(b['box'][3] for b in boxes)
    mw = (xs1 - xs0) * pad + 30; mh = (ys1 - ys0) * pad + 30
    x0 = max(0, int(xs0 - mw)); y0 = max(0, int(ys0 - mh))
    x1 = min(W, int(xs1 + mw)); y1 = min(H, int(ys1 + mh))
    # 目标太小时给个最小取景窗，避免裁得只剩一小块
    if x1 - x0 < W * 0.45:
        cx = (x0 + x1) / 2
        x0 = max(0, int(cx - W * 0.225)); x1 = min(W, int(cx + W * 0.225))
    if y1 - y0 < H * 0.45:
        cy = (y0 + y1) / 2
        y0 = max(0, int(cy - H * 0.225)); y1 = min(H, int(cy + H * 0.225))
    return img.crop((x0, y0, x1, y1))


def fit(img, w, h, bg=(12, 12, 12)):
    r = min(w / img.width, h / img.height)
    im = img.resize((max(1, int(img.width * r)), max(1, int(img.height * r))),
                    Image.LANCZOS)
    out = Image.new('RGB', (w, h), bg)
    out.paste(im, ((w - im.width) // 2, (h - im.height) // 2))
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description='拼三类识别的真帧三联图')
    ap.add_argument('--run', default='')
    ap.add_argument('--runs', default=os.path.join(WS, 'vision_runs'))
    ap.add_argument('-o', '--out',
                    default=os.path.join(WS, 'docs', 'vision_output_3tasks.jpg'))
    ap.add_argument('--panel', type=int, default=700)
    a = ap.parse_args(argv)

    run = a.run or newest_run(a.runs)
    if not run:
        print('找不到 vision_runs 存档'); return 1
    rj = os.path.join(run, 'results.json')
    data = json.load(open(rj, encoding='utf-8'))
    points = data.get('points', [])
    print('  存档: %s（%d 条识别记录）' % (os.path.relpath(run, WS), len(points)))

    pw = a.panel; ph = int(pw * 0.75); band = int(pw * 0.20); gap = 14
    panels = []
    for kind, point, title in PICK:
        e = pick_entry(points, kind, point)
        if not e:
            print('  !! 存档里没有 %s 的记录' % kind); return 1
        # results.json 里的 image 是相对 vision_runs/ 的路径
        ipath = os.path.join(a.runs, e['image'])
        if not os.path.isfile(ipath):
            ipath = os.path.join(run, e['image'])
        img = Image.open(ipath).convert('RGB')
        crop = fit(crop_for(img, e.get('boxes')), pw, ph)
        line = (e.get('lines') or [''])[0]
        panels.append((title, e.get('point'), crop, line))
        print('  %-8s %-8s 框 %d 个  终端行: %s' % (kind, e.get('point'),
                                                   len(e.get('boxes') or []), line))

    W = pw * len(panels) + gap * (len(panels) + 1)
    cap = int(pw * 0.145)                      # 说明区: 留够两行
    H = gap + band + ph + cap + gap
    canvas = Image.new('RGB', (W, H), (18, 18, 18))
    d = ImageDraw.Draw(canvas)
    f_title = font(int(pw * 0.052)); f_line = font(int(pw * 0.040))

    for i, (title, point, crop, line) in enumerate(panels):
        x = gap + i * (pw + gap)
        d.rectangle([x, gap, x + pw, gap + band - 6], fill=(28, 28, 28))
        d.text((x + 12, gap + 8), '%s  %s' % ('abc'[i] + ')', title),
               font=f_title, fill=(255, 255, 255))
        d.text((x + 12, gap + 8 + int(pw * 0.062)), point, font=f_line,
               fill=(255, 190, 60))
        canvas.paste(crop, (x, gap + band))
        d.rectangle([x, gap + band, x + pw, gap + band + ph], outline=(70, 70, 70))
        # 终端输出的那一行（与上面的框一一对应）
        for k, seg in enumerate(_wrap(line, 34)):
            d.text((x + 12, gap + band + ph + 8 + k * int(pw * 0.046)), seg,
                   font=f_line, fill=(210, 235, 255))
    canvas.save(a.out, quality=92, subsampling=0)
    print('  已写出 %s  %dx%d' % (os.path.relpath(a.out, WS), W, H))
    return 0


def _wrap(s, n):
    out, cur = [], ''
    for ch in s:
        cur += ch
        if len(cur) >= n and ch in ' /）)':
            out.append(cur); cur = ''
    if cur:
        out.append(cur)
    return out[:2] or ['']


if __name__ == '__main__':
    sys.exit(main())
