#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""YOLO 框车牌 -> 裁下来 -> HyperLPR3 识别字符，端到端跑一遍并对着真值打分。

为什么要有这个脚本：
  * `tools/plate_ocr.py` 只负责"给一张裁好的车牌图，读出字符"，它不带检测。
  * YOLO 只负责"车牌在哪"。两者中间的**裁剪方式**没人验证过 —— 而这是会出问题的：
    实测 HyperLPR3 的识别网络偏好**紧裁剪**（0.9998），带背景反而略低（0.9990）。
    裁剪多留几个像素，准确率会掉。所以这里把 margin 做成参数并可以扫一遍。
  * `meta.jsonl` 里有每块车牌的真值字符串，所以端到端准确率是**能算出来的**，
    不用靠眼睛看。

用法:
    # 端到端评估 (对着 meta.jsonl 的车牌真值打分)
    python3 tools/detect_ocr.py --eval

    # 顺便扫一遍 margin, 找出最合适的裁剪外扩量
    python3 tools/detect_ocr.py --eval --sweep

    # 单独跑几张图
    python3 tools/detect_ocr.py captures/xxx/0001.jpg
"""
import argparse
import glob
import json
import os
import re
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import plate_ocr  # noqa: E402

SKIP = re.compile(r'[·\-\s\.]')


def norm(s):
    """真值里的 '苏A·PL12A' 和 OCR 输出统一成 '苏APL12A' 再比。"""
    return SKIP.sub('', (s or '')).upper()


def one_edit_apart(a, b):
    """只差一次插入/删除 —— 中文车牌 OCR 最常见的失败模式就是漏一位字符。"""
    if abs(len(a) - len(b)) != 1:
        return False
    lo, hi = (a, b) if len(a) < len(b) else (b, a)
    for i in range(len(hi)):
        if hi[:i] + hi[i + 1:] == lo:
            return True
    return False


def iou(a, b):
    ix0, iy0 = max(a[0], b[0]), max(a[1], b[1])
    ix1, iy1 = min(a[2], b[2]), min(a[3], b[3])
    inter = max(0.0, ix1 - ix0) * max(0.0, iy1 - iy0)
    ua = ((a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter)
    return inter / ua if ua > 0 else 0.0


def crop(img, box, margin):
    """按 margin 外扩(可为负=内缩)裁剪, 并夹到图像范围内。"""
    H, W = img.shape[:2]
    x0, y0, x1, y1 = box
    x0, y0 = x0 - margin, y0 - margin
    x1, y1 = x1 + margin, y1 + margin
    x0, y0 = max(0, int(round(x0))), max(0, int(round(y0)))
    x1, y1 = min(W, int(round(x1))), min(H, int(round(y1)))
    if x1 - x0 < 4 or y1 - y0 < 4:
        return None
    return img[y0:y1, x0:x1]


def load_gt(meta_path):
    """{图片基名: [(车牌真值, 框), ...]}"""
    gt = {}
    with open(meta_path, encoding='utf-8') as f:
        for ln in f:
            ln = ln.strip()
            if not ln:
                continue
            r = json.loads(ln)
            base = os.path.splitext(os.path.basename(r['file']))[0]
            items = [(o.get('plate', ''), o['box'])
                     for o in r['objects'] if o['cls'] == 'plate']
            if items:
                gt[base] = items
    return gt


def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument('images', nargs='*', help='要识别的图片; 不给就配 --eval')
    ap.add_argument('--eval', action='store_true',
                    help='对着 meta.jsonl 的车牌真值做端到端评估')
    ap.add_argument('--weights', default='weights/plate_yolo11n.pt',
                    help='车牌检测权重 (单类 plate)')
    ap.add_argument('--val-dir', default='datasets/vision_yolo3/images/val',
                    help='--eval 时用哪个目录的图 (默认 3 类数据集的 val)')
    ap.add_argument('--meta', default='datasets/vision/meta.jsonl')
    ap.add_argument('--conf', type=float, default=0.25)
    ap.add_argument('--iou-thr', type=float, default=0.5, help='判定"框中了"的 IoU 阈值')
    ap.add_argument('--margin', type=int, default=0,
                    help='裁剪外扩像素; 0=用 YOLO 框本身, 负数=往里缩')
    ap.add_argument('--sweep', action='store_true', help='扫一遍 margin 并给出对比表')
    ap.add_argument('--device', default='0')
    ap.add_argument('--limit', type=int, default=0, help='只评估前 N 张 (调试用)')
    return ap.parse_args()


def evaluate(model, gt, paths, args, margin, verbose=False):
    n_gt = n_hit = n_exact = n_loose = 0
    misses = []
    for p in paths:
        base = os.path.splitext(os.path.basename(p))[0]
        if base not in gt:
            continue
        img = cv2.imread(p)
        if img is None:
            continue
        r = model.predict(p, conf=args.conf, device=args.device, verbose=False)[0]
        # 只要 plate 类（按名字取, 免得依赖类别索引）
        boxes = [b.xyxy[0].tolist() for b in r.boxes
                 if r.names[int(b.cls.item())] == 'plate']
        for truth, gbox in gt[base]:
            n_gt += 1
            best, bi = 0.0, -1
            for i, b in enumerate(boxes):
                v = iou(gbox, b)
                if v > best:
                    best, bi = v, i
            if best < args.iou_thr:
                misses.append((base, truth, '没框住'))
                continue
            n_hit += 1
            c = crop(img, boxes[bi], margin)
            text, conf = plate_ocr.recognize(c) if c is not None else ('', 0.0)
            if norm(text) == norm(truth):
                n_exact += 1
            else:
                if one_edit_apart(norm(text), norm(truth)):
                    n_loose += 1
                if verbose:
                    print('    %-24s 真值 %-10s OCR %-10s %.3f  IoU %.2f'
                          % (base, truth, text or '(空)', conf, best))
    return dict(n_gt=n_gt, n_hit=n_hit, n_exact=n_exact, n_loose=n_loose, misses=misses)


def report(tag, s):
    g, h, e, l = s['n_gt'], s['n_hit'], s['n_exact'], s['n_loose']
    print('%-14s GT %3d | 检出 %3d (%5.1f%%) | 端到端全对 %3d (%5.1f%%) | 差一位 %2d'
          % (tag, g, h, 100.0 * h / max(g, 1), e, 100.0 * e / max(g, 1), l))
    if h:
        print('%-14s 检出后字符准确率 %5.1f%% (全对 %d + 差一位 %d)/%d'
              % ('', 100.0 * (e + l) / h, e, l, h))


def main():
    args = parse_args()
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    os.chdir(root)

    from ultralytics import YOLO
    model = YOLO(args.weights)
    print('检测权重 %s' % args.weights)
    print('类别     %s' % model.names)

    if args.images:
        for p in args.images:
            img = cv2.imread(p)
            if img is None:
                print('%-40s 读取失败' % p)
                continue
            r = model.predict(p, conf=args.conf, device=args.device, verbose=False)[0]
            print('\n%s' % p)
            for b in r.boxes:
                cls = r.names[int(b.cls.item())]
                box = b.xyxy[0].tolist()
                conf = float(b.conf.item())
                line = '  %-14s %.3f  框 %s' % (cls, conf, [round(v) for v in box])
                if cls == 'plate':
                    if args.margin and args.margin > 0:
                        line += '  (裁剪外扩 %dpx)' % args.margin
                    c = crop(img, box, args.margin)
                    text, oconf = plate_ocr.recognize(c) if c is not None else ('', 0.0)
                    line += '  -> 车牌 %s (%.3f)' % (text or '(未识别)', oconf)
                print(line)
        return 0

    val_dir = args.val_dir
    paths = sorted(glob.glob(os.path.join(val_dir, '*.jpg')))
    if not paths:
        sys.exit('找不到 val 图片: %s' % val_dir)
    gt = load_gt(args.meta)
    paths = [p for p in paths if os.path.splitext(os.path.basename(p))[0] in gt]
    if args.limit:
        paths = paths[:args.limit]
    print('评估图片 %d 张 (只统计有车牌真值的), IoU 阈值 %.2f' % (len(paths), args.iou_thr))
    print()

    if args.sweep:
        print('%-14s %s' % ('margin(px)', '结果'))
        best = None
        for m in (-6, -4, -2, 0, 2, 4, 8, 12):
            s = evaluate(model, gt, paths, args, m)
            report('margin=%d' % m, s)
            if best is None or s['n_exact'] > best[1]:
                best = (m, s['n_exact'])
        print()
        print('最佳 margin = %d px (%d 张全对)' % best)
    else:
        s = evaluate(model, gt, paths, args, args.margin)
        report('margin=%d' % args.margin, s)
        if s['misses']:
            print()
            print('未检出的 %d 块:' % len(s['misses']))
            for base, truth, why in s['misses'][:12]:
                print('  %-24s %-10s %s' % (base, truth, why))
    return 0


if __name__ == '__main__':
    sys.exit(main())
