#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""meta.jsonl -> YOLO 数据集 (images/labels + data.yaml)

gen_vision_dataset.py 已经把每个目标的像素框写进了 meta.jsonl (由真值位姿做几何投影),
这个脚本把它转成 Ultralytics 直接能吃的目录结构, 不重新生成图片 (默认用软链接, 省 39MB)。

★★★ 重要: 这批框有系统性竖直偏移 ★★★
  投影出来的框整体比画面里的目标**高约一个车牌高**(0.66 m 处约 26 px / 0.0175 m),
  所以「YOLO 框车牌 -> 裁剪 -> OCR」端到端是 **0/37** (检测 37/37 全中, 但裁到的是保险杠)。
  同样地, plate mAP50=0.995 只是"YOLO 把错的框学得很准", **不代表标签正确**。
  转化本身没问题(越界 0 / 丢弃 0), 但**在这些框修好之前不要拿去训最终模型**。
  证据、已排除的怀疑、两条修法见 docs/issue_log.md #10。

划分方式: **按整段 scenario 留出 val**, 而不是随机抽帧 —— 同一段轨迹里相邻帧几乎一样,
随机抽会让 val 虚高。background(空帧) 按文件名奇偶对半分。

用法:
    python3 tools/meta_to_yolo.py                     # 默认划分, 软链接
    python3 tools/meta_to_yolo.py --copy              # 复制图片而不是软链接
    python3 tools/meta_to_yolo.py --val-scenarios car_2,A_south,tl_top,near_tl_bot
    python3 tools/meta_to_yolo.py --min-box-px 16     # 丢掉太小的框 (远处目标)
"""
import argparse
import json
import os
import shutil
import sys

CLASSES = ['standee', 'non_community', 'traffic_light', 'plate']

# 默认留出: 每个类别各挑一段完整轨迹, 保证 val 里四类都有
DEFAULT_VAL = ['car_2', 'A_south', 'tl_top', 'near_tl_bot']


def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument('--meta', default='datasets/vision/meta.jsonl')
    ap.add_argument('--images', default='datasets/vision/images',
                    help='源图片目录 (meta 里的 file 是相对 datasets/vision 的路径)')
    ap.add_argument('--out', default='datasets/vision_yolo')
    ap.add_argument('--val-scenarios', default=','.join(DEFAULT_VAL))
    ap.add_argument('--min-box-px', type=float, default=8.0,
                    help='框的短边小于这个值就丢掉 (远处目标, OCR 也用不上)')
    ap.add_argument('--drop-classes', default='',
                    help='不要的类别, 逗号分隔。默认不丢。'
                         '红绿灯状态用 tools/traffic_light.py (OpenCV 读颜色) 已经 6/6, '
                         '不需要 YOLO 管, 所以常加 --drop-classes traffic_light')
    ap.add_argument('--copy', action='store_true', help='复制图片而不是软链接')
    ap.add_argument('--force', action='store_true', help='out 已存在也继续 (覆盖 labels)')
    return ap.parse_args()


def main():
    args = parse_args()
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    os.chdir(root)

    val_scen = {s.strip() for s in args.val_scenarios.split(',') if s.strip()}
    drop = {s.strip() for s in args.drop_classes.split(',') if s.strip()}
    unknown = drop - set(CLASSES)
    if unknown:
        sys.exit('--drop-classes 里有未知类别: %s (可选: %s)'
                 % (', '.join(sorted(unknown)), ', '.join(CLASSES)))
    names = [c for c in CLASSES if c not in drop]
    if not names:
        sys.exit('不能把所有类别都丢掉')

    rows = []
    with open(args.meta, encoding='utf-8') as f:
        for ln in f:
            ln = ln.strip()
            if ln:
                rows.append(json.loads(ln))
    print('读入 meta: %d 行' % len(rows))
    print('val 留出的 scenario: %s' % ', '.join(sorted(val_scen)))
    if drop:
        print('丢掉类别: %s  -> 保留 %s (索引重排)' % (', '.join(sorted(drop)), names))

    if os.path.isdir(args.out) and not args.force:
        print('!! %s 已存在, 加 --force 才会覆盖' % args.out)
        return 1

    # 先算划分, 便于打印统计
    assign = {}
    bg_i = 0
    for r in rows:
        sc = r['scenario']
        if sc in val_scen:
            assign[r['file']] = 'val'
        elif sc == 'background':
            assign[r['file']] = 'val' if (bg_i % 2) else 'train'
            bg_i += 1
        else:
            assign[r['file']] = 'train'

    for sub in ('train', 'val'):
        os.makedirs(os.path.join(args.out, 'images', sub), exist_ok=True)
        os.makedirs(os.path.join(args.out, 'labels', sub), exist_ok=True)

    src_root = os.path.abspath('datasets/vision')
    stats = {s: {'img': 0, 'box': {}, 'drop': 0, 'oob': 0} for s in ('train', 'val')}
    bad = []

    for r in rows:
        sub = assign[r['file']]
        src = os.path.join(src_root, r['file'])
        if not os.path.isfile(src):
            bad.append(r['file'])
            continue
        base = os.path.splitext(os.path.basename(r['file']))[0]
        dst_img = os.path.abspath(os.path.join(args.out, 'images', sub, base + '.jpg'))
        if args.copy:
            shutil.copyfile(src, dst_img)
        else:
            if os.path.lexists(dst_img):
                os.remove(dst_img)
            os.symlink(os.path.relpath(src, os.path.dirname(dst_img)), dst_img)

        # 需要图片宽高来做归一化 —— 从 meta 的框本身推不出来, 读一次文件头
        import cv2
        im = cv2.imread(src)
        if im is None:
            bad.append(r['file'])
            continue
        H, W = im.shape[:2]

        lines = []
        for o in r['objects']:
            if o['cls'] in drop:
                continue
            x0, y0, x1, y1 = o['box']
            if not (0 <= x0 < x1 <= W and 0 <= y0 < y1 <= H):
                stats[sub]['oob'] += 1
                continue
            if min(x1 - x0, y1 - y0) < args.min_box_px:
                stats[sub]['drop'] += 1
                continue
            cx = (x0 + x1) / 2.0 / W
            cy = (y0 + y1) / 2.0 / H
            bw = (x1 - x0) / W
            bh = (y1 - y0) / H
            lines.append('%d %.6f %.6f %.6f %.6f' % (names.index(o['cls']), cx, cy, bw, bh))
            stats[sub]['box'][o['cls']] = stats[sub]['box'].get(o['cls'], 0) + 1
        with open(os.path.join(args.out, 'labels', sub, base + '.txt'), 'w') as f:
            f.write(''.join(l + '\n' for l in lines))
        stats[sub]['img'] += 1

    yaml_path = os.path.abspath(args.out).replace('\\', '/')
    with open(os.path.join(args.out, 'data.yaml'), 'w') as f:
        f.write('# 由 tools/meta_to_yolo.py 生成, 源: %s\n' % args.meta)
        f.write('path: %s\n' % yaml_path)
        f.write('train: images/train\n')
        f.write('val: images/val\n')
        f.write('nc: %d\n' % len(names))
        f.write('names: %s\n' % names)

    print()
    for sub in ('train', 'val'):
        s = stats[sub]
        tot = sum(s['box'].values())
        print('%-5s %3d 张  框 %4d  丢弃(太小) %3d  越界 %d' % (sub, s['img'], tot, s['drop'], s['oob']))
        for c in names:
            print('        %-14s %4d' % (c, s['box'].get(c, 0)))
    if bad:
        print('\n!! 读不到的图片 %d 张, 例如: %s' % (len(bad), bad[:3]))
        return 1
    print('\ndata.yaml -> %s/data.yaml' % args.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())
