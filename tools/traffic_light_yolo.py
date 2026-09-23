#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""红绿灯状态识别（YOLO11n，直接检三种灯珠）。

模型检的是**亮着的那一颗灯珠**（不是灯箱），一帧通常只有 1 个框 —— 框的类别就是状态。
3 类: `red_light` / `yellow_light` / `green_light`。

用法
----
    python3 tools/traffic_light_yolo.py 图片...              # 逐张打印状态
    python3 tools/traffic_light_yolo.py --dir <目录>          # 批量
    python3 tools/traffic_light_yolo.py --json 图片           # 机器可读

程序里用
--------
    import sys; sys.path.insert(0, 'tools')
    from traffic_light_yolo import read_state
    state, conf, box = read_state(image_bgr)     # state: 'red'|'yellow'|'green'|'none'

实测（2026-09-23）
------------------
* 在它自己的训练分布上（采集世界，正对 1.0~2.0 m，强制灯态）：**90/90 = 100%**
* 在比赛场地 600 帧（独立数据）上：只要检出就几乎全对（147 次检出仅 2 错），
  但**漏检较多** —— 训练时只有"正对 + 1.0~2.0 m"，没有角度变化。
  详见 docs/issue_log.md 的 #12。
"""
from __future__ import annotations

import argparse
import glob
import json
import os
import sys

import cv2

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_MODEL = os.path.join(WS, 'weights', 'traffic_light_yolo11n.pt')

_MODEL = None


def _model(path=None):
    global _MODEL
    if _MODEL is None:
        from ultralytics import YOLO
        p = path or DEFAULT_MODEL
        if not os.path.isfile(p):
            raise RuntimeError('找不到模型 %s' % p)
        _MODEL = YOLO(p)
    return _MODEL


def read_states(image_bgr, conf=0.25, model=None):
    """返回 [(状态, 置信度, 框), ...]，按置信度从高到低。

    状态是 'red' / 'yellow' / 'green'（已去掉 `_light` 后缀）。
    """
    m = model or _model()
    r = m.predict(image_bgr, conf=conf, verbose=False)[0]
    out = []
    for b in r.boxes:
        name = m.names[int(b.cls.item())]
        out.append((name.replace('_light', ''), float(b.conf.item()),
                    [float(v) for v in b.xyxy[0].tolist()]))
    out.sort(key=lambda x: -x[1])
    return out


def read_state(image_bgr, conf=0.25, model=None):
    """只要一个结果 -> (状态, 置信度, 框)；没检出就是 ('none', 0.0, None)。"""
    s = read_states(image_bgr, conf=conf, model=model)
    return s[0] if s else ('none', 0.0, None)


def main(argv=None):
    ap = argparse.ArgumentParser(description='红绿灯状态识别 (YOLO, 检灯珠)')
    ap.add_argument('images', nargs='*', help='图片路径')
    ap.add_argument('--dir', help='目录（批量）')
    ap.add_argument('--model', default=DEFAULT_MODEL)
    ap.add_argument('--conf', type=float, default=0.25)
    ap.add_argument('--json', action='store_true', help='输出 JSON')
    args = ap.parse_args(argv)

    files = list(args.images)
    if args.dir:
        for ext in ('jpg', 'jpeg', 'png'):
            files += sorted(glob.glob(os.path.join(args.dir, '*.' + ext)))
    if not files:
        print(__doc__)
        return 2

    m = _model(args.model)
    if not args.json:
        print('模型 %s' % args.model)
        print('类别 %s' % m.names)
        print()

    results = []
    for f in files:
        im = cv2.imread(f)
        if im is None:
            results.append(dict(file=f, state='ERROR', conf=0.0, boxes=[]))
            continue
        st = read_states(im, conf=args.conf, model=m)
        state = st[0][0] if st else 'none'
        results.append(dict(file=f, state=state,
                            conf=st[0][1] if st else 0.0,
                            boxes=[dict(state=s, conf=round(c, 3),
                                        box=[round(v, 1) for v in bx]) for s, c, bx in st]))
        if not args.json:
            extra = ''
            if len(st) > 1:
                extra = '   (另检出 %d 个: %s)' % (
                    len(st) - 1, ', '.join('%s %.2f' % (s, c) for s, c, _ in st[1:]))
            print('%-42s -> %-7s %.2f%s' % (os.path.basename(f), state,
                                            st[0][1] if st else 0.0, extra))

    if args.json:
        print(json.dumps(results, ensure_ascii=False, indent=1))
    return 0


if __name__ == '__main__':
    sys.exit(main())
