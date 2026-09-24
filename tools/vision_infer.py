#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""视觉推理统一入口 —— 三个模型（立牌 / 红绿灯 / 车牌）一个模块。

给谁用
------
* `src/competition_robot/scripts/vision_detect.py`（ROS 节点）调它；
* 也可以直接命令行单测：`python3 tools/vision_infer.py 图片...`

三个任务的口径（很重要，别搞混）
--------------------------------
| 任务 | 模型 | 检的是什么 | 输出 |
|---|---|---|---|
| 人偶立牌 | `standee_yolo11n.pt` | **整块立牌**（2 类：社区 / 非社区人员）| 框 + 类别 |
| 红绿灯 | `traffic_light_yolo11n.pt` | ★ **直接检"亮着的那颗灯珠"**（3 类 red/yellow/green_light），**不是灯箱** | 框的类别 = 灯态 |
| 车牌 | `plate_yolo11n.pt` + HyperLPR3 | YOLO 框车牌 → 裁剪 → HyperLPR3 读字符 | 车牌字符串 |

模型文件都在 `weights/`（**已入库**，clone 就有；如需换模型直接替换同名文件）。
**缺模型不会崩**：
`read_*` 返回空结果, `available()` 会告诉你哪个不可用 —— 这样只训好一部分也能先跑起来。
"""
from __future__ import annotations

import argparse
import glob
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WEIGHTS = os.path.join(WS, 'weights')

MODEL_FILE = {
    'standee': 'standee_yolo11n.pt',
    'light': 'traffic_light_yolo11n.pt',
    'plate': 'plate_yolo11n.pt',
}
# 立牌模型的类别名 -> 统一标签（模型里叫什么都能对上）
STANDEE_LABEL = {
    'comm': 'community', 'community': 'community', 'standee': 'community',
    'normal': 'community', 'person': 'community',
    'non_comm': 'non_community', 'non_community': 'non_community',
    'noncommunity': 'non_community', 'f': 'non_community', 'intruder': 'non_community',
}
LABEL_CN = {'community': '社区人员', 'non_community': '非社区人员'}
LIGHT_CN = {'red': '红灯', 'yellow': '黄灯', 'green': '绿灯', 'none': '没看到灯'}

_CACHE = {}


def model_path(kind, models_dir=None):
    return os.path.join(models_dir or WEIGHTS, MODEL_FILE[kind])


def load(kind, models_dir=None, quiet=False):
    """加载模型（带缓存）。文件不在就返回 None，并说明一次。"""
    key = (kind, models_dir)
    if key in _CACHE:
        return _CACHE[key]
    p = model_path(kind, models_dir)
    m = None
    if os.path.isfile(p):
        try:
            from ultralytics import YOLO
            m = YOLO(p)
        except ImportError as e:                     # ★ 最常见的坑: 用错解释器
            if not quiet:
                print('[vision] ✗ 加载 %s 失败: %s' % (os.path.basename(p), e))
                print('[vision]   多半是用了**系统 python3**（rosrun 默认就是它）。')
                print('[vision]   请用仓库的 venv:  .venv/bin/python <脚本>')
                print('[vision]   （vision_detect.py 现在会自动切 venv, 这里只是说明原因）')
        except Exception as e:                       # noqa: BLE001
            if not quiet:
                print('[vision] 加载 %s 失败: %s' % (os.path.basename(p), e))
    elif not quiet:
        print('[vision] 没有模型 %s —— 这一项先跳过（clone 后 weights/ 需另行获取）' % p)
    _CACHE[key] = m
    return m


def available(models_dir=None):
    """-> {'standee': True/False, ...}（不加载模型，只看文件在不在）"""
    return dict((k, os.path.isfile(model_path(k, models_dir))) for k in MODEL_FILE)


# ---------------------------------------------------------------- 人偶立牌
def read_standees(image_bgr, conf=0.3, model=None, models_dir=None):
    """-> [{'cls': 'community'|'non_community', 'conf': float, 'box': [x0,y0,x1,y1]}]"""
    m = model or load('standee', models_dir)
    if m is None:
        return []
    r = m.predict(image_bgr, conf=conf, verbose=False)[0]
    out = []
    for b in r.boxes:
        raw = m.names[int(b.cls.item())]
        cls = STANDEE_LABEL.get(str(raw).lower().strip(), str(raw))
        out.append(dict(cls=cls, conf=float(b.conf.item()),
                        box=[float(v) for v in b.xyxy[0].tolist()], raw=raw))
    out.sort(key=lambda d: -d['conf'])
    return out


def count_standees(dets):
    """-> (总数, 社区数, 非社区数)"""
    c = sum(1 for d in dets if d['cls'] == 'community')
    n = sum(1 for d in dets if d['cls'] != 'community')
    return len(dets), c, n


# ---------------------------------------------------------------- 红绿灯
def read_light(image_bgr, conf=0.25, model=None, models_dir=None):
    """★ 检的是**亮着的灯珠**（不是灯箱）。-> (状态, 置信度, 框)

    状态: 'red' / 'yellow' / 'green' / 'none'
    """
    m = model or load('light', models_dir)
    if m is None:
        return ('none', 0.0, None)
    import traffic_light_yolo as T
    return T.read_state(image_bgr, conf=conf, model=m)


def read_lights(image_bgr, conf=0.25, model=None, models_dir=None):
    """所有灯珠（按置信度降序）-> [(状态, 置信度, 框), ...]；一帧通常只有 1 个。"""
    m = model or load('light', models_dir)
    if m is None:
        return []
    import traffic_light_yolo as T
    return T.read_states(image_bgr, conf=conf, model=m)


# ---------------------------------------------------------------- 车牌
def read_plate(image_bgr, conf=0.25, margin=10, model=None, models_dir=None,
               return_all=False):
    """YOLO 框车牌 -> 裁剪(外扩 margin) -> HyperLPR3 读字符。-> (字符串, 置信度, 框)

    ★ margin 别给 0：HyperLPR3 的**检测器**要背景（紧裁剪会直接失败），
      而**识别网络**偏好紧裁剪。我们用 `plate_ocr.recognize`（纯识别网络）
      喂裁剪图，所以 margin 取 10 左右安全（见 docs/vision_plan.md 的实测表）。
    """
    m = model or load('plate', models_dir)
    if m is None:
        return ('', 0.0, None)
    import plate_ocr
    import detect_ocr as D
    r = m.predict(image_bgr, conf=conf, verbose=False)[0]
    boxes = [b for b in r.boxes if str(r.names[int(b.cls.item())]).lower() == 'plate']
    if not boxes:
        return ('', 0.0, None)
    boxes.sort(key=lambda b: -float(b.conf.item()))
    dets = []
    for b in boxes:
        box = [float(v) for v in b.xyxy[0].tolist()]
        c = D.crop(image_bgr, box, margin)
        text, tconf = plate_ocr.recognize(c) if c is not None else ('', 0.0)
        dets.append((text, tconf, box))
    if return_all:
        return dets
    return dets[0]


# ---------------------------------------------------------------- CLI (单测用)
def main(argv=None):
    ap = argparse.ArgumentParser(description='视觉推理统一入口（单测用，不需要 ROS）')
    ap.add_argument('images', nargs='+')
    ap.add_argument('--models-dir', default=None)
    ap.add_argument('--conf', type=float, default=0.3)
    ap.add_argument('--margin', type=float, default=10, help='车牌裁剪外扩 (px)')
    ap.add_argument('--kind', default='all', choices=['all', 'standee', 'light', 'plate'])
    a = ap.parse_args(argv)

    av = available(a.models_dir)
    print('模型: ' + '  '.join('%s=%s' % (k, '有' if v else '缺') for k, v in av.items()))
    paths = []
    for p in a.images:
        paths += sorted(glob.glob(p)) if any(ch in p for ch in '*?[') else [p]
    for p in paths:
        img = cv2.imread(p)
        if img is None:
            print('%-34s 读不到' % os.path.basename(p))
            continue
        parts = []
        if a.kind in ('all', 'standee'):
            ds = read_standees(img, conf=a.conf, models_dir=a.models_dir)
            if av['standee']:
                tot, c, n = count_standees(ds)
                parts.append('人偶 %d（社区 %d / 非社区 %d）' % (tot, c, n))
        if a.kind in ('all', 'light'):
            st, cf, _ = read_light(img, conf=0.25, models_dir=a.models_dir)
            if av['light']:
                parts.append('红绿灯 %s(%.2f)' % (LIGHT_CN.get(st, st), cf))
        if a.kind in ('all', 'plate'):
            tx, cf, _ = read_plate(img, conf=0.25, margin=a.margin, models_dir=a.models_dir)
            if av['plate']:
                parts.append('车牌 %s(%.2f)' % (tx or '(没读到)', cf))
        print('%-34s %s' % (os.path.basename(p), '  '.join(parts)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
