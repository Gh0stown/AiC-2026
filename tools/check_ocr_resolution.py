#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""车牌 OCR 分辨率下界验证 —— 回答"多少像素还读得出来"。

背景: tools/gen_recognition_points.py 算出来, 在**不压车道线**的前提下, 机器人
      最近只能站到车牌前约 0.70 m, 车牌在 1280x960 画面里约 **157 px 宽**。
      所以必须确认 157 px 这个分辨率 HyperLPR3 还认不认。

做法: 把官方车牌素材 (200x81) 按不同宽度重采样, 再喂给 plate_ocr 的识别网络,
      统计每个宽度下的正确率。这模拟了"车牌在相机里只占 N px"的情形。

用法:
    .venv/bin/python tools/check_ocr_resolution.py
    .venv/bin/python tools/check_ocr_resolution.py --widths 80 100 120 140 157 180 200
    .venv/bin/python tools/check_ocr_resolution.py --blur 0.6     # 加一点模糊, 更接近实拍
"""
from __future__ import annotations

import argparse
import os
import pwd
import sys

import cv2
import numpy as np

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
# ★ 注意: plate_ocr 在 import 时会把 HOME 改到 .cache/home (受限沙箱用), 所以
#   素材路径必须在 import 之前、且不依赖 $HOME 地定下来。
_REAL_HOME = pwd.getpwuid(os.getuid()).pw_dir
SRC = os.path.join(_REAL_HOME, '复赛资料', '车辆识别')

import plate_ocr                                        # noqa: E402

PLATES = [('车牌一.png', '苏A·B8Q62'), ('车牌二.png', '鄂D·7B5Q2'), ('车牌三.png', '苏A·PL12A')]


def norm(t):
    """归一化: HyperLPR3 的输出**不含分隔点**, 比较前必须去掉 '·' 和空白"""
    return (t or '').replace('·', '').replace(' ', '').replace('.', '').strip()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--widths', type=int, nargs='+',
                    default=[80, 100, 120, 140, 157, 180, 200])
    ap.add_argument('--blur', type=float, default=0.0,
                    help='高斯模糊 sigma (px, 在原图尺度); 0=不模糊')
    a = ap.parse_args()

    imgs = []
    for fn, truth in PLATES:
        p = os.path.join(SRC, fn)
        im = cv2.imread(p)
        if im is None:
            sys.exit('读不到素材: %s' % p)
        imgs.append((fn, truth, im))
    print('素材 %d 张, 原始 %dx%d, 识别网络: HyperLPR3 (仅识别, 不做检测)'
          % (len(imgs), imgs[0][2].shape[1], imgs[0][2].shape[0]))
    print()
    print('%8s %10s %10s %10s' % ('宽度(px)', '识别正确', '平均置信度', '每字像素'))
    print('-' * 44)
    rows = []
    for w in a.widths:
        ok, confs, n, detail = 0, [], 0, []
        for fn, truth, im in imgs:
            h = max(8, int(round(im.shape[0] * w / float(im.shape[1]))))
            small = cv2.resize(im, (w, h), interpolation=cv2.INTER_AREA)
            if a.blur > 0:
                small = cv2.GaussianBlur(small, (0, 0), a.blur)
            text, conf = plate_ocr.recognize(small)
            n += 1
            hit = (norm(text) == norm(truth))
            detail.append('%s:%s' % (fn.split('.')[0][-1], text or '空'))
            ok += hit
            confs.append(conf if hit else 0.0)
        avg = float(np.mean(confs)) if confs else 0.0
        # "苏A·B8Q62" 共 8 个字形 (含分隔点)
        print('%8d %8d/%d %10.3f %10.2f   %s'
              % (w, ok, n, avg, w / 8.0, ' '.join(detail)))
        rows.append((w, ok, n))
    print()
    good = [w for w, ok, n in rows if ok == n]
    print('全部正确的宽度: %s' % (', '.join('%d px' % w for w in good) if good else '(无)'))
    if good:
        print('→ 最小可用宽度 %d px; 识别点位实测 157 px, 余量 %.0f%%'
              % (min(good), (157.0 / min(good) - 1) * 100))
    return 0


if __name__ == '__main__':
    sys.exit(main())
