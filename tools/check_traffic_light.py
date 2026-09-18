#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""看一眼机器人相机里能不能认出红绿灯（临时红绿灯的自检 / 以后识别器的雏形）。

用法:
    python3 tools/check_traffic_light.py            # 打印当前相机里各颜色像素数
    python3 tools/check_traffic_light.py --save a.png

返回: 占主导且够大的颜色 (red/yellow/green), 或者 none
判据: 像素数 > 40 且 比第二多的颜色多 3 倍 (仿真里颜色很干净, 阈值足够)
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np
import rospy
from sensor_msgs.msg import Image


def classify(rgb):
    """把 RGB 图分成 红/黄/绿/暗 四类, 返回每类像素数"""
    r = rgb[:, :, 0].astype(np.int16)
    g = rgb[:, :, 1].astype(np.int16)
    b = rgb[:, :, 2].astype(np.int16)
    bright = (r + g + b) > 90                      # 太暗的不算
    red = bright & (r > 110) & (g < 90) & (b < 90)
    yellow = bright & (r > 110) & (g > 90) & (b < 80)
    green = bright & (g > 110) & (r < 120) & (b < 120)
    return int(red.sum()), int(yellow.sum()), int(green.sum())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', default='/camera/rgb/image_raw')
    ap.add_argument('--save')
    a = ap.parse_args()

    rospy.init_node('check_traffic_light', anonymous=True)
    msg = rospy.wait_for_message(a.topic, Image, timeout=30)
    h, w, enc = msg.height, msg.width, msg.encoding
    arr = np.frombuffer(msg.data, dtype=np.uint8)
    if enc == 'rgb8':
        rgb = arr.reshape(h, w, 3)
    elif enc == 'bgr8':
        rgb = arr.reshape(h, w, 3)[:, :, ::-1]
    else:
        rospy.logerr('不支持的编码 %s' % enc)
        sys.exit(1)
    nr, ny, ng = classify(rgb)
    counts = {'red': nr, 'yellow': ny, 'green': ng}
    best = max(counts, key=counts.get)
    second = sorted(counts.values())[-2]
    ok = counts[best] > 40 and counts[best] > 3 * max(second, 1)
    print('  相机 %dx%d %s   像素数: 红 %d  黄 %d  绿 %d' % (w, h, enc, nr, ny, ng))
    print('  => 识别结果: %s' % (best if ok else 'none (没看到灯 / 太远太小)'))
    if a.save:
        try:
            from PIL import Image as PILImage
            PILImage.fromarray(rgb).save(a.save)
            print('  存图 ->', a.save)
        except Exception as e:
            print('  存图失败:', e)
    sys.exit(0 if ok else 2)


if __name__ == '__main__':
    main()
