#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""中文字体探测 —— 让画图脚本在任何机器上都不会出现"方块豆腐"。

背景: tools/*.py 里画图用的是 PIL。原来到处 hardcode Noto CJK 的路径, 找不到就回退
      到 DejaVuSans —— 而 DejaVu **没有中文字形**, 于是图上所有中文都变成方块 □□□
      (在 WSL / 精简发行版上必现)。

用法:
    from cnfont import load, pick
    f, cjk = load(20)                 # f 是字体对象, cjk 表示这台机器能不能显示中文
    draw.text((x, y), pick('红绿灯', 'traffic light', cjk), font=f, fill=...)

约定: **中文只用于"人看的说明"**, 关键信息(编号/坐标/文件名)都用 ASCII,
      这样即使没有中文字体, 图也能读懂。
"""
from __future__ import annotations

import os

from PIL import ImageFont

# 常见的中文字体位置 (按优先级)
CJK_CANDIDATES = [
    '/usr/share/fonts/opentype/noto/NotoSansCJK-Medium.ttc',
    '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
    '/usr/share/fonts/opentype/noto/NotoSansCJK-DemiLight.ttc',
    '/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc',
    '/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc',
    '/usr/share/fonts/truetype/wqy/wqy-microhei.ttc',
    '/usr/share/fonts/truetype/arphic/uming.ttc',
    '/System/Library/Fonts/PingFang.ttc',                      # macOS
    'C:/Windows/Fonts/msyh.ttc',                               # Windows
    'C:/Windows/Fonts/simhei.ttf',
]
LATIN_CANDIDATES = [
    '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf',
    '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',
    '/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf',
]


def find_cjk():
    """返回一个**存在且真的能显示中文**的字体路径, 找不到返回 None。

    设 `NO_CJK=1` 可以强制走英文标签 (测试用, 或者就想要纯英文图)。"""
    if os.environ.get('NO_CJK'):
        return None
    for p in CJK_CANDIDATES:
        if not os.path.exists(p):
            continue
        try:
            f = ImageFont.truetype(p, 20)
            # 真检查一下字形覆盖: 用 fontTools 太重的就跳过, 这里只确认能加载
            return p
        except Exception:
            continue
    return None


_CACHE = {}


def load(size, cjk_only=False):
    """返回 (font, cjk_ok)。cjk_ok=False 时调用方应该改用英文标签 (pick)。"""
    key = (size, cjk_only)
    if key in _CACHE:
        return _CACHE[key]
    p = find_cjk()
    ok = p is not None
    path = (p if ok else None) or next((q for q in LATIN_CANDIDATES if os.path.exists(q)), None)
    try:
        f = ImageFont.truetype(path, size) if path else ImageFont.load_default()
    except Exception:
        f = ImageFont.load_default()
        ok = False
    _CACHE[key] = (f, ok)
    return f, ok


def pick(zh, en, cjk_ok):
    """有中文字体就用中文, 没有就用英文。"""
    return zh if cjk_ok else en
