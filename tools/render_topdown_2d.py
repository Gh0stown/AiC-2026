#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""从**几何真值**画一张场地俯视图（不依赖 Gazebo 渲染，也不依赖地面贴图）。

为什么要这个
------------
原来的俯视图 `src/competition_arena/docs/arena_preview.png` 是「从平面图提取出的
**矢量线条**预览」，内部车道线与外围画成同一种白色 —— 看图的人会以为"内部白线也建
成了墙"，与正文"白线是车道线、不是墙"互相打架。本脚本改为从真值源作图，并把三类
东西明确区分开：

  * 灰白粗条  = 围墙（`worlds/competition_arena.world` 的 arena_walls，实测只有 4 面）
  * 细白/黄线 = 车道线与停止线（`config/lane_lines.json`，仅地面标线，不参与碰撞）
  * 彩色标记  = 物料（立牌 / 车辆 / 红绿灯），带名字

用法:
    python3 tools/render_topdown_2d.py [-o 输出png]
"""
from __future__ import annotations

import argparse
import json
import os
import re
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt              # noqa: E402
from matplotlib import font_manager         # noqa: E402
from matplotlib.patches import Rectangle    # noqa: E402

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORLD = os.path.join(WS, 'src', 'competition_arena', 'worlds', 'competition_arena.world')
LINES = os.path.join(WS, 'src', 'competition_arena', 'config', 'lane_lines.json')


def use_cjk_font():
    """图上要写中文；matplotlib 默认字体没有 CJK 字形（会显示成方块）。
    ★ 注意不能用 "Droid Sans Fallback" —— 它只有 CJK、没有拉丁字母，
      结果英文/数字反而变方块。要选拉丁与 CJK 都全的字体。"""
    for p in ('/mnt/c/Windows/Fonts/msyh.ttc', '/mnt/c/Windows/Fonts/simhei.ttf',
              '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'):
        if os.path.isfile(p):
            try:
                font_manager.fontManager.addfont(p)
                name = font_manager.FontProperties(fname=p).get_name()
                matplotlib.rcParams['font.sans-serif'] = [name, 'DejaVu Sans']
                matplotlib.rcParams['axes.unicode_minus'] = False
                print('  字体: %s (%s)' % (name, p))
                return name
            except Exception as e:                                  # noqa: BLE001
                print('  字体加载失败 %s: %s' % (p, e))
    print('  !! 找不到中文字体, 中文会显示成方块')
    return None


def parse_world(path):
    """-> (墙面矩形列表, 物料 [(名, x, y)])"""
    txt = open(path, encoding='utf-8').read()
    walls = []
    m = re.search(r'<model name="arena_walls">(.*?)</model>', txt, re.S)
    if m:
        for cm in re.finditer(r'<collision name="wall_(\d+)">\s*'
                              r'<pose>([-\d.eE ]+)</pose>\s*'
                              r'<geometry><box><size>([-\d.eE ]+)</size>', m.group(1)):
            px, py, _z, _r, _p, _y = [float(v) for v in cm.group(2).split()]
            sx, sy, _sz = [float(v) for v in cm.group(3).split()]
            walls.append((px, py, sx, sy))
    props = []
    for inc in re.finditer(r'<include>\s*<uri>model://([^<]+)</uri>\s*'
                           r'(?:<name>([^<]*)</name>\s*)?'
                           r'(?:<pose>([-\d.eE ]+)</pose>)?', txt):
        model, name, pose = inc.group(1), inc.group(2), inc.group(3)
        if not name or not pose or 'sun' in model or 'ground_plane' in model:
            continue
        v = [float(x) for x in pose.split()]
        props.append((name, v[0], v[1]))
    return walls, props


def load_lines():
    d = json.load(open(LINES, encoding='utf-8'))
    return d.get('segments', [])


def main(argv=None):
    ap = argparse.ArgumentParser(description='从几何真值画场地俯视图')
    ap.add_argument('-o', '--out',
                    default=os.path.join(WS, 'src', 'competition_arena', 'docs',
                                         'arena_topdown.png'))
    a = ap.parse_args(argv)
    use_cjk_font()

    walls, props = parse_world(WORLD)
    segs = load_lines()
    kinds = {}
    for s in segs:
        kinds[s.get('type', '?')] = kinds.get(s.get('type', '?'), 0) + 1
    print('  围墙 %d 面 | 标线 %d 条 %s | 物料 %d 个' % (len(walls), len(segs), kinds, len(props)))

    fig, ax = plt.subplots(figsize=(9.6, 9.6), dpi=170)
    ax.set_facecolor('#0b0b0b')

    style = {'lane': ('#e8e8e8', 0.9), 'parking': ('#7fc4ff', 0.9), 'stop': ('#ffd54f', 1.6)}
    for s in segs:
        col, lw = style.get(s.get('type', 'lane'), ('#e8e8e8', 0.9))
        w = float(s.get('w', 0.02))
        if s['kind'] == 'h':
            y, x0, x1 = float(s['c']), float(s['a']), float(s['b'])
            ax.add_patch(Rectangle((min(x0, x1), y - w / 2), abs(x1 - x0), w,
                                   facecolor=col, edgecolor='none', zorder=2))
        else:
            x, y0, y1 = float(s['c']), float(s['a']), float(s['b'])
            ax.add_patch(Rectangle((x - w / 2, min(y0, y1)), w, abs(y1 - y0),
                                   facecolor=col, edgecolor='none', zorder=2))

    for px, py, sx, sy in walls:                    # 围墙: 灰白粗条
        ax.add_patch(Rectangle((px - sx / 2, py - sy / 2), sx, sy,
                               facecolor='#c9c9c9', edgecolor='#6f6f6f',
                               linewidth=0.7, zorder=4))

    groups = {'人偶立牌': ('#ffb300', 'o', 78), '车辆': ('#29b6f6', 's', 78),
              '红绿灯': ('#66bb6a', '^', 95)}
    seen = set()
    # 立牌按街区成组标注（逐个标名字会挤成一团，而图上真正要看的是"哪个街区几个"）
    byblk = {}
    for name, x, y in sorted(props):
        if name[0] in 'AB':
            byblk.setdefault(name[0], []).append((name, x, y))
        else:
            key = '车辆' if name.startswith('car') else '红绿灯'
            c, mk, ms = groups[key]
            ax.scatter([x], [y], c=c, marker=mk, s=ms, edgecolors='k', linewidths=0.6,
                       zorder=6, label=key if key not in seen else None)
            seen.add(key)
            ax.annotate(name, (x, y), xytext=(x + 0.07, y + 0.05), fontsize=8.0, zorder=7,
                        color='#ffffff', ha='left',
                        bbox=dict(boxstyle='round,pad=0.16', fc='#000000b0', ec='none'))
    for blk, items in sorted(byblk.items()):
        xs = [i[1] for i in items]; ys = [i[2] for i in items]
        c, mk, ms = groups['人偶立牌']
        ax.scatter(xs, ys, c=c, marker=mk, s=ms, edgecolors='k', linewidths=0.6,
                   zorder=6, label='人偶立牌' if '人偶立牌' not in seen else None)
        seen.add('人偶立牌')
        cnt = {}
        for name, x, y in items:
            edge = name.split('_')[1]
            cnt[edge] = cnt.get(edge, 0) + 1
        detail = ' + '.join('%s %d' % (k, v) for k, v in
                            sorted(cnt.items(), key=lambda kv: -kv[1]))
        ax.annotate('%s 街区 %d 个\n(%s)' % (blk, len(items), detail),
                    (sum(xs) / len(xs), sum(ys) / len(ys)),
                    xytext=(0.16, 0.20), textcoords='offset points',
                    fontsize=8.6, zorder=8, color='#ffe9b0', ha='left', va='bottom',
                    bbox=dict(boxstyle='round,pad=0.24', fc='#3a2c00cc', ec='#ffb30088'))

    ax.plot([-2.1, 2.1, 2.1, -2.1, -2.1], [-2.1, -2.1, 2.1, 2.1, -2.1],
            ls=':', lw=1.0, c='#00e5ff', zorder=5, label='白线可行驶范围 4.2×4.2 m')
    ax.set_xlim(-2.4, 2.4); ax.set_ylim(-2.4, 2.4)
    ax.set_aspect('equal')
    fig.suptitle('程序化生成后的场地俯视', fontsize=13.5, y=0.975)
    fig.text(0.5, 0.028,
             '真值源：world 的 arena_walls + config/lane_lines.json　｜　'
             '灰白粗条 = 围墙（仅外沿一圈，4 面）　｜　'
             '细线 = 地面标线（车道线 / 车位线 / 停止线，只在贴图上、不参与碰撞）　｜　'
             '标记 = 物料',
             ha='center', fontsize=8.6, color='#cfcfcf')
    ax.set_xlabel('世界坐标 x / m'); ax.set_ylabel('世界坐标 y / m')
    ax.grid(alpha=0.12, ls=':', lw=0.6)
    ax.legend(loc='lower right', fontsize=8.4, facecolor='#1c1c1c',
              edgecolor='#666666', labelcolor='#eeeeee', framealpha=0.92).set_zorder(11)
    fig.patch.set_facecolor('#0b0b0b')
    for t in ax.get_xticklabels() + ax.get_yticklabels():
        t.set_color('#bbbbbb')
    for sp in ax.spines.values():
        sp.set_color('#555555')
    fig.subplots_adjust(left=0.085, right=0.975, top=0.945, bottom=0.075)
    fig.savefig(a.out, facecolor=fig.get_facecolor())
    print('  已写出 %s' % os.path.relpath(a.out, WS))
    return 0


if __name__ == '__main__':
    sys.exit(main())
