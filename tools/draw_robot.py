#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""按 URDF 里的盒体视觉画一张机器人三维外观图 (用于确认自研形态)。"""
import re, os, sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

WS = '/home/ubuntu/AiC-2026'
URDF = os.path.join(WS, 'src/competition_robot/urdf/competition_robot.urdf')
OUT = os.path.join(WS, 'docs', 'robot_form.png')

for p in ('/mnt/c/Windows/Fonts/msyh.ttc', '/mnt/c/Windows/Fonts/simhei.ttf'):
    if os.path.isfile(p):
        font_manager.fontManager.addfont(p)
        matplotlib.rcParams['font.sans-serif'] = [font_manager.FontProperties(fname=p).get_name()]
        matplotlib.rcParams['axes.unicode_minus'] = False
        break

u = open(URDF, encoding='utf-8').read()
link = re.search(r'<link name="base_link">(.*?)</link>', u, re.S).group(1)
boxes = []
for v in re.finditer(r'<visual name="([^"]+)">\s*<origin xyz="([^"]+)"[^/]*/>\s*'
                     r'<geometry><box size="([^"]+)"', link):
    name, xyz, size = v.group(1), [float(x) for x in v.group(2).split()], \
                      [float(x) for x in v.group(3).split()]
    boxes.append((name, xyz, size))
print('车体视觉部件 %d 个: %s' % (len(boxes), ', '.join(b[0] for b in boxes)))

COL = {'frame': '#c9ced6', 'chassis': '#3a4048', 'label': '#e5541a', 'deck': '#c9ced6'}
fig = plt.figure(figsize=(10, 7), dpi=160)
ax = fig.add_subplot(111, projection='3d')


def cuboid(c, s):
    x, y, z = c; dx, dy, dz = [v / 2.0 for v in s]
    X = [x - dx, x + dx]
    Y = [y - dy, y + dy]
    Z = [z - dz, z + dz]
    faces = [((X[0], Y[0], Z[0]), (X[1], Y[0], Z[0]), (X[1], Y[1], Z[0]), (X[0], Y[1], Z[0])),
             ((X[0], Y[0], Z[1]), (X[1], Y[0], Z[1]), (X[1], Y[1], Z[1]), (X[0], Y[1], Z[1])),
             ((X[0], Y[0], Z[0]), (X[1], Y[0], Z[0]), (X[1], Y[0], Z[1]), (X[0], Y[0], Z[1])),
             ((X[0], Y[1], Z[0]), (X[1], Y[1], Z[0]), (X[1], Y[1], Z[1]), (X[0], Y[1], Z[1])),
             ((X[0], Y[0], Z[0]), (X[0], Y[1], Z[0]), (X[0], Y[1], Z[1]), (X[0], Y[0], Z[1])),
             ((X[1], Y[0], Z[0]), (X[1], Y[1], Z[0]), (X[1], Y[1], Z[1]), (X[1], Y[0], Z[1]))]
    return faces


for name, xyz, size in boxes:
    col = COL.get('chassis' if 'batt' in name or 'pc' in name else
                  'label' if 'label' in name else 'frame', '#c9ced6')
    ax.add_collection3d(Poly3DCollection(cuboid(xyz, size), facecolor=col,
                                         edgecolor='#7a7f88', linewidths=0.5, alpha=0.95))

# 轮子 + 传感器位置
for (x, y) in ((0.1272, 0.1454), (0.1272, -0.1454), (-0.0798, 0.1454), (-0.0798, -0.1454)):
    ax.add_collection3d(Poly3DCollection(cuboid((x, y, 0.0485), (0.097, 0.05, 0.097)),
                                         facecolor='#22262c', edgecolor='#555', alpha=0.9))
for pt, lbl, col in (((0.02, 0.0, 0.14), '雷达 z=0.14 (整圈 360°)', '#e53935'),
                     ((0.1491, -0.0005, 0.20), '相机 z=0.20', '#1565c0'),
                     ((0.1442, 0.0022, 0.1034), 'IMU', '#2e7d32')):
    ax.scatter(*pt, c=col, s=70, depthshade=False)
    ax.text(pt[0], pt[1], pt[2] + 0.028, lbl, fontsize=9, color=col)

ax.plot([-0.13, 0.16, 0.16, -0.13, -0.13], [-0.11, -0.11, 0.11, 0.11, -0.11],
        [0, 0, 0, 0, 0], ls='--', lw=1, c='#888')
ax.text(-0.05, 0.13, 0.0, '车身 0.3334 × 0.2187 m（与实车一致）', fontsize=9, color='#555')
ax.set_xlabel('x / m'); ax.set_ylabel('y / m'); ax.set_zlabel('z / m')
ax.set_title('自研低顶架外观（顶面 0.115 m）\n'
             '底板 + 四立柱 + 顶板 + 电池盒 + 工控机 + 队伍标牌；雷达装顶板上方 0.14 m，可整圈扫描',
             fontsize=10)
ax.set_xlim(-0.25, 0.30); ax.set_ylim(-0.22, 0.22); ax.set_zlim(0, 0.26)
ax.view_init(elev=22, azim=-58)
fig.tight_layout()
fig.savefig(OUT, facecolor='white')
print('已写出', OUT)
