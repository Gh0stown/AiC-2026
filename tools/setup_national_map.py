#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""国赛图层配套: 静态障碍区 + 国赛地图 + 从车道上的实拍。

为什么必须加静态障碍区（关键）:
    代价地图的障碍层吃的是 **2D /scan**, 没有高度信息 —— 实际能"看到"的只有
    **雷达扫描平面 z = 0.140 m** 上的东西。复赛三个物料都够高（立牌 0.150、
    车板 0.250、灯腿 0.340）所以都能被扫到; 但国赛的**垃圾桶 0.088 m**、
    **电动车 0.098 m** 都在扫描平面之下 -> 两个代价地图都看不见 -> 车会直接压过去
    （国赛规则: 碰撞 −1/次）。所以把它们写进**几何地图的障碍区**,
    全局规划器就会绕开, 局部代价地图也从静态层继承。
"""
from __future__ import annotations
import io, math, os, subprocess, sys

WS = '/home/ubuntu/AiC-2026'
os.chdir(WS)
sys.path.insert(0, WS + '/tools')
import gen_national_props as G

# 各物料的占地尺寸 (w=x 方向, d=y 方向), 用于生成障碍矩形
FOOT = {
    'bldg_a_fire': (0.62, 0.34), 'bldg_b': (0.55, 0.32),
    'bldg_c': (0.44, 0.30), 'bldg_d': (0.68, 0.30),
    'station': (0.34, 0.26),
    'bin_full': (0.07, 0.07), 'bin_empty': (0.07, 0.07),
    'ebike': (0.16, 0.06), 'ebike_toppled': (0.16, 0.10),
    'sign': (0.09, 0.03), 'heat_bad': (0.11, 0.04), 'heat_ok': (0.11, 0.04),
}

out = ['# 国赛图层的**不可通行**区域 (交给 tools/gen_map_from_arena.py --obstacles)',
       '#',
       '# 与 map_obstacles.yaml 的区别: 那份是复赛的街区/停车列; 这份是国赛新增物料。',
       '# 之所以要显式标成障碍, 见 tools/setup_national_map.py 顶部说明:',
       '#   * 楼宇/站房/指示牌/温度面板 高于雷达扫描平面 0.14 m -> 雷达自己能扫到',
       '#   * **垃圾桶 0.088 m / 电动车 0.098 m 低于扫描平面 -> 两个代价地图都看不见**',
       '#     不标进地图, 车会直接压过去 (国赛碰撞 −1/次)',
       '#',
       '# 矩形 [x0, y0, x1, y1], 世界坐标 (m); 由各物料占地 + 朝向算出, 已留 15 mm 余量。']
for p in G.PUT:
    w, d = FOOT[p['model']]
    yaw = p['yaw']
    # 旋转后的轴对齐包围盒
    ex = abs(w / 2 * math.cos(yaw)) + abs(d / 2 * math.sin(yaw))
    ey = abs(w / 2 * math.sin(yaw)) + abs(d / 2 * math.cos(yaw))
    m = 0.015
    out.append('  - [%.3f, %.3f, %.3f, %.3f]   # %s (%s)'
               % (p['x'] - ex - m, p['y'] - ey - m, p['x'] + ex + m, p['y'] + ey + m,
                  p['name'], p['model']))
obsp = 'src/competition_arena/config/map_obstacles_national.yaml'
io.open(obsp, 'w', encoding='utf-8').write('\n'.join(out) + '\n')
print('  障碍区文件: %s (%d 个矩形)' % (obsp, len(G.PUT)))

# 合并复赛障碍 + 国赛障碍
s = io.open('src/competition_arena/config/map_obstacles.yaml', encoding='utf-8').read()
lines = [l for l in s.splitlines() if l.strip().startswith('- [')]
merged = 'src/competition_arena/config/map_obstacles_national_merged.yaml'
nat_lines = [l.strip() for l in out if l.strip().startswith('- [')]
io.open(merged, 'w', encoding='utf-8').write(
    '# 复赛障碍 + 国赛物料障碍 (国赛地图用)\n' + '\n'.join(lines)
    + '\n' + '\n'.join(nat_lines) + '\n')
print('  合并障碍: %s' % merged)

# 生成国赛地图
r = subprocess.run(['.venv/bin/python', 'tools/gen_map_from_arena.py',
                    '--obstacles', merged, '--out', 'maps/arena_national'], capture_output=True, text=True)
print('  国赛地图: %s' % ((r.stdout or r.stderr).strip().splitlines() or ['?'])[-1])
for f in ('maps/arena_national.pgm', 'maps/arena_national.yaml'):
    print('    %-28s %s B' % (f, os.path.getsize(f) if os.path.isfile(f) else '缺'))
# 同步进机器人包的 maps/
import shutil
for ext in ('pgm', 'yaml'):
    src = 'maps/arena_national.%s' % ext
    if os.path.isfile(src):
        shutil.copyfile(src, 'src/competition_robot/maps/arena_national.%s' % ext)
print('  已同步到 src/competition_robot/maps/')
