#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把人物立牌按街区摆进 world。

依据:
  * 复赛任务示意图 —— A 街区人偶朝 北/南/西 三个方向, B 街区朝 北/东 两个方向
    (示意图上每组人偶下面都有橙黄箭头表朝向: A 是 ←↓↑, B 是 ↑→)
  * 用户要求 —— 每个方向都要有人; 两个非社区人员(F1/F2)必须都摆、不重复,
                建议 A/B 各一个

街区矩形 (白线是**车道线**, 街区就是车道之间的区域; 由 build_arena 提取):
    A 街区  x[-1.472, 0.843] × y[0.847, 1.468]   2.32 × 0.62 m   (高 62cm ~ 示意图 60cm)
    B 街区  x[-1.475, -0.407] × y[-0.537, 0.197]  1.07 × 0.73 m

朝向与坐标:
    人偶模型板面朝 +x, 所以 yaw 决定面向:  北=+y(90°)  南=-y(-90°)  东=+x(0°)  西=-x(180°)
    摆在各边**内侧** inset 处 (inset 要盖住立牌底座的进深, 免得伸出车道线外)。

用法:
    python3 tools/setup_standees.py            # 生成 <include> 插进 world (幂等)
    python3 tools/setup_standees.py --remove   # 从 world 移除
    python3 tools/setup_standees.py --show     # 只打印清单
"""
from __future__ import annotations

import argparse
import os
import re
import sys

import yaml

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PKG = os.path.join(WS, 'src', 'competition_arena')
WORLD = os.path.join(PKG, 'worlds', 'competition_arena.world')
CFG = os.path.join(PKG, 'config', 'standees.yaml')
BEGIN = '    <!-- ===== 人物立牌 (tools/setup_standees.py 生成) ===== -->'
END = '    <!-- ===== 人物立牌结束 ===== -->'

INSET = 0.032          # 从街区边线往内缩, 盖住立牌底座进深(0.045/2=0.0225)并留余量
# ★ 同一方向的几个人偶**紧凑成一组**、居中摆在该边上。
#   识别是"机器人开到固定点位、原地转向拍照", 一组人偶挤在一起才容易一张图拍全;
#   沿整条边铺开的话, 离得远的那个要么出画、要么像素太小。
CLUSTER_GAP = 0.090    # 同组内相邻人偶的间距 (m)

# 边 -> (朝向 yaw, 沿哪根轴排开)
EDGE = {
    'north': (90.0, 'x'),
    'south': (-90.0, 'x'),
    'east':  (0.0, 'y'),
    'west':  (180.0, 'y'),
}


def edge_points(rect, edge, n):
    """在街区的某条边上取 n 个位置 (世界坐标 + yaw)。"""
    x0, y0, x1, y1 = rect
    yaw, along = EDGE[edge]
    if edge == 'north':
        fixed, lo, hi = y1 - INSET, x0, x1
    elif edge == 'south':
        fixed, lo, hi = y0 + INSET, x0, x1
    elif edge == 'east':
        fixed, lo, hi = x1 - INSET, y0, y1
    else:
        fixed, lo, hi = x0 + INSET, y0, y1
    # 居中成组: 以该边中点为中心, 按 CLUSTER_GAP 排开
    mid = (lo + hi) / 2.0
    span = CLUSTER_GAP * (n - 1)
    if span > (hi - lo):                     # 边太短就按边宽压缩
        span = hi - lo
    out = []
    for i in range(n):
        v = mid - span / 2.0 + (i * span / (n - 1) if n > 1 else 0.0)
        if along == 'x':
            out.append((v, fixed, yaw))
        else:
            out.append((fixed, v, yaw))
    return out


def load_plan():
    cfg = yaml.safe_load(open(CFG))
    return cfg['standees'], cfg.get('blocks', [])


def build_includes(plan):
    out = [BEGIN]
    for blk in plan:
        rect = [float(v) for v in blk['rect']]
        for e in blk['edges']:
            people = e['people']
            pts = edge_points(rect, e['edge'], len(people))
            for i, (p, (x, y, yaw)) in enumerate(zip(people, pts), 1):
                out.append('    <include>')
                out.append('      <uri>model://%s</uri>' % p)
                out.append('      <name>%s_%s_%d</name>' % (blk['name'], e['edge'], i))
                out.append('      <pose>%.4f %.4f 0 0 0 %.4f</pose>'
                           % (x, y, yaw * 3.141592653589793 / 180.0))
                out.append('    </include>')
    out.append(END)
    return '\n'.join(out) + '\n'


def patch_world(plan, remove=False):
    s = open(WORLD).read()
    s = re.sub(re.escape(BEGIN) + r'.*?' + re.escape(END) + r'\n?', '', s, flags=re.S)
    if not remove:
        if '</world>' not in s:
            raise SystemExit('world 里没有 </world>')
        s = s.replace('</world>', build_includes(plan) + '  </world>', 1)
    open(WORLD, 'w').write(s)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--remove', action='store_true')
    ap.add_argument('--show', action='store_true')
    a = ap.parse_args()
    standees, plan = load_plan()
    by_name = {s['name']: s for s in standees}

    if a.show:
        for blk in plan:
            x0, y0, x1, y1 = blk['rect']
            print('  %s 街区  x[%.3f, %.3f] y[%.3f, %.3f]  (%.2f x %.2f m)'
                  % (blk['name'], x0, x1, y0, y1, x1 - x0, y1 - y0))
            for e in blk['edges']:
                pts = edge_points([float(v) for v in blk['rect']], e['edge'], len(e['people']))
                print('    %-6s 朝 %-5s x%d: %s' % (e['edge'], e['edge'], len(e['people']),
                      ', '.join('%s@(%+.2f,%+.2f)' % (p.replace('standee_', ''), x, y)
                                for p, (x, y, _) in zip(e['people'], pts))))
        return

    patch_world(plan, remove=a.remove)
    if a.remove:
        print('已从 world 移除人物立牌')
        return
    total = sum(len(e['people']) for b in plan for e in b['edges'])
    print('已把 %d 个人偶摆进 world:' % total)
    for blk in plan:
        for e in blk['edges']:
            print('  %s 街区 %-6s x%d  %s' % (blk['name'], e['edge'], len(e['people']),
                  ' '.join(p.replace('standee_', '') for p in e['people'])))
    print('  注意: 改了 world 要重启仿真才生效')
    # ★ 干涉检查: 立牌不能压到红绿灯的脚上。
    #   立牌是 static、红绿灯不是, 压上去会把灯顶起来 (踩过: tl_top 被 c02 顶歪一个角)
    tl_cfg = os.path.join(PKG, 'config', 'traffic_lights.yaml')
    if os.path.isfile(tl_cfg):
        import math
        LEG, FOOT_X, FOOT_Y = 0.615 / 2.0, 0.140, 0.050
        obstacles = []
        for L in yaml.safe_load(open(tl_cfg))['lights']:
            yaw = float(L.get('yaw', 0.0))
            for sgn in (-1, 1):
                if abs(yaw - math.pi / 2) < 0.1:       # 朝 +y: 腿沿 x 排开
                    lx = float(L['x']) + sgn * LEG
                    ly = float(L['y'])
                    obstacles.append((lx, ly, FOOT_Y, FOOT_X))
                else:                                   # 朝 +x: 腿沿 y 排开
                    lx = float(L['x'])
                    ly = float(L['y']) + sgn * LEG
                    obstacles.append((lx, ly, FOOT_X, FOOT_Y))
        hit = 0
        for blk in plan:
            rect = [float(v) for v in blk['rect']]
            for e in blk['edges']:
                pts = edge_points(rect, e['edge'], len(e['people']))
                for nm, (x, y, _) in zip(e['people'], pts):
                    for (ox, oy, ow, oh) in obstacles:
                        if abs(x - ox) < (ow / 2 + 0.05) and abs(y - oy) < (oh / 2 + 0.06):
                            print('  ⚠ %s @(%+.3f,%+.3f) 可能压到红绿灯脚 @(%+.3f,%+.3f)'
                                  % (nm.replace('standee_', ''), x, y, ox, oy))
                            hit += 1
        if hit == 0:
            print('  干涉检查: 10 个立牌都没压到红绿灯的脚 ✓')

    uniq = [p for b in plan for e in b['edges'] for p in e['people']]
    if len(uniq) != len(set(uniq)):
        print('  ⚠ 有重复摆放的人偶!')
    for f in ('standee_F1', 'standee_F2'):
        if f in by_name and f not in uniq:
            print('  ⚠ %s (非社区人员) 没有摆!' % f)


if __name__ == '__main__':
    main()
