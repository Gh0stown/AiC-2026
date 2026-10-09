#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把**比赛场地**(competition_arena.world) 里的物料导出成采集布局文件。

为什么需要它
------------
用户要求: **立牌就在比赛场地采集, 不要去采集世界** —— 采集世界里的立牌是人为摆成
圆环/正方形的, 信息量低 (姿态与真实布局无关), 而且要几千张。比赛场地里立牌就是
真实的 A/B 街区布局, 机位就是 5 个识别点位, 数据分布与部署完全一致, 几十张就够。

产物 config/arena_layout.yaml 的 objects 与 collect_layout.yaml 同构
(cls/name/model/x/y/yaw), 供 gen_collect_dataset.py 的 boxes_from_layout() 用。

用法: python3 tools/make_arena_layout.py
"""
from __future__ import annotations
import io, os, re, sys
import yaml

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORLD = os.path.join(WS, 'src', 'competition_arena', 'worlds', 'competition_arena.world')
OUT = os.path.join(WS, 'src', 'competition_arena', 'config', 'arena_layout.yaml')


def main():
    txt = io.open(WORLD, encoding='utf-8').read()
    objs = []
    for m in re.finditer(r'<include>\s*<uri>model://([^<]+)</uri>\s*'
                         r'(?:<name>([^<]*)</name>\s*)?'
                         r'(?:<pose>([-\d.eE ]+)</pose>)?', txt):
        model, name, pose = m.group(1), m.group(2), m.group(3)
        if not name or not pose or 'sun' in model or 'ground_plane' in model:
            continue
        v = [float(x) for x in pose.split()]
        x, y, yaw = v[0], v[1], (v[5] if len(v) > 5 else 0.0)
        if model.startswith('standee_'):
            cls = 'non_community' if '_F' in model else 'standee'
        elif model.startswith('car_'):
            cls = 'plate'
        elif model.startswith('traffic_light'):
            cls = 'traffic_light'
        else:
            continue
        objs.append(dict(cls=cls, name=name, model=model,
                         x=round(x, 4), y=round(y, 4), yaw=round(yaw, 6)))
    # 红绿灯的朝向要按"灯面朝向"给 (与 collect_layout 一致), 用 traffic_lights.yaml 覆盖
    tls = yaml.safe_load(io.open(os.path.join(WS, 'src', 'competition_arena', 'config',
                                             'traffic_lights.yaml'), encoding='utf-8'))
    tlm = {L['name']: L for L in tls['lights']}
    for o in objs:
        if o['cls'] == 'traffic_light' and o['name'] in tlm:
            L = tlm[o['name']]
            o['x'], o['y'], o['yaw'] = float(L['x']), float(L['y']), float(L.get('yaw', 0.0))
    layout = dict(
        note='比赛场地采集布局 (tools/make_arena_layout.py 从 competition_arena.world 导出)',
        world=WORLD,
        objects=objs,
    )
    with io.open(OUT, 'w', encoding='utf-8') as f:
        yaml.safe_dump(layout, f, allow_unicode=True, sort_keys=False)
    import collections
    print('导出 %d 个物料 -> %s' % (len(objs), os.path.relpath(OUT, WS)))
    print('  ', dict(collections.Counter(o['cls'] for o in objs)))


if __name__ == '__main__':
    sys.exit(main())
