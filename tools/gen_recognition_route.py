#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把识别点位并进巡检环线, 生成"最终路线" + 路线图。

思路 (和你给的那张 patrol_trace.png 一致):
    最终路线 = 原来那条巡检环线 (config/waypoints.yaml), 只是沿途要停下来拍照。
    所以这里把 10 个识别点**按在环线上的弧长**插进航点序列 —— 不另造路线。

做法:
    1) 用 waypoints.yaml 的 start + waypoints + 回起点 拼出环线折线
    2) 把每个识别点投影到环线上, 取弧长
    3) 航点/识别点按弧长排序 -> 一条有序路线
    4) 写 config/recognition_route.yaml (patrol.py 可直接跑, yaw=拍照朝向)
    5) 画 docs/recognition_route.png

用法:
    python3 tools/gen_recognition_route.py
    python3 tools/gen_recognition_route.py --trace maps/patrol_trace_demo.csv   # 叠加实测轨迹
    python3 tools/gen_recognition_route.py --no-trace
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import sys

import yaml
from PIL import Image, ImageDraw, ImageFont

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ARENA = os.path.join(WS, 'src', 'competition_arena')
ROBOT = os.path.join(WS, 'src', 'competition_robot')
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from map_pixels import SRC, x2px, y2py, px2x, px2y      # noqa: E402

WAYPOINTS = os.path.join(ROBOT, 'config', 'waypoints.yaml')
RECOG = os.path.join(ROBOT, 'config', 'recognition_points.yaml')
OUT_YAML = os.path.join(ROBOT, 'config', 'recognition_route.yaml')
OUT_PNG = os.path.join(WS, 'docs', 'recognition_route.png')

# 颜色 (深色底图, 全部用亮色)
C_TRACE = (255, 40, 40)
C_LOOP = (255, 170, 40)
C_START = (0, 90, 255)
C_WP = (0, 200, 0)
C_TASK = {'traffic_light': (255, 225, 0), 'standee': (0, 230, 255),
          'plate': (255, 140, 20)}
TASK_CN = {'traffic_light': '红绿灯', 'standee': '人偶', 'plate': '车牌'}
PREFIX = {'traffic_light': 'T', 'standee': 'S', 'plate': 'P'}
SHORT = {'traffic_light': '灯', 'standee': '人', 'plate': '牌'}
SHORT_EN = {'traffic_light': 'TL', 'standee': 'P', 'plate': 'PL'}


from cnfont import load as _load_font, pick as _pick       # noqa: E402


def font(size, cjk=False):
    """★ 没有中文字体时自动回退到英文字体 (见 tools/cnfont.py)。
    返回值兼容老用法: 直接当字体对象用; 中文标签请配合 CJK_OK 判断。"""
    f, _ok = _load_font(size)
    return f


def text_w(d, txt, f):
    """Pillow<8 没有 textlength"""
    for fn in ('textlength', 'textsize'):
        try:
            r = getattr(d, fn)(txt, font=f)
            return r if isinstance(r, (int, float)) else r[0]
        except AttributeError:
            continue
    return f.getsize(txt)[0]


def load_loop():
    """返回 (start, [(name, x, y)], 环线折线点, 各段累计弧长)"""
    cfg = yaml.safe_load(open(WAYPOINTS))
    start = tuple(cfg['start'])
    wps = []
    for w in cfg['waypoints']:
        wps.append((w['name'], float(w['world'][0]), float(w['world'][1])))
    ret = tuple(cfg.get('return_to', start))
    poly = [start] + [(x, y) for _, x, y in wps] + [ret]
    return start, wps, ret, poly


def project(poly, p):
    """把点 p 投到折线上, 返回 (弧长, 投影点, 距离)"""
    best = (None, None, 1e9)
    acc = 0.0
    for i in range(len(poly) - 1):
        ax, ay = poly[i]
        bx, by = poly[i + 1]
        vx, vy = bx - ax, by - ay
        L2 = vx * vx + vy * vy
        t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p[0] - ax) * vx + (p[1] - ay) * vy) / L2))
        qx, qy = ax + t * vx, ay + t * vy
        d = math.hypot(p[0] - qx, p[1] - qy)
        if d < best[2]:
            best = (acc + t * math.sqrt(L2), (qx, qy), d)
        acc += math.sqrt(L2)
    return best


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--trace', default=os.path.join(WS, 'maps', 'patrol_trace_demo.csv'))
    ap.add_argument('--no-trace', action='store_true')
    ap.add_argument('--out-yaml', default=OUT_YAML)
    ap.add_argument('--out-png', default=OUT_PNG)
    a = ap.parse_args()

    start, wps, ret, poly = load_loop()
    rec = yaml.safe_load(open(RECOG))['points']

    # ---- 1) 识别点 + 航点 一起按弧长排序 ----
    items = []
    for name, x, y in wps:
        s, q, d = project(poly, (x, y))
        items.append(dict(kind='waypoint', name=name, x=x, y=y, yaw=None,
                          s=s, q=q, off=d))
    for i, p in enumerate(rec):
        x, y, yaw = p['pose']
        s, q, d = project(poly, (x, y))
        items.append(dict(kind='recog', name=p['name'], task=p['task'],
                          x=x, y=y, yaw=yaw, s=s, q=q, off=d, metric=p['metric'],
                          lane=p.get('lane', ''), label='%s%d' % (PREFIX[p['task']],
                                                                  sum(1 for q2 in rec[:i]
                                                                      if q2['task'] == p['task']) + 1)))
    items.sort(key=lambda it: it['s'])
    for i, it in enumerate(items, 1):
        it['order'] = i

    # ---- 2) 写路线 yaml (patrol.py 的 schema) ----
    # 倒车入库 (issue #6): 原配置只在旧的 waypoints.yaml 里, 新路线要把它带过来,
    #   patrol.py 跑完航点后会: 先用 move_base 开到 from 点 -> 原地转到背离库位 ->
    #   激光对墙的位姿伺服倒进去 (不写死距离/时间)。
    #   注意要用**发出去的名字** (带序号前缀, 如 17_ring) —— patrol.py 是按名字找点的
    ring_it = next((it for it in items
                    if it['kind'] == 'waypoint' and it['name'] == 'ring'), None)
    ring_name = ('%02d_%s' % (ring_it['order'], ring_it['name'])) if ring_it else None
    out = dict(
        note='最终巡检路线 = 原巡检环线 + 沿途识别点; 由 tools/gen_recognition_route.py 生成',
        usage='python3 tools/patrol.py --file src/competition_robot/config/recognition_route.yaml',
        start=list(start),
        return_to=list(ret),
        waypoints=[])
    for it in items:
        w = dict(name='%02d_%s' % (it['order'], it['name']),
                 world=[round(it['x'], 4), round(it['y'], 4)])
        if it['yaw'] is not None:
            w['yaw'] = round(it['yaw'], 4)
            w['task'] = it['task']
            w['shot'] = dict(metric=it['metric']['unit'],
                             value=it['metric']['value'],
                             threshold=it['metric']['threshold'])
        else:
            w['task'] = 'waypoint'
        out['waypoints'].append(w)
    if ring_name:
        # 库位 = 出生点那个角落 (三个停车位被车占着, 沿用旧 waypoints.yaml 的做法)
        out['reverse_park'] = {
            'from': ring_name,                     # ★ 'from' 是关键字, 只能这样写
            'to': [round(start[0], 4), round(start[1], 4), 3.1416],
            'note': '航点跑完后倒车入库: from=起倒点(航点名), to=[x,y,yaw]=库位中心位姿',
        }
    with open(a.out_yaml, 'w') as f:
        f.write('# 最终巡检路线 (唯一真值源) —— 由 tools/gen_recognition_route.py 生成\n'
                '# 顺序 = 沿巡检环线的弧长; recog 点带 yaw = 到点后原地转向拍照的朝向\n'
                '# 跑法: python3 tools/patrol.py --file src/competition_robot/config/recognition_route.yaml\n'
                '#      (航点跑完会自动接倒车入库; 不想倒车加 --no-park)\n')
        yaml.safe_dump(out, f, allow_unicode=True, sort_keys=False, default_flow_style=False)

    # ---- 3) 画图 (地图在上, 图例在下, 标签自动避让) ----
    MAP = 1280
    LEG = 150
    base = Image.open(SRC).convert('RGB')
    img = Image.new('RGB', (MAP, MAP + LEG), (18, 18, 18))
    img.paste(base, (0, 0))
    d = ImageDraw.Draw(img, 'RGBA')
    f_mid, f_sm = font(21, True), font(17, True)
    _, CJK = _load_font(17)
    if not CJK:
        print('  · 这台机器没有中文字体, 图上的说明改用英文 (装 fonts-noto-cjk 可恢复中文)')

    def P(x, y):
        return (x2px(x), y2py(y))

    # 巡检环线 (橙) + 实测轨迹 (红)
    d.line([P(x, y) for x, y in poly], fill=C_LOOP + (150,), width=4, joint='curve')
    if not a.no_trace and os.path.exists(a.trace):
        pts = []
        with open(a.trace) as fh:
            for r in csv.DictReader(fh):
                pts.append((x2px(float(r['x'])), y2py(float(r['y']))))
        if len(pts) > 1:
            d.line(pts, fill=C_TRACE + (200,), width=3, joint='curve')
    # 行进方向箭头 (沿环线)
    L = 0.0
    segs = []
    for i in range(len(poly) - 1):
        segs.append((L, poly[i], poly[i + 1]))
        L += math.hypot(poly[i + 1][0] - poly[i][0], poly[i + 1][1] - poly[i][1])
    for frac in (0.10, 0.35, 0.60, 0.85):
        want = frac * L
        for s0, pa, pb in segs:
            ln = math.hypot(pb[0] - pa[0], pb[1] - pa[1])
            if s0 <= want <= s0 + ln:
                t = (want - s0) / max(ln, 1e-9)
                mx, my = pa[0] + t * (pb[0] - pa[0]), pa[1] + t * (pb[1] - pa[1])
                ang = math.atan2(pb[1] - pa[1], pb[0] - pa[0])
                for da in (2.6, -2.6):
                    d.line([P(mx, my), P(mx + 0.20 * math.cos(ang + da),
                                       my + 0.20 * math.sin(ang + da))],
                           fill=C_LOOP + (230,), width=4)
                break
    # 识别点 -> 环线 的斜线
    for it in items:
        if it['kind'] != 'recog' or it['off'] < 0.03:
            continue
        d.line([P(it['x'], it['y']), P(it['q'][0], it['q'][1])],
               fill=C_TASK[it['task']] + (140,), width=2)

    # 已占用的标签框 (自动避让)
    placed = []

    def put_label(px, py, txt, col, big=False):
        f = f_mid if big else f_sm
        tw = text_w(d, txt, f)
        th = 22 if big else 19
        for dx, dy in ((16, -th - 4), (16, 8), (-tw - 16, -th - 4), (-tw - 16, 8),
                       (16, -th // 2), (-tw - 16, -th // 2)):
            x0, y0 = px + dx, py + dy
            if x0 < 4 or y0 < 4 or x0 + tw > MAP - 4 or y0 + th > MAP - 4:
                continue
            box = (x0 - 3, y0 - 2, x0 + tw + 3, y0 + th + 2)
            if any(not (box[2] < b[0] or box[0] > b[2] or box[3] < b[1] or box[1] > b[3])
                   for b in placed):
                continue
            placed.append(box)
            d.rectangle(box, fill=(0, 0, 0, 200))
            d.text((x0, y0), txt, fill=col, font=f)
            return
        placed.append((px, py, px + tw, py + th))

    # 起点
    sx, sy = P(*start)
    d.ellipse([sx - 13, sy - 13, sx + 13, sy + 13], fill=C_START + (255,))
    put_label(sx, sy, 'START', C_START, big=True)
    # 各点
    for it in items:
        px, py = P(it['x'], it['y'])
        if it['kind'] == 'recog':
            col = C_TASK[it['task']]
            d.ellipse([px - 12, py - 12, px + 12, py + 12], fill=col + (255,),
                      outline=(0, 0, 0, 255), width=2)
            L2 = 40
            d.line([(px, py), (px + L2 * math.cos(-it['yaw']), py + L2 * math.sin(-it['yaw']))],
                   fill=col + (255,), width=3)
            put_label(px, py, '%d %s' % (it['order'], _pick(SHORT[it['task']],
                                                            SHORT_EN[it['task']], CJK)),
                      col, big=True)
        else:
            col = C_WP
            d.ellipse([px - 10, py - 10, px + 10, py + 10], outline=col + (255,), width=4)
            put_label(px, py, '%d %s' % (it['order'], it['name']), col)

    # 图例 (地图下方, 不挡地图)
    d.rectangle([0, MAP, MAP, MAP + LEG], fill=(18, 18, 18))
    def L(zh, en):
        return _pick(zh, en, CJK)
    lines = [
        [(L('红线 = 实测巡检轨迹', 'red = actual driven path'), C_TRACE),
         (L('   橙线 = 巡检环线 (箭头=行进方向)', '   orange = planned loop (arrows = direction)'), C_LOOP),
         (L('   蓝点 = 起点', '   blue = start'), C_START),
         (L('   绿圈 = 原航点', '   green ring = original waypoint'), C_WP)],
        [(L('黄 = 红绿灯', 'yellow = traffic light'), C_TASK['traffic_light']),
         (L('   青 = 人偶立牌', '   cyan = standee'), C_TASK['standee']),
         (L('   橙 = 车牌', '   orange = plate'), C_TASK['plate']),
         (L('   数字 = 最终路线顺序; 粗短线 = 到点后原地转向的拍照朝向',
            '   number = route order; short bar = shooting heading'), (220, 220, 220))],
        [('run: python3 tools/patrol.py --file src/competition_robot/config/recognition_route.yaml',
          (170, 170, 170))],
    ]
    for r, row in enumerate(lines):
        x = 16
        for txt, col in row:
            d.text((x, MAP + 18 + 34 * r), txt, fill=col, font=f_sm)
            x += text_w(d, txt, f_sm) + 6
    img.save(a.out_png)

    print('最终路线 (%d 站)' % len(items))
    for it in items:
        tag = '航点' if it['kind'] == 'waypoint' else TASK_CN[it['task']]
        extra = ''
        if it['kind'] == 'recog':
            extra = '  %s=%.0f   离环线 %.2f m' % (it['metric']['unit'].split()[0],
                                                  it['metric']['value'], it['off'])
        print('  %2d) %-18s %-6s (%+.3f, %+.3f)%s'
              % (it['order'], it['name'], tag, it['x'], it['y'], extra))
    print()
    print('写出:')
    for p in (a.out_yaml, a.out_png):
        print('   ', os.path.relpath(p, WS))


if __name__ == '__main__':
    main()
