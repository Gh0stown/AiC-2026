#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""视觉识别点位计算 (复赛三个视觉任务) —— 全部由真值源算出来, 不手填。

复赛要求的三件事都是"开到固定点位 → 原地转向 → 拍照":
    1) 红绿灯状态识别   (2 盏)
    2) 人偶立牌检测     (A/B 街区共 10 个)
    3) 车牌字符识别     (3 辆)

这个脚本回答两个问题:
    * 车应该停在哪、朝哪 (位姿)
    * 那个位姿下目标在相机里占多少像素 (够不够识别)

真值源 (全部读现有配置, 不复制常量):
    道具位姿   src/competition_arena/worlds/competition_arena.world
    人偶尺寸   src/competition_arena/config/standees.yaml + tools/gen_standees.py
    车辆/车牌  src/competition_arena/config/cars.yaml     + tools/setup_cars.py
    红绿灯     src/competition_arena/config/traffic_lights.yaml + models/traffic_light_h/model.sdf
    相机/底盘  src/competition_robot/config/robot_params.yaml
    车道线     src/competition_arena/tools/map_source.jpg (官方平面图, 程序提取)
               -> 缓存 src/competition_arena/config/lane_lines.json

硬约束 (不满足直接淘汰):
    ① 车体(含余量)不出白线        |x|,|y| <= 2.075
    ② 车体不压车道线 / 停止线      压线扣分; 红灯越过停止线违规
    ③ 车体不碰道具                人偶 / 车 / 红绿灯腿
    ④ 目标整个进画面, 留 8% 边距
    ⑤ 像素指标达阈值

用法:
    python3 tools/gen_recognition_points.py                  # 计算 + 写配置/报告/图
    python3 tools/gen_recognition_points.py --top 8          # 每个目标多看几个备选
    python3 tools/gen_recognition_points.py --refresh-lanes  # 重新从平面图提车道线
"""
from __future__ import annotations

import argparse
import json
import math
import os
import re
import struct
import sys
import zlib

import numpy as np
import yaml

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ARENA = os.path.join(WS, 'src', 'competition_arena')
ROBOT = os.path.join(WS, 'src', 'competition_robot')
WORLD = os.path.join(ARENA, 'worlds', 'competition_arena.world')
LANE_CACHE = os.path.join(ARENA, 'config', 'lane_lines.json')

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import setup_cars as SC          # noqa: E402  车辆/车牌几何唯一真值源
import gen_standees as GS        # noqa: E402  人偶立牌几何唯一真值源

FIELD_LINE = 2.075       # 白线内沿 (车不过这条线)
ROT_R = 0.2582           # 原地转向时车心到墙的最小距离, 取自 tools/patrol.py 的
                         # FOOTPRINT_R —— 点位必须满足它, 否则执行时"转不开"
                         # (转向时车体外接圆扫过: 2.076 - 0.2582 = 1.818)
FOOT_MARGIN = 0.02       # 车体安全余量
PROP_MARGIN = 0.02       # 道具碰撞余量
FRAME_MARGIN = 0.88      # 目标点须落在半视场的这个比例内 (硬约束)
FRAME_MIN = 0.10         # 还要求画面留出 >=10% 余量 (抗定位误差)
D_MIN, D_MAX = 0.30, 4.00
# 搜索网格: 以目标为中心撒一圈候选相机位姿.
# ★ 别调粗: 人偶组受"相机 0.20m 无俯仰"限制, 可行域很窄 (A_west 在 2°/4cm 网格下
#   会被整个漏掉 -> 误报"无可行解"); 1°/2cm 才够稳。
SEARCH_DTHETA = math.radians(1.0)
SEARCH_DD = 0.02
COS_MIN = 0.25           # 立牌/车牌必须从**正面**看: 入射角 <= 75.5°
VIS_MIN = 0.040          # 正对拍立牌时, 画面里至少要看得见这么高的立牌 (m)
                         # 相机 0.20m 无俯仰 -> 正对必然切掉脚, 但上身够识别
                         # (板上的图只贴在正面; 背面是镜像/空白, 不能当识别样本)
STANDEE_NAME = re.compile(r'^(A|B)_(north|south|east|west)_\d+$')
# 每个任务的拍摄距离窗口: 太近会撞/畸变, 太远像素不够
STACK_WINDOW = {'traffic_light': (0.60, 1.60),
                'standee': (0.45, 1.60),
                'plate': (0.55, 0.95)}


# =============================================================================
#  1. 真值源
# =============================================================================
def load_arena_transform():
    geo = json.load(open(os.path.join(ARENA, 'tools', 'arena_geometry.json')))
    s = geo['scale_m_per_px']
    x0, y0 = geo['arena_px'][0], geo['arena_px'][1]
    half = geo['arena_m'] / 2.0
    return dict(scale=s, x0=x0, y0=y0, half=half,
                px2x=lambda px: (px - x0) * s - half,
                px2y=lambda py: half - (py - y0) * s)


def load_robot():
    p = yaml.safe_load(open(os.path.join(ROBOT, 'config', 'robot_params.yaml')))
    cam = p['sensors']['camera']
    ch = p['chassis']
    W, H = float(cam['width']), float(cam['height'])
    hfov = math.radians(float(cam['horizontal_fov_deg']))
    fx = (W / 2.0) / math.tan(hfov / 2.0)
    vfov_half = math.atan((H / 2.0) / fx)
    cox, coy = float(ch['center_offset'][0]), float(ch['center_offset'][1])
    return dict(W=W, H=H, fx=fx,
                half_w_tan=math.tan(hfov / 2.0), half_h_tan=math.tan(vfov_half),
                hfov_deg=math.degrees(hfov), vfov_deg=2 * math.degrees(vfov_half),
                mount=(float(cam['mount'][0]), float(cam['mount'][1]),
                       float(cam['mount'][2])),
                half_x=float(ch['length']) / 2.0 + abs(cox) + FOOT_MARGIN,
                half_y=float(ch['width']) / 2.0 + abs(coy) + FOOT_MARGIN,
                length=float(ch['length']), width=float(ch['width']))


INC_RE = re.compile(
    r'<include>\s*<uri>model://([^<]+)</uri>\s*<name>([^<]+)</name>\s*<pose>([^<]*)</pose>',
    re.S)


def load_world_props():
    txt = open(WORLD).read()
    out = []
    for uri, name, pose in INC_RE.findall(txt):
        v = [float(x) for x in pose.split()]
        out.append(dict(uri=uri, name=name, x=v[0], y=v[1], z=v[2],
                        yaw=(v[5] if len(v) > 5 else 0.0)))
    return out


def load_standees():
    cfg = yaml.safe_load(open(os.path.join(ARENA, 'config', 'standees.yaml')))
    return {s['model']: float(s['width_m']) for s in cfg['standees']}


def load_cars():
    cfg = yaml.safe_load(open(os.path.join(ARENA, 'config', 'cars.yaml')))
    return {c['model']: c for c in cfg['cars']}


def load_lights():
    cfg = yaml.safe_load(open(os.path.join(ARENA, 'config', 'traffic_lights.yaml')))
    return {l['name']: l for l in cfg['lights']}


def load_blocks():
    """街区(非道路)矩形 —— standees.yaml 是唯一真值源.

    街区在仿真里没有实体墙, 只有地面画线, 所以必须显式禁行, 否则车会开进街区里。
    """
    cfg = yaml.safe_load(open(os.path.join(ARENA, 'config', 'standees.yaml')))
    out = []
    for b in cfg.get('blocks', []):
        x0, y0, x1, y1 = b['rect']
        out.append((b['name'], (x0 + x1) / 2.0, (y0 + y1) / 2.0,
                    abs(x1 - x0) / 2.0, abs(y1 - y0) / 2.0))
    return out


# =============================================================================
#  2. 车道线 / 停止线 (从官方平面图提取)
# =============================================================================
def extract_lane_lines(refresh=False):
    """轴对齐线段: {kind:'v'|'h', c, a, b, w, type}

    type: lane 车道线 / stop 停止线 / parking 车位分隔线
    短于 0.30 m 的按斑马线条纹忽略 —— 斑马线是给人走的, 车正常通过。
    """
    if os.path.exists(LANE_CACHE) and not refresh:
        d = json.load(open(LANE_CACHE))
        return d['segments'], d['meta']

    from PIL import Image
    tf = load_arena_transform()
    px2x, px2y = tf['px2x'], tf['px2y']
    im = np.array(Image.open(os.path.join(ARENA, 'tools', 'map_source.jpg')).convert('L'))
    w = im > 180

    def runs(vec, gap=3, minlen=23):
        idx = np.where(vec)[0]
        out = []
        if len(idx) == 0:
            return out
        s = p = idx[0]
        for i in list(idx[1:]) + [10 ** 9]:
            if i > p + gap:
                if p - s >= minlen:
                    out.append((s, p))
                s = i
            p = i
        return out

    def merge(segs):
        segs.sort()
        merged = []
        for c, s, p in segs:
            for m in merged:
                if abs(m['c1'] - c) <= 1 and not (p < m['s'] - 5 or s > m['p'] + 5):
                    m['c1'] = c
                    m['s'] = min(m['s'], s)
                    m['p'] = max(m['p'], p)
                    m['n'] += 1
                    break
            else:
                merged.append(dict(c0=c, c1=c, s=s, p=p, n=1))
        return merged

    n_px = int(round(tf['half'] * 2 / tf['scale']))
    vraw = [(px, s, p) for px in range(tf['x0'], tf['x0'] + n_px)
            for s, p in runs(w[:, px])]
    hraw = [(py, s, p) for py in range(tf['y0'], tf['y0'] + n_px)
            for s, p in runs(w[py, :])]

    segments = []
    for m in merge(vraw):
        if m['n'] < 2:
            continue
        c = px2x((m['c0'] + m['c1']) / 2.0)
        a, b = px2y(m['p']), px2y(m['s'])
        width = (m['c1'] - m['c0'] + 1) * tf['scale']
        if abs(abs(c) - 2.086) < 0.02 or (b - a) < 0.30:
            continue
        segments.append(dict(kind='v', c=round(c, 4), a=round(a, 4), b=round(b, 4),
                             w=width,
                             type='stop' if width >= 0.030 else
                                  ('parking' if c > 1.4 else 'lane')))
    for m in merge(hraw):
        if m['n'] < 2:
            continue
        c = px2y((m['c0'] + m['c1']) / 2.0)
        a, b = px2x(m['s']), px2x(m['p'])
        width = (m['c1'] - m['c0'] + 1) * tf['scale']
        if abs(abs(c) - 2.086) < 0.02 or (b - a) < 0.30:
            continue
        segments.append(dict(kind='h', c=round(c, 4), a=round(a, 4), b=round(b, 4),
                             w=width,
                             type='stop' if width >= 0.030 else
                                  ('parking' if a > 1.4 else 'lane')))
    segments.sort(key=lambda s: (s['type'], s['kind'], s['c']))
    meta = dict(field_line=FIELD_LINE,
                note='从 map_source.jpg 提取; <0.30m 视为斑马线条纹忽略; '
                     '线宽>=30mm 判为停止线')
    os.makedirs(os.path.dirname(LANE_CACHE), exist_ok=True)
    json.dump(dict(segments=segments, meta=meta), open(LANE_CACHE, 'w'),
              ensure_ascii=False, indent=1)
    return segments, meta


# =============================================================================
#  3. 几何 (全部对位姿数组向量化)
# =============================================================================
# 路网走廊 = 一组矩形 (x0,y0,x1,y1). 其余是街区/楼宇, 车不许进。
# 依据:
#   * 平面图程序提取的车道线 (config/lane_lines.json) —— 走廊都是 0.605~0.635 m 宽
#   * 官方复赛任务示意图的布局 —— 中右列 (x 0.21~0.83) 是**楼宇A/B**,
#     下方带 (y -1.46~-0.53) 是**楼宇D/站房**, 都不是路
#   * 内部走廊是有端的: 竖车道只从底车道通到中车道; 中车道只从左车道通到竖车道;
#     东竖车道从底车道通到顶车道
ROADS = [
    (-2.075, -2.075,  2.075, -1.460, '底车道'),
    (-2.075,  1.470,  2.075,  2.075, '顶车道'),
    (-2.075, -2.075, -1.466,  2.075, '左车道'),
    ( 1.458, -2.075,  2.075,  2.075, '右车道/停车位'),
    (-0.421, -1.460,  0.206,  0.848, '竖车道'),
    ( 0.833, -1.460,  1.458,  1.470, '东竖车道'),
    (-1.466,  0.213,  0.206,  0.848, '中车道'),
]


def rect_corners(cx, cy, yaw, hx, hy):
    c, s = np.cos(yaw), np.sin(yaw)
    lx = np.array([+hx, -hx, -hx, +hx])
    ly = np.array([+hy, +hy, -hy, -hy])
    X = cx[:, None] + lx[None, :] * c[:, None] - ly[None, :] * s[:, None]
    Y = cy[:, None] + lx[None, :] * s[:, None] + ly[None, :] * c[:, None]
    return np.stack([X, Y], axis=-1)


def rect_samples(cx, cy, yaw, hx, hy):
    """车体上 9 个采样点 (4 角 + 4 边中点 + 中心), 用于"整块都在路网里"判定"""
    c, s = np.cos(yaw), np.sin(yaw)
    lx = np.array([+hx, -hx, -hx, +hx, 0.0, +hx, -hx, 0.0, 0.0])
    ly = np.array([+hy, +hy, -hy, -hy, 0.0, 0.0, 0.0, +hy, -hy])
    X = cx[:, None] + lx[None, :] * c[:, None] - ly[None, :] * s[:, None]
    Y = cy[:, None] + lx[None, :] * s[:, None] + ly[None, :] * c[:, None]
    return np.stack([X, Y], axis=-1)


def in_road(P):
    """P: (N,K,2) -> (N,) 每个采样点都必须落在**某一个路网矩形**里"""
    x, y = P[..., 0], P[..., 1]
    ok = np.zeros(x.shape, dtype=bool)
    for (x0, y0, x1, y1, _n) in ROADS:
        ok |= (x >= x0) & (x <= x1) & (y >= y0) & (y <= y1)
    return ok.all(axis=1)


def obb_hit(ax, ay, ayaw, ahx, ahy, bx, by, byaw, bhx, bhy):
    """分离轴定理, 输入 (N,), 返回 (N,) bool"""
    dx, dy = bx - ax, by - ay
    ca, sa = np.cos(ayaw), np.sin(ayaw)
    cb, sb = np.cos(byaw), np.sin(byaw)
    hit = np.ones(len(ax), dtype=bool)
    for ux, uy in ((ca, sa), (-sa, ca), (cb, sb), (-sb, cb)):
        pd = np.abs(dx * ux + dy * uy)
        ra = ahx * np.abs(ux * ca + uy * sa) + ahy * np.abs(-ux * sa + uy * ca)
        rb = bhx * np.abs(ux * cb + uy * sb) + bhy * np.abs(-ux * sb + uy * cb)
        hit &= pd <= ra + rb
    return hit


def seg_hit_rect(seg, cx, cy, yaw, hx, hy):
    """轴对齐线段 vs 旋转车体矩形 (线段按线宽+余量加粗)"""
    hw = seg['w'] / 2.0 + FOOT_MARGIN
    if seg['kind'] == 'v':
        sx, sy = seg['c'], (seg['a'] + seg['b']) / 2.0
        shx, shy = hw, (seg['b'] - seg['a']) / 2.0 + FOOT_MARGIN
    else:
        sx, sy = (seg['a'] + seg['b']) / 2.0, seg['c']
        shx, shy = (seg['b'] - seg['a']) / 2.0 + FOOT_MARGIN, hw
    n = len(cx)
    return obb_hit(cx, cy, yaw, hx, hy,
                   np.full(n, sx), np.full(n, sy), np.zeros(n),
                   np.full(n, shx), np.full(n, shy))


# =============================================================================
#  4. 目标
# =============================================================================
def standee_box(x, y, yaw, w):
    hx, hy = GS.THICK / 2.0, w / 2.0
    z0, z1 = GS.BASE_H, GS.BASE_H + GS.STANDEE_H
    c, s = math.cos(yaw), math.sin(yaw)
    pts = []
    for dx in (+hx, -hx):
        for dy in (+hy, -hy):
            for z in (z0, z1):
                pts.append([x + dx * c - dy * s, y + dx * s + dy * c, z])
    return dict(x=x, y=y, w=w, normal=(c, s), z0=z0, z1=z1,
                foot=(GS.BASE_X / 2.0, w / 2.0), pts=pts)


def light_geom(lt):
    """红绿灯: 灯箱角点 + 3 灯珠 + 2 条腿 (局部 -> 世界)"""
    c, s = math.cos(lt['yaw']), math.sin(lt['yaw'])

    def W(lx, ly, lz=0.0):
        return [lt['x'] + lx * c - ly * s, lt['y'] + lx * s + ly * c, lz]

    lamps = []
    for dy, col in ((-0.220, 'red'), (0.0, 'yellow'), (0.220, 'green')):
        lamps.append(dict(color=col, p=W(0.0500 + 0.004, dy, 0.410)))
    pts = []
    for dx in (+0.040, -0.040):
        for dy in (+0.320, -0.320):
            for dz in (+0.070, -0.070):
                pts.append(W(dx, dy, 0.410 + dz))
    for dy in (-0.220, 0.0, 0.220):            # 灯珠盘边沿 (会伸到 0.0555)
        for dx in (+0.0555,):
            pts.append(W(dx, dy + 0.0425, 0.410))
            pts.append(W(dx, dy - 0.0425, 0.410))
    legs = [(W(0.0, dy)[0], W(0.0, dy)[1], lt['yaw'], 0.070, 0.025)
            for dy in (-0.307, +0.307)]
    return dict(x=lt['x'], y=lt['y'], yaw=lt['yaw'], lamps=lamps, pts=pts, legs=legs)


def build_targets(standee_w, cars, lights):
    T = []
    for name, lt in lights.items():
        g = light_geom(lt)
        T.append(dict(name=name, task='traffic_light', kind='light', geom=g,
                      pts=g['pts'], center=(g['x'], g['y']),
                      thr=25.0, unit='灯珠直径 px'))
    groups = {}
    for p in load_world_props():
        m = STANDEE_NAME.match(p['name'])
        if m:
            groups.setdefault('%s_%s' % (m.group(1), m.group(2)), []).append(p)
    for gname, ps in sorted(groups.items()):
        boxes = [standee_box(p['x'], p['y'], p['yaw'], standee_w[p['uri']]) for p in ps]
        T.append(dict(name=gname, task='standee', kind='standee', geom=boxes,
                      pts=[q for b in boxes for q in b['pts']],
                      center=(float(np.mean([p['x'] for p in ps])),
                              float(np.mean([p['y'] for p in ps]))),
                      thr=40.0, unit='立牌宽 px'))
    for model, c in sorted(cars.items()):
        pcx = c['x'] - (SC.BOARD_T / 2.0 + SC.PLATE_T / 2.0)   # 车牌贴片中心 x
        face = pcx - SC.PLATE_T / 2.0                          # 朝车道那一面
        pz = SC.BASE_T + SC.PLATE_CZ
        pw, ph = SC.PLATE_W, SC.PLATE_H
        pts = [[face, c['y'] + dy, pz + dz]
               for dy in (+pw / 2.0, -pw / 2.0) for dz in (+ph / 2.0, -ph / 2.0)]
        T.append(dict(name=model, task='plate', kind='plate',
                      geom=dict(face=face, y=c['y'], w=pw, h=ph,
                                normal=(-1.0, 0.0), plate=c['plate']),
                      pts=pts, center=(face, c['y']),
                      thr=140.0, unit='车牌宽 px'))
    return T


def all_prop_rects(standee_w, cars, lights):
    out = []
    for p in load_world_props():
        if p['name'] in lights:
            for r in light_geom(lights[p['name']])['legs']:
                out.append(r)
        elif STANDEE_NAME.match(p['name']):
            out.append((p['x'], p['y'], p['yaw'],
                        GS.BASE_X / 2.0, standee_w[p['uri']] / 2.0))
        elif p['name'].startswith('car_'):
            out.append((p['x'], p['y'], p['yaw'], SC.BASE_T / 2.0, SC.BOARD_W / 2.0))
    return out


# =============================================================================
#  5. 评估 / 搜索
# =============================================================================
def camera_xy(cx, cy, yaw, rob):
    c, s = np.cos(yaw), np.sin(yaw)
    mx, my = rob['mount'][0], rob['mount'][1]
    return cx + mx * c - my * s, cy + mx * s + my * c


def wrap(a):
    return (np.asarray(a) + math.pi) % (2 * math.pi) - math.pi


def evaluate(target, cx, cy, yaw, rob):
    pts = np.array(target['pts'], dtype=float)
    camx, camy = camera_xy(cx, cy, yaw, rob)
    camz = rob['mount'][2]
    fwd = np.stack([np.cos(yaw), np.sin(yaw)], axis=-1)
    left = np.stack([-np.sin(yaw), np.cos(yaw)], axis=-1)
    d = np.stack([pts[None, :, 0] - camx[:, None],
                  pts[None, :, 1] - camy[:, None]], axis=-1)
    depth = (d * fwd[:, None, :]).sum(-1)
    lateral = (d * left[:, None, :]).sum(-1)
    vert = pts[None, :, 2] - camz
    sd = np.maximum(depth, 1e-9)
    hw, hh = sd * rob['half_w_tan'], sd * rob['half_h_tan']
    inside = ((depth > D_MIN) & (depth < D_MAX)
              & (np.abs(lateral) <= FRAME_MARGIN * hw)
              & (np.abs(vert) <= FRAME_MARGIN * hh)).all(axis=1)
    frame = np.minimum((hw - np.abs(lateral)) / np.maximum(hw, 1e-9),
                       (hh - np.abs(vert)) / np.maximum(hh, 1e-9)).min(axis=1)

    if target['kind'] == 'standee':
        vals, cos = [], []
        for b in target['geom']:
            dd = np.hypot(b['x'] - camx, b['y'] - camy)
            cosi = ((camx - b['x']) * b['normal'][0]
                    + (camy - b['y']) * b['normal'][1]) / np.maximum(dd, 1e-9)   # 有符号
            cos.append(cosi)
            vals.append(b['w'] * np.abs(cosi) * rob['fx'] / np.maximum(dd, 1e-9))
        metric = np.min(np.stack(vals), axis=0)
        cosi_min = np.min(np.stack(cos), axis=0)
        hgt = np.min(np.stack([(b['z1'] - b['z0']) * rob['fx']
                               / np.maximum(np.hypot(b['x'] - camx, b['y'] - camy), 1e-9)
                               for b in target['geom']]), axis=0)
    elif target['kind'] == 'light':
        vals, cos = [], []
        n = (math.cos(target['geom']['yaw']), math.sin(target['geom']['yaw']))
        for l in target['geom']['lamps']:
            dx, dy = l['p'][0] - camx, l['p'][1] - camy
            dd = np.hypot(dx, dy)
            vals.append(0.085 * rob['fx'] / np.maximum(dd, 1e-9))
            cos.append(-(dx * n[0] + dy * n[1]) / np.maximum(dd, 1e-9))
        metric = np.min(np.stack(vals), axis=0)
        cosi_min = np.min(np.stack(cos), axis=0)
        hgt = np.zeros(len(camx))
    else:
        g = target['geom']
        dd = np.hypot(g['face'] - camx, g['y'] - camy)
        cosi = ((camx - g['face']) * g['normal'][0]
                + (camy - g['y']) * g['normal'][1]) / np.maximum(dd, 1e-9)
        metric = g['w'] * np.abs(cosi) * rob['fx'] / np.maximum(dd, 1e-9)
        cosi_min = cosi
        hgt = g['h'] * rob['fx'] / np.maximum(dd, 1e-9)

    dist = np.hypot(target['center'][0] - camx, target['center'][1] - camy)
    return dict(inside=inside, metric=metric, metric_h=hgt, frame=frame, dist=dist,
                cosi=cosi_min, camx=camx, camy=camy)


def feasible(cx, cy, yaw, rob, segments, props, blocks):
    ok = np.ones(len(cx), dtype=bool)
    C = rect_corners(cx, cy, yaw, rob['half_x'], rob['half_y'])
    ok &= (np.abs(C[..., 0]) <= FIELD_LINE).all(1) & \
          (np.abs(C[..., 1]) <= FIELD_LINE).all(1)
    # ①b 原地转向余量 (和 patrol.py 同一判据): 车心离墙 >= 车体外接圆半径
    ok &= (np.maximum(np.abs(cx), np.abs(cy)) <= FIELD_LINE - ROT_R)
    # ② 只能在路网走廊上 (街区之间没有实体墙, 穿街区=压线/撞人偶)
    ok &= in_road(rect_samples(cx, cy, yaw, rob['half_x'], rob['half_y']))
    if not ok.any():
        return ok
    # ③ 不压车道线 / 停止线
    for seg in segments:
        if not ok.any():
            return ok
        i = np.where(ok)[0]
        ok[i] &= ~seg_hit_rect(seg, cx[i], cy[i], yaw[i], rob['half_x'], rob['half_y'])
    # ④ 不碰道具
    for (px, py, pyaw, phx, phy) in props:
        if not ok.any():
            return ok
        i = np.where(ok)[0]
        n = len(i)
        ok[i] &= ~obb_hit(cx[i], cy[i], yaw[i], rob['half_x'], rob['half_y'],
                          np.full(n, px), np.full(n, py), np.full(n, pyaw),
                          np.full(n, phx + PROP_MARGIN), np.full(n, phy + PROP_MARGIN))
    # ⑤ 不进街区 (街区矩形来自 standees.yaml)
    for (_bn, px, py, phx, phy) in blocks:
        if not ok.any():
            return ok
        i = np.where(ok)[0]
        n = len(i)
        ok[i] &= ~obb_hit(cx[i], cy[i], yaw[i], rob['half_x'], rob['half_y'],
                          np.full(n, px), np.full(n, py), np.zeros(n),
                          np.full(n, phx + PROP_MARGIN), np.full(n, phy + PROP_MARGIN))
    return ok


def center_yaw(target, cx, cy, yaw, rob, iters=3):
    """把目标的角展度摆到画面正中 (最小化最大偏角)"""
    pts = np.array(target['pts'], dtype=float)
    for _ in range(iters):
        camx, camy = camera_xy(cx, cy, yaw, rob)
        ang = np.arctan2(pts[None, :, 1] - camy[:, None],
                         pts[None, :, 0] - camx[:, None])
        rel = np.angle(np.exp(1j * (ang - yaw[:, None])))
        yaw = yaw + (rel.min(axis=1) + rel.max(axis=1)) / 2.0
    return yaw


def search_face_on(target, rob, segments, props, blocks, topn=3):
    """**正对**该方向人偶的点位 (用户定的原则).

    相机光轴垂直于立牌板面: 车停在立牌正前方的法线上, 朝向 = 背对法线。
    正对时没有透视缩短, 立牌宽度就是真实宽度。

    因为相机只有 0.20 m 高且无俯仰, 正对到最近只能站 ~0.23 m (再近相机就贴上了,
    再远会被车道/白线/转向余量挡住), 所以**下沿一定被切** —— 判据只要求
    上沿 + 左右入画, 并用"可见高度"卡一个下限 (VIS_MIN)。
    """
    n = np.array(target['geom'][0]['normal'], dtype=float)
    c = np.array(target['center'], dtype=float)
    yaw0 = math.atan2(-n[1], -n[0])
    fwd = np.array([math.cos(yaw0), math.sin(yaw0)])
    perp = np.array([-n[1], n[0]])
    ztop = max(q[2] for b in target['geom'] for q in b['pts'])
    zbot = min(q[2] for b in target['geom'] for q in b['pts'])

    ts, ds = np.meshgrid(np.arange(-0.15, 0.151, 0.01), np.arange(0.10, 1.201, 0.005))
    ts, ds = ts.ravel(), ds.ravel()
    camx = c[0] + ds * n[0] + ts * perp[0]
    camy = c[1] + ds * n[1] + ts * perp[1]
    rx = camx - rob['mount'][0] * fwd[0]
    ry = camy - rob['mount'][0] * fwd[1]
    yaw = np.full(len(ds), yaw0)

    ok = feasible(rx, ry, yaw, rob, segments, props, blocks)
    if not ok.any():
        return []
    frame = np.full(len(ds), 9.0)
    dmin = np.full(len(ds), 9.0)
    for b in target['geom']:
        for q in b['pts']:
            if abs(q[2] - ztop) > 1e-9:            # 只查上沿 (横向极值也在上沿角上)
                continue
            vx, vy = q[0] - rx, q[1] - ry
            depth = vx * fwd[0] + vy * fwd[1]
            lat = vx * perp[0] + vy * perp[1]
            vert = q[2] - rob['mount'][2]
            hw = depth * rob['half_w_tan']
            hh = depth * rob['half_h_tan']
            ok &= (depth > 0.05) & (np.abs(lat) <= FRAME_MARGIN * hw) \
                & (np.abs(vert) <= FRAME_MARGIN * hh)
            frame = np.minimum(frame, np.minimum(
                (hw - np.abs(lat)) / np.maximum(hw, 1e-9),
                (hh - np.abs(vert)) / np.maximum(hh, 1e-9)))
            dmin = np.minimum(dmin, depth)
    vis = ztop - np.maximum(zbot, rob['mount'][2] - FRAME_MARGIN * rob['half_h_tan'] * dmin)
    metric = target['geom'][0]['w'] * rob['fx'] / np.maximum(dmin, 1e-9)   # 正对: 不缩水
    ok &= (vis >= VIS_MIN) & (metric >= target['thr'])
    if not ok.any():
        return []

    # 先要"尽量远"(看得见更多身体), 同距离取横向偏置最小的(立牌更正对画面中心)
    dmax = ds[ok].max()
    near = ok & (ds >= dmax - 0.005)
    out, seen = [], []
    for i in np.argsort(np.where(near, np.abs(ts), 9.9)):
        if not near[i]:
            continue
        if any(abs(rx[i] - s0) < 0.05 and abs(ry[i] - s0) < 0.05 for s0 in []):
            continue
        if seen and any(np.hypot(rx[i] - a, ry[i] - b) < 0.10 for a, b in seen):
            continue
        seen.append((rx[i], ry[i]))
        out.append(dict(x=float(rx[i]), y=float(ry[i]), yaw=float(yaw[i]),
                        metric=float(metric[i]), metric_h=float(vis[i] * rob['fx'] / max(dmin[i], 1e-9)),
                        dist=float(ds[i]), frame=float(frame[i]), cosi=1.0,
                        vis=float(vis[i]), lateral=float(ts[i]),
                        score=float(ds[i])))
        if len(out) >= topn:
            break
    return out


def search_target(target, rob, segments, props, blocks, topn=5):
    cxm, cym = target['center']
    cam = []
    for theta in np.arange(0, 2 * math.pi, SEARCH_DTHETA):
        for dist in np.arange(0.34, 2.62, SEARCH_DD):
            cam.append((cxm + dist * math.cos(theta), cym + dist * math.sin(theta), theta))
    cam = np.array(cam)
    yaw = cam[:, 2] + math.pi
    cx = cam[:, 0] - rob['mount'][0] * np.cos(yaw)
    cy = cam[:, 1] - rob['mount'][0] * np.sin(yaw)
    yaw = wrap(center_yaw(target, cx, cy, yaw, rob))

    ok = feasible(cx, cy, yaw, rob, segments, props, blocks)
    if not ok.any():
        return []
    ev = evaluate(target, cx, cy, yaw, rob)
    lo, hi = STACK_WINDOW[target['task']]
    ok &= (ev['inside'] & (ev['metric'] >= target['thr']) & (ev['frame'] >= FRAME_MIN)
           & (ev['cosi'] >= COS_MIN) & (ev['dist'] >= lo) & (ev['dist'] <= hi))
    if not ok.any():
        return []
    cx, cy, yaw = cx[ok], cy[ok], yaw[ok]
    metric, frame, dist, cosi = ev['metric'][ok], ev['frame'][ok], ev['dist'][ok], ev['cosi'][ok]
    metric_h = ev['metric_h'][ok]
    # 打分: 先要像素够用, 再在同等条件下偏好"画面更松快"的位姿 (抗定位误差)
    score = np.minimum(metric / target['thr'], 4.0) + 0.4 * np.minimum(frame, 0.35) / 0.35
    out, seen = [], []
    for i in np.argsort(-score):
        if any(np.hypot(cx[i] - s[0], cy[i] - s[1]) < 0.25
               and abs((yaw[i] - s[2] + math.pi) % (2 * math.pi) - math.pi)
               < math.radians(25) for s in seen):
            continue
        seen.append((cx[i], cy[i], yaw[i]))
        out.append(dict(x=float(cx[i]), y=float(cy[i]), yaw=float(yaw[i]),
                        metric=float(metric[i]), metric_h=float(metric_h[i]),
                        frame=float(frame[i]), dist=float(dist[i]),
                        cosi=float(cosi[i]), score=float(score[i])))
        if len(out) >= topn:
            break
    return out


# =============================================================================
#  6. 车道命名 / 出图 / 主流程
# =============================================================================
X_BANDS = [(-2.075, -1.466, '左车道'), (-1.466, -0.421, 'A/B 街区列'),
           (-0.421, 0.206, '竖车道'), (0.206, 0.833, '中部街区列'),
           (0.833, 1.458, '东竖车道'), (1.458, 2.075, '右车道/停车位')]
Y_BANDS = [(-2.075, -1.460, '底车道'), (-1.460, -0.527, '下方街区带'),
           (-0.527, 0.213, 'B 街区带'), (0.213, 0.848, '中车道'),
           (0.848, 1.470, 'A 街区带'), (1.470, 2.075, '顶车道')]


def _band_name(v, table):
    for lo, hi, name in table:
        if lo - 1e-6 <= v <= hi + 1e-6:
            return name
    return '?'


def lane_name(x, y, segments=None):
    """把 (x,y) 落到哪条走廊 (只用于报告可读性)"""
    return '%s / %s' % (_band_name(x, X_BANDS), _band_name(y, Y_BANDS))


def write_png(path, segments, targets, chosen, rob):
    """俯视图核对: 车道线(灰)/停止线(红)/车位线(浅绿) + 道具 + 点位(车体框+短视锥+目标环)"""
    scale, size = 110.0, 500
    img = np.full((size, size, 3), 255, np.uint8)

    def w2p(x, y):
        return (int(round((x + 2.2) * scale)), int(round((2.2 - y) * scale)))

    def dot(px, py, col, r=1):
        x0, x1 = max(0, px - r), min(size, px + r + 1)
        y0, y1 = max(0, py - r), min(size, py + r + 1)
        if x0 < x1 and y0 < y1:
            img[y0:y1, x0:x1] = col

    def line(x0, y0, x1, y1, col, r=0):
        n = int(max(abs(x1 - x0), abs(y1 - y0))) + 1
        for t in np.linspace(0, 1, n):
            dot(int(round(x0 + (x1 - x0) * t)), int(round(y0 + (y1 - y0) * t)), col, r)

    def ring(cx, cy, rad, col):
        for a in np.linspace(0, 2 * math.pi, 40):
            dot(int(round(cx + rad * math.cos(a))), int(round(cy + rad * math.sin(a))), col)

    for sx in (-FIELD_LINE, FIELD_LINE):                      # 白线边界
        for t in np.linspace(-FIELD_LINE, FIELD_LINE, 700):
            dot(*w2p(sx, t), (185, 185, 185))
            dot(*w2p(t, sx), (185, 185, 185))

    for s in segments:                                        # 车道线/停止线/车位线
        col = {'stop': (230, 40, 40), 'parking': (150, 205, 150),
               'lane': (120, 120, 120)}[s['type']]
        if s['kind'] == 'v':
            line(*w2p(s['c'], s['a']), *w2p(s['c'], s['b']), col)
        else:
            line(*w2p(s['a'], s['c']), *w2p(s['b'], s['c']), col)

    for p in load_world_props():                              # 道具
        if STANDEE_NAME.match(p['name']):
            dot(*w2p(p['x'], p['y']), (0, 140, 0), 2)
        elif p['name'].startswith('car_'):
            dot(*w2p(p['x'], p['y']), (0, 90, 220), 3)
        elif p['name'] in ('tl_top', 'tl_bot'):
            dot(*w2p(p['x'], p['y']), (250, 140, 0), 3)

    task_col = {'traffic_light': (215, 0, 110), 'standee': (0, 150, 60),
                'plate': (0, 70, 225)}
    for t in targets:
        if t['name'] not in chosen:
            continue
        p = chosen[t['name']][0]
        col = task_col[t['task']]
        C = rect_corners(np.array([p['x']]), np.array([p['y']]), np.array([p['yaw']]),
                         rob['half_x'], rob['half_y'])[0]
        for i in range(4):                                    # 车体框
            line(*w2p(*C[i]), *w2p(*C[(i + 1) % 4]), col)
        camx, camy = camera_xy(np.array([p['x']]), np.array([p['y']]),
                               np.array([p['yaw']]), rob)
        cxp, cyp = w2p(float(camx[0]), float(camy[0]))
        dot(cxp, cyp, col, 2)
        L = min(float(p['dist']) + 0.15, 1.4)                 # 短视锥
        for sgn in (+1, -1):
            ang = p['yaw'] + sgn * math.radians(rob['hfov_deg'] / 2.0)
            line(cxp, cyp, *w2p(float(camx[0]) + L * math.cos(ang),
                                float(camy[0]) + L * math.sin(ang)), col)
        ring(*w2p(*t['center'][:2]), 5, col)                  # 目标环

    raw = b''.join(b'\x00' + img[y].tobytes() for y in range(size))

    def chunk(tag, data):
        c = tag + data
        return struct.pack('>I', len(data)) + c + struct.pack('>I', zlib.crc32(c) & 0xffffffff)

    open(path, 'wb').write(
        b'\x89PNG\r\n\x1a\n'
        + chunk(b'IHDR', struct.pack('>IIBBBBB', size, size, 8, 2, 0, 0, 0))
        + chunk(b'IDAT', zlib.compress(raw)) + chunk(b'IEND', b''))


def stop_gap(x, y, yaw, rob, segments):
    """车体外沿到最近停止线的净距 (m). >0 = 停在停止线内侧"""
    c, s = math.cos(yaw), math.sin(yaw)
    ex = rob['half_x'] * abs(c) + rob['half_y'] * abs(s)
    ey = rob['half_x'] * abs(s) + rob['half_y'] * abs(c)
    best = None
    for seg in segments:
        if seg['type'] != 'stop':
            continue
        if seg['kind'] == 'v':
            if not (seg['a'] - 0.3 <= y <= seg['b'] + 0.3):
                continue
            g = abs(x - seg['c']) - ex - seg['w'] / 2.0
        else:
            if not (seg['a'] - 0.3 <= x <= seg['b'] + 0.3):
                continue
            g = abs(y - seg['c']) - ey - seg['w'] / 2.0
        best = g if best is None else min(best, g)
    return best


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--top', type=int, default=5)
    ap.add_argument('--standee-mode', choices=['face', 'oblique'], default='face',
                    help='人偶点位: face=正对该方向人偶(默认, 用户要求的原则); '
                         'oblique=斜视(能拍全但斜)')
    ap.add_argument('--refresh-lanes', action='store_true')
    ap.add_argument('--out-yaml',
                    default=os.path.join(ROBOT, 'config', 'recognition_points.yaml'))
    ap.add_argument('--out-png',
                    default=os.path.join(WS, 'docs', 'recognition_points.png'))
    ap.add_argument('--out-md',
                    default=os.path.join(WS, 'docs', 'recognition_points.md'))
    a = ap.parse_args()

    rob = load_robot()
    segments, _ = extract_lane_lines(a.refresh_lanes)
    standee_w, cars, lights = load_standees(), load_cars(), load_lights()
    props = all_prop_rects(standee_w, cars, lights)
    blocks = load_blocks()
    targets = build_targets(standee_w, cars, lights)

    print('=' * 100)
    print('视觉识别点位计算 (复赛: 红绿灯 / 人偶立牌 / 车牌)')
    print('=' * 100)
    print('相机  %.0fx%.0f px  HFOV %.1f°  VFOV %.1f°  fx=%.1f px  离地 %.3f m  俯仰 0°'
          % (rob['W'], rob['H'], rob['hfov_deg'], rob['vfov_deg'], rob['fx'],
             rob['mount'][2]))
    print('车体  %.3f x %.3f m (半长 %.3f 已含中心偏移+余量)   可行驶 |x|,|y| <= %.3f'
          % (rob['length'], rob['width'], rob['half_x'], FIELD_LINE))
    print('车道线 %d 段 (停止线 %d)   禁行街区 %d 个 (%s)'
          % (len(segments), sum(1 for s in segments if s['type'] == 'stop'),
             len(blocks), '/'.join(b[0] for b in blocks)))
    print('目标 %d 个 (灯 %d / 人偶组 %d / 车牌 %d)   正面约束 cos>=%.2f'
          % (len(targets), sum(1 for t in targets if t['task'] == 'traffic_light'),
             sum(1 for t in targets if t['task'] == 'standee'),
             sum(1 for t in targets if t['task'] == 'plate'), COS_MIN))

    chosen, rows = {}, []
    for t in targets:
        if t['task'] == 'standee' and a.standee_mode == 'face':
            cands = search_face_on(t, rob, segments, props, blocks, max(3, a.top))
        else:
            cands = search_target(t, rob, segments, props, blocks, a.top)
        print('-' * 100)
        print('■ %-9s [%-13s] 目标 (%+.3f, %+.3f)   阈值 %s >= %g'
              % (t['name'], t['task'], t['center'][0], t['center'][1], t['unit'], t['thr']))
        if not cands:
            print('   ✗ 无可行解 (约束太紧)')
            rows.append(dict(target=t['name'], task=t['task'], ok=False))
            continue
        chosen[t['name']] = cands
        for i, c in enumerate(cands):
            h = '' if t['task'] != 'standee' else '  可见高=%.0f px(%.0f%%)' % (
                c['metric_h'], c.get('vis', 0) / 0.150 * 100)
            print('   %s #%d 停 (%+.3f, %+.3f) 朝 %+7.1f°  %s=%6.1f%s  距离 %.3f m  '
                  '画面余量 %3.0f%%  正视度 %.2f  分 %.2f'
                  % ('★' if i == 0 else ' ', i + 1, c['x'], c['y'],
                     math.degrees(c['yaw']), t['unit'], c['metric'], h, c['dist'],
                     c['frame'] * 100, c['cosi'], c['score']))
            if i == 0:
                print('        %s' % lane_name(c['x'], c['y'], segments))
        rows.append(dict(target=t['name'], task=t['task'], ok=True, cands=cands,
                         unit=t['unit'], thr=t['thr']))

    out = dict(
        note='视觉识别点位 (唯一真值源) —— 由 tools/gen_recognition_points.py 计算',
        usage='把车开到 pose 的 (x,y), 原地转到 yaw, 拍照',
        camera=dict(width=int(rob['W']), height=int(rob['H']),
                    hfov_deg=round(rob['hfov_deg'], 1),
                    vfov_deg=round(rob['vfov_deg'], 1),
                    fx_px=round(rob['fx'], 1), height_m=round(rob['mount'][2], 4),
                    pitch_deg=0.0),
        constraints=dict(field_line=FIELD_LINE, no_cross_lane_lines=True,
                         no_prop_collision=True, frame_margin=FRAME_MARGIN),
        points=[])
    for r in rows:
        if not r['ok']:
            continue
        c = r['cands'][0]
        pt = dict(
            name=r['target'], task=r['task'],
            pose=[round(c['x'], 4), round(c['y'], 4), round(c['yaw'], 4)],
            yaw_deg=round(math.degrees(c['yaw']), 1),
            lane=lane_name(c['x'], c['y'], segments),
            shot_distance_m=round(c['dist'], 3),
            metric=dict(unit=r['unit'], value=round(c['metric'], 1), threshold=r['thr'],
                        height_px=round(c['metric_h'], 1) if r['task'] == 'standee' else None),
            view_incidence_cos=round(c['cosi'], 3),
            frame_margin=round(c['frame'], 3),
            alternatives=[[round(q['x'], 4), round(q['y'], 4), round(q['yaw'], 4),
                           round(q['metric'], 1)] for q in r['cands'][1:]])
        if r['task'] == 'traffic_light':
            g = stop_gap(c['x'], c['y'], c['yaw'], rob, segments)
            pt['stop_line_gap_m'] = None if g is None else round(g, 3)
        out['points'].append(pt)
    os.makedirs(os.path.dirname(a.out_yaml), exist_ok=True)
    with open(a.out_yaml, 'w') as f:
        f.write('# 视觉识别点位 (唯一真值源) —— 由 tools/gen_recognition_points.py 计算\n'
                '# 坐标: 场地世界系 (= map 系), m / rad; 重算: python3 tools/gen_recognition_points.py\n')
        yaml.safe_dump(out, f, allow_unicode=True, sort_keys=False, default_flow_style=False)
    write_png(a.out_png, segments, targets, chosen, rob)

    with open(a.out_md, 'w') as f:
        f.write('# 视觉识别点位与拍照距离测算 (复赛)\n\n'
                '> 全部由 `tools/gen_recognition_points.py` 从真值源算出, 不是手填。\n'
                '> 重算: `python3 tools/gen_recognition_points.py`\n\n'
                '## 一、相机与车体\n\n| 项 | 值 |\n|---|---|\n')
        f.write('| 分辨率 | %.0f × %.0f |\n| HFOV / VFOV | %.1f° / %.1f° |\n'
                '| 像素焦距 fx | %.1f px |\n| 相机高度 / 俯仰 | %.3f m / 0° |\n'
                '| 车体 | %.3f × %.3f m |\n' % (rob['W'], rob['H'], rob['hfov_deg'],
                                            rob['vfov_deg'], rob['fx'], rob['mount'][2],
                                            rob['length'], rob['width']))
        f.write('\n距离 d 处: 1 m 宽目标占 %.0f/d px; 1 px = %.2f·d mm。\n\n'
                % (rob['fx'], 1000.0 / rob['fx']))
        f.write('## 二、点位\n\n'
                '| 目标 | 任务 | 车停 (x, y) | 朝向 | 距离 | 指标 | 正视度 | 画面余量 | 所在车道 |\n'
                '|---|---|---|---|---|---|---|---|---|\n')
        for r in rows:
            if not r['ok']:
                f.write('| `%s` | %s | — | — | — | **无可行解** | — | — | — |\n'
                        % (r['target'], r['task']))
                continue
            c = r['cands'][0]
            extra = ''
            if r['task'] == 'traffic_light':
                g = stop_gap(c['x'], c['y'], c['yaw'], rob, segments)
                extra = ' (距停止线 %.2f m)' % g if g is not None else ''
            f.write('| `%s` | %s | (%+.3f, %+.3f) | %+.1f° | %.3f m%s | %s %.1f (阈值 %g) '
                    '| %.2f | %.0f%% | %s |\n'
                    % (r['target'], r['task'], c['x'], c['y'], math.degrees(c['yaw']),
                       c['dist'], extra, r['unit'], c['metric'], r['thr'], c['cosi'],
                       c['frame'] * 100, lane_name(c['x'], c['y'], segments)))
            if r['task'] == 'standee':
                f.write('\n> `%s` 立牌在画面里高约 %.0f px。\n' % (r['target'], c['metric_h']))
        f.write('\n## 三、车道线 / 停止线真值 (从官方平面图提取)\n\n'
                '| 类型 | 方向 | 位置 | 范围 | 线宽 |\n|---|---|---|---|---|\n')
        for s in segments:
            f.write('| %s | %s | %s = %+.3f | [%+.3f, %+.3f] | %.0f mm |\n'
                    % ({'lane': '车道线', 'stop': '**停止线**', 'parking': '车位分隔线'}[s['type']],
                       '竖直' if s['kind'] == 'v' else '水平',
                       'x' if s['kind'] == 'v' else 'y', s['c'], s['a'], s['b'],
                       s['w'] * 1000))
        f.write('\n## 四、结论与注意点 (自动生成)\n\n')
        light = [r['cands'][0] for r in rows if r['ok'] and r['task'] == 'traffic_light']
        plate = [r['cands'][0] for r in rows if r['ok'] and r['task'] == 'plate']
        st = [r['cands'][0] for r in rows if r['ok'] and r['task'] == 'standee']
        if light:
            gaps = [g for g in (stop_gap(c['x'], c['y'], c['yaw'], rob, segments)
                                for c in light) if g is not None]
            f.write('* **红绿灯**: 点位落在**停止线内侧** (净距 %s m), 满足"红灯期间车身'
                    '不得越过停止线"; 灯珠直径 %.0f~%.0f px, 判颜色余量很大。\n'
                    % ('%.2f~%.2f' % (min(gaps), max(gaps)) if gaps else '?',
                       min(c['metric'] for c in light), max(c['metric'] for c in light)))
        if plate:
            f.write('* **车牌**: 不压车道线的前提下车只能停到车牌前 %.2f m, 车牌 **%.0f px 宽**'
                    '(每字形约 %.0f px)。\n'
                    '  - 分辨率下界实测: `tools/check_ocr_resolution.py` 把官方车牌重采样后喂'
                    'HyperLPR3 识别网络, **70 px 仍 3/3 全对**(加重模糊亦然)。\n'
                    '  - 所以识别不是瓶颈, 要解决的是"从整幅画面里把车牌框出来"(检测/定位)。\n'
                    % (min(c['dist'] for c in plate), min(c['metric'] for c in plate),
                       min(c['metric'] for c in plate) / 8.0))
        if st:
            vis = [c.get('vis', 0) for c in st]
            lat = [abs(c.get('lateral', 0)) for c in st]
            f.write('* **人偶立牌 (正对原则)**: 每个方向定一个**正对**该方向人偶的点位 —— '
                    '相机光轴垂直于立牌板面 (正视度 %.2f, 横向偏置 <=%.2f m)\n'
                    '  - 画面里立牌宽 %.0f~%.0f px、可见高 %.0f~%.0f px。\n'
                    % (min(c['cosi'] for c in st), max(lat),
                       min(c['metric'] for c in st), max(c['metric'] for c in st),
                       min(c['metric_h'] for c in st), max(c['metric_h'] for c in st)))
            f.write('  - 代价: 相机只有 **0.20 m 高且无俯仰**, 正对时最近只能站到 %.2f~%.2f m, '
                    '**下沿必然被切**, 可见高度约占立牌的 %.0f%%~%.0f%%。\n'
                    % (min(c['dist'] for c in st), max(c['dist'] for c in st),
                       min(vis) / 0.150 * 100, max(vis) / 0.150 * 100))
            f.write('  - 想拍全身需要 d >= %.2f m, 而车道只有 0.61 m 宽装不下 —— '
                    '唯一办法是给相机加 10°~15° 俯仰或抬高 (会偏离实车参数, 需先确认实车相机姿态)。\n'
                    % ((rob['mount'][2] - 0.004) / rob['half_h_tan']))
        f.write('* **车道约束**: 点位是按"车体不出白线 + 不压车道线/停止线 + 不进街区 + '
                '不碰道具"逐条筛出来的。街区在仿真里**没有实体墙**(只有地面画线), '
                '所以必须显式禁行, 否则车会从街区中间穿过去。\n'
                '  若官方认定某条走廊不是路 (例如 x∈[0.21,0.83] 那条竖走廊), '
                '改脚本顶部 `ROADS` (矩形走廊列表) 后重跑即可。\n'
                '* 复核用图: `docs/recognition_points.png` (灰=车道线, 红=停止线, '
                '浅绿=车位分隔线; 绿=人偶, 品红=红绿灯, 蓝=车牌)。\n')
    print()
    print('已写出:')
    for p in (a.out_yaml, a.out_md, a.out_png):
        print('   ', os.path.relpath(p, WS))


if __name__ == '__main__':
    main()
