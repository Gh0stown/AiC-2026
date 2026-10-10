#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""国赛（总决赛）场景要素生成器 —— 复赛场地的一个**可选图层**。

为什么单做一层
--------------
复赛的验收 / 路线 / 地图都建立在 `competition_arena.world` 上（当前 10/10 全过）。
国赛要素（楼宇/垃圾桶/电动车/站房/指示牌/异常温度）如果直接塞进那个 world，
会改动车道占用与雷达回波，**把已经验证过的东西重新搅乱**。
所以这里生成一个**独立 world** `competition_arena_national.world`：
    = 复赛 world 原样 + 国赛物料 include 块
复赛验收仍用原来的 world（一字未改），国赛场景用它。

依据（都在仓库里，可核对）
------------------------
* `复赛要求与差距分析.md` §3「总决赛考察内容」给了分值表与**已核对 PDF 的 ASCII 布局**
* `交接文档.md`：「官方示意图里中右列 (x 0.21~0.83) 是楼宇 A/B、下方带是楼宇 D/站房，都不是路」
* 比例：官方物料是 ~1:11 的比例模型（车板 34.5cm ↔ 真车 4.5m），这里沿用同一比例

各要素的"状态"怎么做成可识别的
------------------------------
国赛考的不是"有没有这个物体"，而是**状态**（垃圾桶满没满、楼宇有没有着火、
电动车违停/倒伏、站房仪表读数、有没有异常温度）。所以每个物料都带**可见状态差异**：

| 物料 | 状态 | 视觉差异 |
|---|---|---|
| 楼宇 A/B/C/D | 正常 / **火灾** | 火灾那栋有两扇窗是**自发光橙红火焰贴图** |
| 垃圾桶 | 正常 / **满溢** | 满溢时桶口堆出垃圾块、盖子掀开 |
| 电动车 | 正常 / **违停** / **倒伏** | 违停=停在黄网格禁停区；倒伏=整车侧翻 90° |
| 站房 | 仪表读数 | 表盘贴图（指针角度可配），旁边有读数牌 |
| 异常温度 | 正常 / **异常** | 异常时面板自发光红 + 温度牌显示高温 |

用法
----
    python3 tools/gen_national_props.py                # 生成模型 + 独立 world
    python3 tools/gen_national_props.py --show         # 只看摆放清单
    python3 tools/gen_national_props.py --remove       # 从国赛 world 里撤掉
"""
from __future__ import annotations
import argparse
import io
import math
import os
import shutil
import sys

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ARENA = os.path.join(WS, 'src', 'competition_arena')
MODELS = os.path.join(ARENA, 'models')
WORLD = os.path.join(ARENA, 'worlds', 'competition_arena.world')
OUT_WORLD = os.path.join(ARENA, 'worlds', 'competition_arena_national.world')
BEGIN = '<!-- BEGIN national -->'
END = '<!-- END national -->'

# ---------------------------------------------------------------------------
#  尺寸：沿用 1:11 的模型比例（真实 → 模型）
# ---------------------------------------------------------------------------
BLDG = dict(a=(0.62, 0.34, 0.46), b=(0.55, 0.32, 0.44),
            c=(0.44, 0.30, 0.40), d=(0.68, 0.30, 0.42))
STATION = (0.34, 0.26, 0.30)
BIN = dict(r=0.026, h=0.088)                 # 真桶 Ø0.55m / 高 1.0m
EBIKE = (0.155, 0.048, 0.098)                # 真车 1.8m x 0.6m x 1.1m
SIGN = (0.075, 0.016, 0.265)
HEATP = (0.095, 0.018, 0.135)

PUT = [
    # 楼宇（中右列 + 下方带，见交接文档的核对结论）
    dict(model='bldg_a_fire', name='bldg_a', x=0.95, y=1.30, yaw=math.pi, fire=True),
    dict(model='bldg_b', name='bldg_b', x=0.95, y=0.55, yaw=math.pi, fire=False),
    dict(model='bldg_c', name='bldg_c', x=0.35, y=-0.10, yaw=math.pi, fire=False),
    dict(model='bldg_d', name='bldg_d', x=-0.70, y=-1.72, yaw=0.0, fire=False),
    # 站房（2 处：一个在主场地下方带，一个在车位尽端）
    dict(model='station', name='station_1', x=0.42, y=-1.72, yaw=0.0, gauge=0.35),
    dict(model='station', name='station_2', x=1.95, y=-0.60, yaw=-math.pi / 2, gauge=0.62),
    # 垃圾桶（楼宇 C 旁一组三只，其中一只满溢）
    dict(model='bin_full', name='bin_1', x=0.60, y=-0.28, yaw=0.0),
    dict(model='bin_empty', name='bin_2', x=0.66, y=-0.36, yaw=0.0),
    dict(model='bin_empty', name='bin_3', x=0.72, y=-0.44, yaw=0.0),
    # 电动车 ×4（中右下：两台违停、一台倒伏、一台正常）
    dict(model='ebike', name='ebike_1', x=0.62, y=-0.72, yaw=1.2),
    dict(model='ebike', name='ebike_2', x=0.72, y=-0.86, yaw=1.45),
    dict(model='ebike_toppled', name='ebike_3', x=0.86, y=-0.68, yaw=2.6),
    dict(model='ebike', name='ebike_4', x=0.95, y=-0.95, yaw=0.4),
    # 指示牌（A 街区东侧）
    dict(model='sign', name='sign_1', x=-0.62, y=1.15, yaw=-math.pi / 2),
    # 异常温度面板（B 街区南侧管线）
    dict(model='heat_ok', name='heat_1', x=-1.30, y=-0.72, yaw=0.0),
    dict(model='heat_bad', name='heat_2', x=-1.16, y=-0.72, yaw=0.0),
]


# ---------------------------------------------------------------------------
#  贴图（程序化生成，与其它物料一致：不手摆、可复算）
# ---------------------------------------------------------------------------
def _tex_dir(model):
    d = os.path.join(MODELS, model, 'materials', 'textures')
    os.makedirs(d, exist_ok=True)
    return d


def tex_windows(model, cols, rows, fire=False, seed=0):
    """楼宇立面：窗格。fire=True 时右下两扇窗是自发光火焰色。"""
    from PIL import Image, ImageDraw
    import random
    rng = random.Random(seed)
    W, H = 256, 256
    im = Image.new('RGB', (W, H), (206, 200, 188))
    d = ImageDraw.Draw(im)
    d.rectangle([0, H - 26, W, H], fill=(150, 146, 138))            # 勒脚
    for r in range(rows):
        for c in range(cols):
            x = 14 + c * (W - 28) / cols
            y = 16 + r * (H - 56) / rows
            w = (W - 28) / cols * 0.62
            h = (H - 56) / rows * 0.55
            lit = rng.random() < 0.35
            col = (250, 238, 190) if lit else (96, 120, 140)
            d.rectangle([x, y, x + w, y + h], fill=col, outline=(120, 116, 108))
    if fire:
        for (c, r) in ((cols - 1, rows - 1), (cols - 2, rows - 2)):
            x = 14 + c * (W - 28) / cols
            y = 16 + r * (H - 56) / rows
            w = (W - 28) / cols * 0.62
            h = (H - 56) / rows * 0.55
            d.rectangle([x, y, x + w, y + h], fill=(255, 120, 20))
            d.polygon([(x + w / 2, y + 2), (x + w - 2, y + h - 2), (x + 2, y + h - 2)],
                      fill=(255, 230, 90))
    p = os.path.join(_tex_dir(model), 'facade.png')
    im.save(p)
    return p


def tex_gauge(model, frac=0.35):
    """站房仪表盘：白色表盘 + 指针（frac = 指针位置 0~1）。"""
    from PIL import Image, ImageDraw
    W = H = 192
    im = Image.new('RGB', (W, H), (60, 64, 70))
    d = ImageDraw.Draw(im)
    d.ellipse([18, 18, W - 18, H - 18], fill=(242, 240, 232), outline=(30, 30, 32), width=4)
    for i in range(11):
        a = math.radians(-210 + i * 24)
        r0, r1 = 62, 74
        cx, cy = W / 2, H / 2
        d.line([cx + r0 * math.cos(a), cy + r0 * math.sin(a),
                cx + r1 * math.cos(a), cy + r1 * math.sin(a)], fill=(40, 40, 44), width=3)
    a = math.radians(-210 + frac * 240)
    d.line([W / 2, H / 2, W / 2 + 58 * math.cos(a), H / 2 + 58 * math.sin(a)],
           fill=(200, 30, 30), width=5)
    d.ellipse([W / 2 - 6, H / 2 - 6, W / 2 + 6, H / 2 + 6], fill=(40, 40, 44))
    p = os.path.join(_tex_dir(model), 'gauge.png')
    im.save(p)
    return p


def tex_heat(model, bad=True):
    """异常温度面板：正常=绿底读数；异常=自发光红 + 高温数值条。"""
    from PIL import Image, ImageDraw
    W, H = 160, 96
    im = Image.new('RGB', (W, H), (40, 46, 52))
    d = ImageDraw.Draw(im)
    d.rectangle([6, 6, W - 7, H - 7], outline=(90, 96, 104), width=2)
    bar = (210, 60, 50) if bad else (70, 190, 110)
    d.rectangle([16, 20, W - 17, 40], fill=bar)
    for i in range(6):
        d.rectangle([18 + i * 18, 48, 30 + i * 18, 74],
                    fill=(230, 90, 70) if bad else (90, 180, 120))
    p = os.path.join(_tex_dir(model), 'panel.png')
    im.save(p)
    return p


def _mat(name, tex):
    return ('<material><script><uri>model://%s/materials/scripts</uri>'
            '<uri>model://%s/materials/textures</uri><name>%s</name></script></material>'
            % (name, name, tex))


def _script(model, pairs):
    d = os.path.join(MODELS, model, 'materials', 'scripts')
    os.makedirs(d, exist_ok=True)
    out = []
    for mat, tex, emissive in pairs:
        out.append('material %s\n{\n  technique { pass {\n'
                   '    ambient 0.9 0.9 0.9 1\n    diffuse 1 1 1 1\n'
                   '    emissive %s\n'
                   '    texture_unit { texture %s }\n  } }\n}\n'
                   % (mat, emissive, os.path.basename(tex)))
    io.open(os.path.join(d, '%s.material' % model), 'w', encoding='utf-8').write('\n'.join(out))


def _model_config(model, extra=''):
    d = os.path.join(MODELS, model)
    io.open(os.path.join(d, 'model.config'), 'w', encoding='utf-8').write(
        '<?xml version="1.0"?>\n<model>\n  <name>%s</name>\n  <version>1.0</version>\n'
        '  <sdf version="1.7">model.sdf</sdf>\n%s</model>\n' % (model, extra))


def _write_sdf(model, name, body):
    d = os.path.join(MODELS, model)
    os.makedirs(d, exist_ok=True)
    io.open(os.path.join(d, 'model.sdf'), 'w', encoding='utf-8').write(
        '<?xml version="1.0"?>\n<sdf version="1.7">\n  <model name="%s">\n'
        '    <static>true</static>\n    <link name="link">\n%s\n    </link>\n'
        '  </model>\n</sdf>\n' % (name, body))


def _box(nm, size, pose, mat=None, col=False):
    m = _mat('', '') if mat is None else ''
    geo = ''
    if col:
        geo = '<collision name="%s_c"><pose>%s</pose><geometry><box><size>%s</size></box>' \
              '</geometry></collision>' % (nm, pose, size)
    vis = ('<visual name="%s"><pose>%s</pose><geometry><box><size>%s</size></box></geometry>'
           '%s</visual>' % (nm, pose, size, mat or '')) if not col else ''
    return (vis + geo).replace('  ', ' ')


def _cyl(nm, r, l, pose, mat=None, col=False):
    geo = '<geometry><cylinder><radius>%.4f</radius><length>%.4f</length></cylinder></geometry>' \
          % (r, l)
    if col:
        return '<collision name="%s_c"><pose>%s</pose>%s</collision>' % (nm, pose, geo)
    return '<visual name="%s"><pose>%s</pose>%s%s</visual>' % (nm, pose, geo, mat or '')


# ---------------------------------------------------------------------------
#  材质引用
# ---------------------------------------------------------------------------
def MAT(model, matname):
    return ('<material><script><uri>model://%s/materials/scripts</uri>'
            '<uri>model://%s/materials/textures</uri><name>%s</name></script></material>'
            % (model, model, matname))


def MAT_PLAIN(rgb, emissive=None):
    e = '<emissive>%s</emissive>' % rgb if emissive else ''
    return ('<material><ambient>%s</ambient><diffuse>%s</diffuse>%s</material>'
            % (rgb, rgb, e))


# ---------------------------------------------------------------------------
#  各物料的 SDF
# ---------------------------------------------------------------------------
def build_bldg(key, fire):
    """楼宇：主楼体（立面窗格贴图）+ 屋顶挑檐 + 门；fire=True 时两扇窗自发光火焰。"""
    model = 'bldg_%s%s' % (key, '_fire' if fire else '')
    w, d, h = BLDG[key]
    cols, rows = (5, 4) if key != 'c' else (4, 3)
    tex = tex_windows(model, cols, rows, fire=fire, seed=ord(key))
    _script(model, [('BldgFacade', tex, '0.25 0.25 0.25 1'),
                    ('BldgFire', tex, '1 0.75 0.35 1')])
    body = []
    body.append(_box('mass', '%.3f %.3f %.3f' % (w, d, h), '0 0 %.4f 0 0 0' % (h / 2),
                     MAT(model, 'BldgFire' if fire else 'BldgFacade'), col=False))
    body.append(_box('mass_c', '%.3f %.3f %.3f' % (w, d, h), '0 0 %.4f 0 0 0' % (h / 2),
                     col=True))
    body.append(_box('roof', '%.3f %.3f 0.012' % (w + 0.02, d + 0.02),
                     '0 0 %.4f 0 0 0' % (h + 0.006), MAT_PLAIN('0.35 0.34 0.33 1')))
    body.append(_box('door', '0.035 0.004 0.055', '0 %.4f %.4f 0 0 0' % (-d / 2 - 0.002, 0.028),
                     MAT_PLAIN('0.25 0.22 0.20 1')))
    _write_sdf(model, model, '\n      '.join(body))
    _model_config(model)
    return model


def build_station(gauge_frac):
    """站房：小屋 + 门 + 仪表盘（指针角度 = gauge_frac）+ 读数牌。"""
    model = 'station'
    w, d, h = STATION
    tex = tex_gauge(model, gauge_frac)
    _script(model, [('Gauge', tex, '0.55 0.55 0.55 1')])
    body = [
        _box('hut', '%.3f %.3f %.3f' % (w, d, h), '0 0 %.4f 0 0 0' % (h / 2),
             MAT_PLAIN('0.72 0.70 0.66 1')),
        _box('hut_c', '%.3f %.3f %.3f' % (w, d, h), '0 0 %.4f 0 0 0' % (h / 2), col=True),
        _box('roof', '%.3f %.3f 0.014' % (w + 0.03, d + 0.03), '0 0 %.4f 0 0 0' % (h + 0.007),
             MAT_PLAIN('0.42 0.40 0.38 1')),
        _box('door', '0.045 0.004 0.075', '0 %.4f %.4f 0 0 0' % (-d / 2 - 0.002, 0.038),
             MAT_PLAIN('0.30 0.34 0.40 1')),
        _box('gauge', '0.055 0.006 0.055', '%.4f %.4f %.4f 0 0 0'
             % (-w / 4, -d / 2 - 0.004, h * 0.62), MAT(model, 'Gauge')),
        _box('gauge_c', '0.055 0.006 0.055', '%.4f %.4f %.4f 0 0 0'
             % (-w / 4, -d / 2 - 0.004, h * 0.62), col=True),
        _box('plate', '0.075 0.004 0.03', '%.4f %.4f %.4f 0 0 0'
             % (w / 5, -d / 2 - 0.003, h * 0.45), MAT_PLAIN('0.90 0.90 0.88 1')),
    ]
    _write_sdf(model, model, '\n      '.join(body))
    _model_config(model)
    return model


def build_bin(full):
    """垃圾桶：桶身 + 盖子；满溢时盖子掀开、桶口堆垃圾。"""
    model = 'bin_full' if full else 'bin_empty'
    r, h = BIN['r'], BIN['h']
    body = [
        _cyl('body', r, h, '0 0 %.4f 0 0 0' % (h / 2), MAT_PLAIN('0.22 0.45 0.28 1')),
        _cyl('body_c', r, h, '0 0 %.4f 0 0 0' % (h / 2), col=True),
        _box('rim', '%.4f %.4f 0.008' % (r * 2.2, r * 2.2), '0 0 %.4f 0 0 0' % (h - 0.004),
             MAT_PLAIN('0.16 0.34 0.20 1')),
    ]
    if full:
        body.append(_box('lid', '%.4f %.4f 0.006' % (r * 2.2, r * 2.2),
                         '%.4f 0 %.4f 0 -0.55 0' % (r * 0.9, h + 0.022),
                         MAT_PLAIN('0.20 0.40 0.24 1')))
        body += [_box('trash%d' % i, '0.020 0.016 0.014',
                      '%.3f %.3f %.4f %.2f 0 %.2f' % (
                          0.010 * (i - 1), 0.008 * ((i + 1) % 2 - 0.5), h + 0.010,
                          0.3 * i, 0.4 * i),
                      MAT_PLAIN('0.55 0.48 0.30 1')) for i in range(3)]
    else:
        body.append(_box('lid', '%.4f %.4f 0.006' % (r * 2.2, r * 2.2),
                         '0 0 %.4f 0 0 0' % (h + 0.004), MAT_PLAIN('0.20 0.40 0.24 1')))
    _write_sdf(model, model, '\n      '.join(body))
    _model_config(model)
    return model


def build_ebike(toppled):
    """电动车：车架 + 前后轮 + 把手 + 座垫。toppled=True 时整车侧翻 90°。"""
    model = 'ebike_toppled' if toppled else 'ebike'
    L, W, H = EBIKE
    roll = math.pi / 2 if toppled else 0.0

    def pose(x, y, z, yaw=0.0):
        if toppled:
            # 绕 x 轴侧翻：把 (y,z) 平面旋转 90°，同时抬高到"躺着"的高度
            return '%.4f %.4f %.4f %.4f %.4f 0' % (x, -z + W / 2 + 0.012, y + 0.010, roll, yaw)
        return '%.4f %.4f %.4f 0 0 %.4f' % (x, y, z, yaw)

    def rpose(nm, size, x, y, z, yaw=0.0, mat=None):
        return _box(nm, size, pose(x, y, z, yaw), mat)

    body = [
        rpose('frame', '%.3f 0.020 0.030' % (L * 0.72,), 0, 0, H * 0.42,
              mat=MAT_PLAIN('0.16 0.30 0.62 1')),
        rpose('battery', '%.3f 0.030 0.028' % (L * 0.34,), -L * 0.05, 0, H * 0.26,
              mat=MAT_PLAIN('0.12 0.12 0.14 1')),
        rpose('seat', '0.052 0.030 0.014', -L * 0.20, 0, H * 0.60,
              mat=MAT_PLAIN('0.10 0.10 0.12 1')),
        rpose('bar', '0.010 0.056 0.010', L * 0.33, 0, H * 0.78,
              mat=MAT_PLAIN('0.20 0.20 0.22 1')),
        rpose('head', '0.024 0.020 0.022', L * 0.36, 0, H * 0.62,
              mat=MAT_PLAIN('0.85 0.85 0.82 1')),
    ]
    # 两个轮子：圆柱轴沿 y
    for nm, x in (('wheel_f', L * 0.34), ('wheel_r', -L * 0.28)):
        body.append('<visual name="%s"><pose>%s</pose><geometry><cylinder>'
                    '<radius>%.4f</radius><length>0.010</length></cylinder></geometry>'
                    '%s</visual>' % (nm, pose(x, 0, 0.026, math.pi / 2), 0.026,
                                     MAT_PLAIN('0.08 0.08 0.09 1')))
        body.append(_cyl('%s_c' % nm, 0.026, 0.010,
                         pose(x, 0, 0.026, math.pi / 2), col=True))
    # 支撑脚（不倒时装，倒伏时不需要）
    if not toppled:
        body.append(_cyl('kick', 0.003, 0.026, pose(-L * 0.22, 0.022, 0.014, 0.35),
                         MAT_PLAIN('0.25 0.25 0.27 1'), col=True))
    _write_sdf(model, model, '\n      '.join(body))
    _model_config(model)
    return model


def build_sign():
    """指示牌：立柱 + 圆牌（社区导视牌）。"""
    model = 'sign'
    w, t, h = SIGN
    body = [
        _cyl('post', 0.006, h, '0 0 %.4f 0 0 0' % (h / 2), MAT_PLAIN('0.45 0.46 0.48 1')),
        _cyl('post_c', 0.006, h, '0 0 %.4f 0 0 0' % (h / 2), col=True),
        _box('board', '%.4f %.4f 0.055' % (w, t), '0 0 %.4f 0 0 0' % (h - 0.045),
             MAT_PLAIN('0.15 0.35 0.62 1')),
        _box('board_c', '%.4f %.4f 0.055' % (w, t), '0 0 %.4f 0 0 0' % (h - 0.045), col=True),
        _box('plate', '0.055 0.004 0.022', '0 -0.010 %.4f 0 0 0' % (h - 0.045),
             MAT_PLAIN('0.95 0.95 0.92 1')),
    ]
    _write_sdf(model, model, '\n      '.join(body))
    _model_config(model)
    return model


def build_heat(bad):
    """异常温度：管 + 温度面板（异常时自发光红）。"""
    model = 'heat_bad' if bad else 'heat_ok'
    w, t, h = HEATP
    tex = tex_heat(model, bad=bad)
    _script(model, [('Panel', tex, '1 0.55 0.35 1' if bad else '0.45 0.55 0.45 1')])
    body = [
        _cyl('pipe', 0.010, 0.30, '0 0 0.05 0 1.5708 0', MAT_PLAIN('0.52 0.53 0.55 1')),
        _cyl('pipe_c', 0.010, 0.30, '0 0 0.05 0 1.5708 0', col=True),
        _box('panel', '%.4f %.4f %.4f' % (w, t, h), '0 0.020 %.4f 0 0 0' % (0.16),
             MAT(model, 'Panel')),
        _box('panel_c', '%.4f %.4f %.4f' % (w, t, h), '0 0.020 %.4f 0 0 0' % (0.16), col=True),
    ]
    _write_sdf(model, model, '\n      '.join(body))
    _model_config(model)
    return model


# ---------------------------------------------------------------------------
#  world 组装
# ---------------------------------------------------------------------------
def build_world(remove=False):
    if not os.path.isfile(WORLD):
        sys.exit('找不到复赛 world: %s' % WORLD)
    s = io.open(WORLD, encoding='utf-8').read()
    import re
    s = re.sub(re.escape(BEGIN) + r'.*?' + re.escape(END) + r'\n?', '', s, flags=re.S)
    if not remove:
        s = s.replace('</world>', build_includes() + '\n</world>', 1)
    io.open(OUT_WORLD, 'w', encoding='utf-8').write(s)
    return OUT_WORLD


def build_includes():
    out = [BEGIN]
    for p in PUT:
        out += ['  <include>', '    <uri>model://%s</uri>' % p['model'],
                '    <name>%s</name>' % p['name'],
                '    <pose>%.4f %.4f 0 0 0 %.4f</pose>' % (p['x'], p['y'], p['yaw']),
                '  </include>']
    out.append(END)
    return '\n'.join(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--show', action='store_true', help='只列摆放清单')
    ap.add_argument('--remove', action='store_true', help='从国赛 world 里撤掉国赛物料')
    a = ap.parse_args()

    if a.show:
        print('  国赛物料摆放（共 %d 件）:' % len(PUT))
        for p in PUT:
            print('    %-14s %-10s (%+.2f, %+.2f) yaw=%.2f'
                  % (p['name'], p['model'], p['x'], p['y'], p['yaw']))
        return 0

    if a.remove:
        print('  已撤: %s' % build_world(remove=True))
        return 0

    made = []
    for k in BLDG:
        made.append(build_bldg(k, fire=(k == 'a')))
    made.append(build_station(0.35))
    made.append(build_bin(full=True))
    made.append(build_bin(full=False))
    made.append(build_ebike(toppled=False))
    made.append(build_ebike(toppled=True))
    made.append(build_sign())
    made.append(build_heat(bad=True))
    made.append(build_heat(bad=False))
    print('  已生成 %d 个物料模型: %s' % (len(made), ', '.join(made)))

    w = build_world()
    print('  国赛 world -> %s' % os.path.relpath(w, WS))
    print('  摆放 %d 件（楼宇 A 带火灾、垃圾桶一只满溢、电动车含违停与倒伏、'
          '站房 2 处含仪表、异常温度面板）' % len(PUT))
    return 0


if __name__ == '__main__':
    sys.exit(main())
