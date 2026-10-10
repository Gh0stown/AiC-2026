#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""国赛（总决赛）场景要素生成器 —— 复赛场地的一个**可选图层**。

尺寸依据（用户提供的淘宝商品页参考图，非官方物料；国赛道具本身就是塑料模型）
--------------------------------------------------------------------------------
| 道具 | 参考尺寸 | 说明 |
|---|---|---|
| 电单车 | **长 18 × 高 12 cm** | 共享单车风格：前篮 + 黑座箱 + 两轮 + 脚撑；另配头盔 2.9×3.2×2 cm |
| 迷你垃圾桶 | **4 × 4.5 × 5.5 cm** | 四色分类：红=有害 / 蓝=可回收 / 绿=厨余 / 灰=其他，黑盖 |
| 楼宇模型 | **7.2 / 7.8 / 20 cm** 高 | 15#（L 形）/ 11#（带 T 字竖条）/ 20#（窗格塔楼）|
| 站房 | 未找到 | 用户意见：干脆只放**仪表**（0.09×0.03×0.16 m 立牌）|

为什么单做一层
--------------
复赛的验收 / 路线 / 地图都建立在 `competition_arena.world` 上（当前 10/10 全过）。
国赛要素直接塞进去会改动车道占用与雷达回波，**把已验证的东西重新搅乱**。
所以这里生成**独立 world** = 复赛 world 原样 + 国赛 include 块；复赛验收走原 world。

状态差异（国赛考的是"状态"，不是"有没有"）
------------------------------------------
* 楼宇 20#（或指定那栋）两扇窗**自发光火焰** -> 火灾
* 垃圾桶：**满溢**那只有垃圾堆 + 盖子掀开；其余盖子合上
* 电动车：**违停**（停在车位外）/ **倒伏**（整车侧翻 90°）/ 正常
* 仪表：**指针角度可配**（生成时给读数）
* 温度面板：异常那块**自发光红**

用法
----
    python3 tools/gen_national_props.py            # 生成模型 + 独立 world
    python3 tools/gen_national_props.py --show     # 只看摆放清单
    python3 tools/gen_national_props.py --remove   # 撤掉
"""
from __future__ import annotations

import argparse
import io
import math
import os
import re
import sys

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ARENA = os.path.join(WS, 'src', 'competition_arena')
MODELS = os.path.join(ARENA, 'models')
WORLD = os.path.join(ARENA, 'worlds', 'competition_arena.world')
OUT_WORLD = os.path.join(ARENA, 'worlds', 'competition_arena_national.world')
BEGIN = '<!-- BEGIN national -->'
END = '<!-- END national -->'

# ---------------------------------------------------------------------------
#  尺寸（米）
# ---------------------------------------------------------------------------
BLDGS = {
    # 型号:   (宽 x, 深 y, 高 z, 备注)
    'bldg_15': (0.040, 0.035, 0.072),     # 15#: 7.2 cm 高, L 形
    'bldg_11': (0.040, 0.028, 0.078),     # 11#: 7.8 cm 高, 带 T 字竖条
    'bldg_20': (0.085, 0.055, 0.200),     # 20#: 20 cm 高, 窗格塔楼
}
EBIKE = (0.180, 0.050, 0.120)             # 长 x 宽 x 高（18 x 12 cm, 宽按同款取 5 cm）
HELMET = (0.029, 0.032, 0.020)
BIN = (0.040, 0.045, 0.055)               # 迷你垃圾桶 4 x 4.5 x 5.5 cm
GAUGE = (0.090, 0.030, 0.160)             # 仪表立牌（站房那项的替代）
SIGN = (0.075, 0.016, 0.265)
HEATP = (0.095, 0.018, 0.135)

BIN_COLORS = [('red', '有害垃圾', (0.78, 0.15, 0.16)),
              ('blue', '可回收物', (0.13, 0.35, 0.72)),
              ('green', '厨余垃圾', (0.16, 0.52, 0.26)),
              ('gray', '其他垃圾', (0.52, 0.53, 0.55))]

# 摆放：楼宇在中右列（交接文档核对过"x 0.21~0.83 是楼宇 A/B"），
#       垃圾桶在楼宇 C 一带，电动车在中右下（含违停与倒伏），仪表两处，指示牌在 A 街区旁。
PUT = [
    dict(model='bldg_15', name='bldg_15', x=1.02, y=1.34, yaw=math.pi),
    dict(model='bldg_11', name='bldg_11', x=1.02, y=1.02, yaw=math.pi),
    dict(model='bldg_20', name='bldg_20', x=1.06, y=0.42, yaw=math.pi, fire=True),
    dict(model='bin_red', name='bin_1', x=0.34, y=-0.06, yaw=0.0, full=True),
    dict(model='bin_blue', name='bin_2', x=0.40, y=-0.14, yaw=0.0),
    dict(model='bin_green', name='bin_3', x=0.46, y=-0.06, yaw=0.0),
    dict(model='bin_gray', name='bin_4', x=0.52, y=-0.14, yaw=0.0),
    dict(model='ebike', name='ebike_1', x=0.62, y=-0.72, yaw=1.20),
    dict(model='ebike', name='ebike_2', x=0.60, y=-0.90, yaw=1.45),
    dict(model='ebike_toppled', name='ebike_3', x=0.40, y=-0.66, yaw=2.60),
    dict(model='ebike', name='ebike_4', x=0.42, y=-0.98, yaw=0.40),
    dict(model='helmet', name='helmet_1', x=0.36, y=-0.50, yaw=0.4),
    dict(model='gauge', name='gauge_1', x=0.42, y=-1.72, yaw=0.0, dial=0.35),
    dict(model='gauge', name='gauge_2', x=1.95, y=-0.60, yaw=-math.pi / 2, dial=0.72),
    dict(model='sign', name='sign_1', x=-0.62, y=1.15, yaw=-math.pi / 2),
    dict(model='heat_ok', name='heat_1', x=-1.30, y=-0.72, yaw=0.0),
    dict(model='heat_bad', name='heat_2', x=-1.16, y=-0.72, yaw=0.0),
]


# ---------------------------------------------------------------------------
#  贴图（程序化）
# ---------------------------------------------------------------------------
def _tex_dir(model):
    d = os.path.join(MODELS, model, 'materials', 'textures')
    os.makedirs(d, exist_ok=True)
    return d


def tex_bldg(model, kind, fire=False):
    """塑料模型楼立面：浅灰墙 + 深色窗格（15#/11#/20# 三种排布）。"""
    from PIL import Image, ImageDraw
    W = H = 256
    im = Image.new('RGB', (W, H), (206, 204, 198))
    d = ImageDraw.Draw(im)
    if kind == 'bldg_11':
        d.rectangle([W * 0.44, 0, W * 0.56, H], fill=(90, 92, 96))     # T 字竖条
        cols, rows = 4, 7
    elif kind == 'bldg_20':
        cols, rows = 8, 12                                             # 密集窗格
    else:
        cols, rows = 4, 5
    for r in range(rows):
        for c in range(cols):
            if kind == 'bldg_11' and 0.42 < (c + 0.5) / cols < 0.58:
                continue
            x = 10 + c * (W - 20) / cols
            y = 8 + r * (H - 30) / rows
            w = (W - 20) / cols * 0.72
            h = (H - 30) / rows * 0.62
            d.rectangle([x, y, x + w, y + h], fill=(70, 88, 104), outline=(150, 148, 142))
    if fire:
        for (c, r) in ((cols - 1, rows - 1), (cols - 2, rows - 3)):
            x = 10 + c * (W - 20) / cols
            y = 8 + r * (H - 30) / rows
            w = (W - 20) / cols * 0.72
            h = (H - 30) / rows * 0.62
            d.rectangle([x, y, x + w, y + h], fill=(255, 110, 20))
            d.polygon([(x + w / 2, y + 2), (x + w - 2, y + h - 2), (x + 2, y + h - 2)],
                      fill=(255, 232, 96))
    p = os.path.join(_tex_dir(model), 'facade.png')
    im.save(p)
    return p


def tex_bin(model, rgb, label, full=False):
    """垃圾桶侧面：底色 + 白色分类图标（三角形/循环箭/沙漏/叉）+ 中文分类名。"""
    from PIL import Image, ImageDraw
    W, H = 192, 256
    im = Image.new('RGB', (W, H), tuple(int(c * 255) for c in rgb))
    d = ImageDraw.Draw(im)
    cx, cy, r = W / 2, H * 0.36, 46
    d.ellipse([cx - r, cy - r, cx + r, cy + r], outline=(255, 255, 255), width=5)
    if label == '可回收物':
        for k in range(3):
            a = math.radians(-90 + k * 120)
            d.polygon([(cx + (r - 12) * math.cos(a), cy + (r - 12) * math.sin(a)),
                       (cx + (r - 30) * math.cos(a + 0.5), cy + (r - 30) * math.sin(a + 0.5)),
                       (cx + (r - 30) * math.cos(a - 0.5), cy + (r - 30) * math.sin(a - 0.5))],
                      fill=(255, 255, 255))
    elif label == '厨余垃圾':
        d.polygon([(cx - 20, cy - 20), (cx + 20, cy - 20), (cx, cy + 22)], fill=(255, 255, 255))
    elif label == '有害垃圾':
        d.line([cx - 22, cy - 22, cx + 22, cy + 22], fill=(255, 255, 255), width=7)
        d.line([cx - 22, cy + 22, cx + 22, cy - 22], fill=(255, 255, 255), width=7)
    else:
        d.polygon([(cx, cy - 26), (cx + 24, cy + 20), (cx - 24, cy + 20)], fill=(255, 255, 255))
    # ★ 中文分类名要用中文字体 (PIL 默认位图字体是 latin-1, 写中文会抛 UnicodeEncodeError)
    fp = next((f for f in ('/mnt/c/Windows/Fonts/msyhbd.ttc',
                           '/mnt/c/Windows/Fonts/simhei.ttf',
                           '/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf')
               if os.path.isfile(f)), None)
    if fp:
        from PIL import ImageFont
        try:
            d.text((14, H * 0.70), label, fill=(255, 255, 255),
                   font=ImageFont.truetype(fp, 26))
        except Exception:                                       # noqa: BLE001
            pass
    if full:
        d.rectangle([0, 0, W - 1, 14], fill=(250, 240, 180))
    p = os.path.join(_tex_dir(model), 'side.png')
    im.save(p)
    return p


def tex_gauge(model, frac=0.35):
    from PIL import Image, ImageDraw
    W = H = 192
    im = Image.new('RGB', (W, H), (58, 62, 68))
    d = ImageDraw.Draw(im)
    d.ellipse([18, 18, W - 18, H - 18], fill=(242, 240, 232), outline=(28, 28, 30), width=4)
    for i in range(11):
        a = math.radians(-210 + i * 24)
        d.line([W / 2 + 62 * math.cos(a), H / 2 + 62 * math.sin(a),
                W / 2 + 74 * math.cos(a), H / 2 + 74 * math.sin(a)], fill=(40, 40, 44), width=3)
    a = math.radians(-210 + frac * 240)
    d.line([W / 2, H / 2, W / 2 + 58 * math.cos(a), H / 2 + 58 * math.sin(a)],
           fill=(200, 30, 30), width=5)
    d.ellipse([W / 2 - 6, H / 2 - 6, W / 2 + 6, H / 2 + 6], fill=(40, 40, 44))
    p = os.path.join(_tex_dir(model), 'gauge.png')
    im.save(p)
    return p


def tex_heat(model, bad=True):
    from PIL import Image, ImageDraw
    W, H = 160, 96
    im = Image.new('RGB', (W, H), (40, 46, 52))
    d = ImageDraw.Draw(im)
    d.rectangle([6, 6, W - 7, H - 7], outline=(90, 96, 104), width=2)
    d.rectangle([16, 20, W - 17, 40], fill=(210, 60, 50) if bad else (70, 190, 110))
    for i in range(6):
        d.rectangle([18 + i * 18, 48, 30 + i * 18, 74],
                    fill=(230, 90, 70) if bad else (90, 180, 120))
    p = os.path.join(_tex_dir(model), 'panel.png')
    im.save(p)
    return p


# ---------------------------------------------------------------------------
#  SDF 组装小工具
# ---------------------------------------------------------------------------
def MAT(model, matname):
    return ('<material><script><uri>model://%s/materials/scripts</uri>'
            '<uri>model://%s/materials/textures</uri><name>%s</name></script></material>'
            % (model, model, matname))


def FLAT(rgb, emissive=None):
    e = '<emissive>%s</emissive>' % rgb if emissive else ''
    return ('<material><ambient>%s</ambient><diffuse>%s</diffuse>%s</material>'
            % (rgb, rgb, e))


def _script(model, pairs):
    d = os.path.join(MODELS, model, 'materials', 'scripts')
    os.makedirs(d, exist_ok=True)
    out = []
    for mat, tex, em in pairs:
        out.append('material %s\n{\n  technique { pass {\n'
                   '    ambient 0.85 0.85 0.85 1\n    diffuse 1 1 1 1\n'
                   '    emissive %s\n    texture_unit { texture %s }\n  } }\n}\n'
                   % (mat, em, os.path.basename(tex)))
    io.open(os.path.join(d, '%s.material' % model), 'w', encoding='utf-8').write('\n'.join(out))


def BOX(nm, size, pose, mat='', col=False):
    geo = '<geometry><box><size>%s</size></box></geometry>' % size
    if col:
        return '<collision name="%s_c"><pose>%s</pose>%s</collision>' % (nm, pose, geo)
    return '<visual name="%s"><pose>%s</pose>%s%s</visual>' % (nm, pose, geo, mat)


def CYL(nm, r, l, pose, mat='', col=False):
    geo = ('<geometry><cylinder><radius>%.4f</radius><length>%.4f</length></cylinder>'
           '</geometry>' % (r, l))
    if col:
        return '<collision name="%s_c"><pose>%s</pose>%s</collision>' % (nm, pose, geo)
    return '<visual name="%s"><pose>%s</pose>%s%s</visual>' % (nm, pose, geo, mat)


def _write(model, body, static=True):
    d = os.path.join(MODELS, model)
    os.makedirs(d, exist_ok=True)
    io.open(os.path.join(d, 'model.sdf'), 'w', encoding='utf-8').write(
        '<?xml version="1.0"?>\n<sdf version="1.7">\n  <model name="%s">\n'
        '    <static>%s</static>\n    <link name="link">\n%s\n    </link>\n'
        '  </model>\n</sdf>\n' % (model, str(static).lower(), '\n      '.join(body)))
    io.open(os.path.join(d, 'model.config'), 'w', encoding='utf-8').write(
        '<?xml version="1.0"?>\n<model>\n  <name>%s</name>\n  <version>1.0</version>\n'
        '  <sdf version="1.7">model.sdf</sdf>\n</model>\n' % model)

# ---------------------------------------------------------------------------
#  各物料的 SDF（按实物尺寸）
# ---------------------------------------------------------------------------
def build_bldg(key):
    """塑料模型楼：主楼体（窗格贴图）+ 屋顶；bldg_20 带火灾窗。15# 做成 L 形。"""
    w, d, h = BLDGS[key]
    fire = (key == 'bldg_20')
    tex = tex_bldg(key, key, fire=fire)
    _script(key, [('Facade', tex, '1 0.72 0.30 1' if fire else '0.22 0.22 0.22 1')])
    body = []
    if key == 'bldg_15':                       # L 形：主楼 + 侧翼
        body.append(BOX('wing', '%.4f %.4f %.4f' % (w * 0.55, d, h * 0.72),
                        '%.4f 0 %.4f 0 0 0' % (-w * 0.35, h * 0.36),
                        MAT(key, 'Facade')))
        body.append(BOX('wing_c', '%.4f %.4f %.4f' % (w * 0.55, d, h * 0.72),
                        '%.4f 0 %.4f 0 0 0' % (-w * 0.35, h * 0.36), col=True))
    body.append(BOX('mass', '%.4f %.4f %.4f' % (w, d, h), '0 0 %.4f 0 0 0' % (h / 2),
                    MAT(key, 'Facade')))
    body.append(BOX('mass_c', '%.4f %.4f %.4f' % (w, d, h), '0 0 %.4f 0 0 0' % (h / 2),
                    col=True))
    body.append(BOX('roof', '%.4f %.4f 0.004' % (w + 0.004, d + 0.004),
                    '0 0 %.4f 0 0 0' % (h + 0.002), FLAT('0.62 0.61 0.58 1')))
    _write(key, body)
    return key


def build_bin(color, label, full=False):
    """迷你分类垃圾桶：4 x 4.5 x 5.5 cm，桶身带分类图标，黑盖；满溢时盖子掀开 + 垃圾堆。"""
    model = 'bin_%s' % color
    w, d, h = BIN
    rgb = dict((c, v) for c, _l, v in BIN_COLORS)[color]
    rgb_s = '%.3f %.3f %.3f 1' % rgb
    tex = tex_bin(model, rgb, label, full=full)
    _script(model, [('Side', tex, '0.20 0.20 0.20 1')])
    body = [
        BOX('body', '%.4f %.4f %.4f' % (w, d, h * 0.86), '0 0 %.4f 0 0 0' % (h * 0.43),
            MAT(model, 'Side')),
        BOX('body_c', '%.4f %.4f %.4f' % (w, d, h * 0.86), '0 0 %.4f 0 0 0' % (h * 0.43),
            col=True),
        # 黑盖：正常合上；满溢时向后掀起
        BOX('lid', '%.4f %.4f %.4f' % (w * 1.06, d * 1.06, h * 0.10),
            ('%.4f 0 %.4f 0 -0.62 0' % (0, h * 1.06) if full else '0 0 %.4f 0 0 0' % (h * 0.91)),
            FLAT('0.13 0.13 0.14 1')),
        BOX('lid_c', '%.4f %.4f %.4f' % (w * 1.06, d * 1.06, h * 0.10),
            ('%.4f 0 %.4f 0 -0.62 0' % (0, h * 1.06) if full else '0 0 %.4f 0 0 0' % (h * 0.91)),
            col=True),
        BOX('handle', '%.4f 0.006 0.004' % (w * 0.45,), '0 0 %.4f 0 0 0' % (h * 0.97),
            FLAT('0.10 0.10 0.11 1')),
    ]
    if full:                                   # 满溢：桶口堆垃圾
        for i in range(3):
            body.append(BOX('trash%d' % i, '0.012 0.010 0.008',
                            '%.3f %.3f %.4f %.2f 0 %.2f'
                            % (0.008 * (i - 1), 0.005 * ((i + 1) % 2 - 0.5),
                               h * 0.88, 0.3 * i, 0.4 * i),
                            FLAT('0.58 0.50 0.32 1')))
    _write(model, body)
    return model


def build_ebike(toppled=False):
    """共享电单车：长 18 x 高 12 cm。前轮+前叉+车把+前篮+踏板+车身+黑座箱+后轮+脚撑。"""
    model = 'ebike_toppled' if toppled else 'ebike'
    L, W, H = EBIKE
    R = 0.022                                  # 轮半径（照片里轮径约车高的 1/3）
    TEAL = FLAT('0.36 0.80 0.76 1')
    DARK = FLAT('0.11 0.11 0.13 1')
    GREY = FLAT('0.55 0.56 0.58 1')

    def P(x, y, z, yaw=0.0, roll=0.0):
        """倒伏时绕 x 轴侧翻 90°：把 (y,z) 旋转并把整车抬到"躺着"的高度。"""
        if not toppled:
            return '%.4f %.4f %.4f 0 0 %.4f' % (x, y, z, yaw)
        return '%.4f %.4f %.4f %.4f %.4f 0' % (x, -z + W / 2 + 0.004, y + 0.010, roll, yaw)

    def b(nm, size, x, y, z, mat, yaw=0.0):
        return BOX(nm, size, P(x, y, z, yaw), mat)

    def c(nm, r, l, x, y, z, mat, yaw=0.0):
        return CYL(nm, r, l, P(x, y, z, yaw, math.pi / 2), mat)

    body = [
        # 车身主体（前踏板 → 后座箱），做成两段梯形感的盒子
        b('deck', '%.4f %.4f 0.020' % (L * 0.40, W * 0.78), -L * 0.05, 0, R * 1.05, GREY),
        b('body_f', '%.4f %.4f 0.048' % (L * 0.13, W * 0.62), L * 0.20, 0, H * 0.34, TEAL),
        b('body_r', '%.4f %.4f 0.052' % (L * 0.16, W * 0.70), -L * 0.20, 0, H * 0.40, TEAL),
        # 黑座箱（照片里最显眼的那块）
        b('seatbox', '%.4f %.4f 0.040' % (L * 0.26, W * 0.72), -L * 0.10, 0, H * 0.62, DARK),
        b('seat', '%.4f %.4f 0.010' % (L * 0.22, W * 0.68), -L * 0.12, 0, H * 0.72, DARK),
        # 车把立柱 + 车把 + 前篮
        b('stem', '0.012 %.4f %.4f' % (W * 0.30, H * 0.30), L * 0.32, 0, H * 0.62, TEAL),
        b('bar', '0.010 %.4f 0.010' % (W * 1.1), L * 0.34, 0, H * 0.76, DARK),
        b('basket', '%.4f %.4f 0.026' % (L * 0.11, W * 0.62), L * 0.40, 0, H * 0.66,
          FLAT('0.16 0.16 0.17 1')),
        # 挡泥板
        b('fender_f', '%.4f %.4f 0.006' % (R * 2.2, W * 0.5), L * 0.34, 0, R * 1.75, TEAL),
        b('fender_r', '%.4f %.4f 0.006' % (R * 2.2, W * 0.5), -L * 0.34, 0, R * 1.75, TEAL),
    ]
    for nm, x in (('wheel_f', L * 0.34), ('wheel_r', -L * 0.34)):
        body.append(c(nm, R, W * 0.16, x, 0, R, DARK))
        body.append(CYL('%s_c' % nm, R, W * 0.16, P(x, 0, R, 0, math.pi / 2), col=True))
    if not toppled:
        body.append(c('kick', 0.0035, 0.030, -L * 0.18, W * 0.45, 0.015, DARK))
    _write(model, body)
    return model


def build_helmet():
    """头盔 2.9 x 3.2 x 2 cm（球 + 帽檐）。"""
    model = 'helmet'
    w, d, h = HELMET
    body = [
        '<visual name="shell"><pose>0 0 %.4f 0 0 0</pose><geometry><sphere>'
        '<radius>%.4f</radius></sphere></geometry>%s</visual>'
        % (h * 0.52, d * 0.5, FLAT('0.36 0.80 0.76 1')),
        '<collision name="shell_c"><pose>0 0 %.4f 0 0 0</pose><geometry><sphere>'
        '<radius>%.4f</radius></sphere></geometry></collision>' % (h * 0.52, d * 0.5),
        BOX('brim', '%.4f 0.010 0.006' % (w * 0.9,), '%.4f 0 %.4f 0 0 0'
            % (-w * 0.12, h * 0.42), FLAT('0.20 0.20 0.22 1')),
    ]
    _write(model, body)
    return model


def build_gauge(dial=0.35):
    """仪表立牌（站房那项的替代）：立柱 + 表盘 + 底座 + 读数牌。"""
    model = 'gauge'
    w, d, h = GAUGE
    _script(model, [('Dial', tex_gauge(model, dial), '0.45 0.45 0.45 1')])
    body = [
        BOX('base', '%.4f %.4f 0.010' % (w * 0.5, d * 1.4), '0 0 0.005 0 0 0',
            FLAT('0.35 0.36 0.38 1')),
        BOX('base_c', '%.4f %.4f 0.010' % (w * 0.5, d * 1.4), '0 0 0.005 0 0 0', col=True),
        BOX('panel', '%.4f %.4f %.4f' % (w, d * 0.5, h * 0.46), '0 0 %.4f 0 0 0'
            % (h * 0.58), MAT(model, 'Dial')),
        BOX('panel_c', '%.4f %.4f %.4f' % (w, d * 0.5, h * 0.46), '0 0 %.4f 0 0 0'
            % (h * 0.58), col=True),
        BOX('post', '0.014 %.4f %.4f' % (d * 0.5, h * 0.40), '0 0 %.4f 0 0 0' % (h * 0.20),
            FLAT('0.48 0.49 0.52 1')),
        BOX('tag', '%.4f 0.004 0.024' % (w * 0.7,), '0 %.4f %.4f 0 0 0'
            % (d * 0.35, h * 0.30), FLAT('0.92 0.92 0.88 1')),
    ]
    _write(model, body)
    return model


def build_sign():
    """指示牌：立柱 + 圆牌。"""
    model = 'sign'
    w, t, h = SIGN
    body = [
        CYL('post', 0.006, h, '0 0 %.4f 0 0 0' % (h / 2), FLAT('0.45 0.46 0.48 1')),
        CYL('post_c', 0.006, h, '0 0 %.4f 0 0 0' % (h / 2), col=True),
        BOX('board', '%.4f %.4f 0.055' % (w, t), '0 0 %.4f 0 0 0' % (h - 0.045),
            FLAT('0.15 0.35 0.62 1')),
        BOX('board_c', '%.4f %.4f 0.055' % (w, t), '0 0 %.4f 0 0 0' % (h - 0.045), col=True),
        BOX('plate', '0.055 0.004 0.022', '0 -0.010 %.4f 0 0 0' % (h - 0.045),
            FLAT('0.95 0.95 0.92 1')),
    ]
    _write(model, body)
    return model


def build_heat(bad):
    """异常温度：管 + 面板（异常时自发光红）。"""
    model = 'heat_bad' if bad else 'heat_ok'
    w, t, h = HEATP
    _script(model, [('Panel', tex_heat(model, bad=bad),
                     '1 0.5 0.3 1' if bad else '0.45 0.55 0.45 1')])
    body = [
        CYL('pipe', 0.010, 0.30, '0 0 0.05 0 1.5708 0', FLAT('0.52 0.53 0.55 1')),
        CYL('pipe_c', 0.010, 0.30, '0 0 0.05 0 1.5708 0', col=True),
        BOX('panel', '%.4f %.4f %.4f' % (w, t, h), '0 0.020 %.4f 0 0 0' % 0.16,
            MAT(model, 'Panel')),
        BOX('panel_c', '%.4f %.4f %.4f' % (w, t, h), '0 0.020 %.4f 0 0 0' % 0.16, col=True),
    ]
    _write(model, body)
    return model


# ---------------------------------------------------------------------------
#  world 组装
# ---------------------------------------------------------------------------
def build_includes():
    out = [BEGIN]
    for p in PUT:
        out += ['  <include>', '    <uri>model://%s</uri>' % p['model'],
                '    <name>%s</name>' % p['name'],
                '    <pose>%.4f %.4f 0 0 0 %.4f</pose>' % (p['x'], p['y'], p['yaw']),
                '  </include>']
    out.append(END)
    return '\n'.join(out)


def build_world(remove=False):
    if not os.path.isfile(WORLD):
        sys.exit('找不到复赛 world: %s' % WORLD)
    s = io.open(WORLD, encoding='utf-8').read()
    s = re.sub(re.escape(BEGIN) + r'.*?' + re.escape(END) + r'\n?', '', s, flags=re.S)
    if not remove:
        s = s.replace('</world>', build_includes() + '\n</world>', 1)
    io.open(OUT_WORLD, 'w', encoding='utf-8').write(s)
    return OUT_WORLD


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--show', action='store_true', help='只列摆放清单')
    ap.add_argument('--remove', action='store_true', help='撤掉国赛物料')
    a = ap.parse_args()
    if a.show:
        print('  国赛物料摆放（共 %d 件）:' % len(PUT))
        for p in PUT:
            print('    %-12s %-14s (%+.2f, %+.2f) yaw=%.2f'
                  % (p['name'], p['model'], p['x'], p['y'], p['yaw']))
        return 0
    if a.remove:
        print('  已撤: %s' % build_world(remove=True))
        return 0

    made = [build_bldg(k) for k in BLDGS]
    for color, label, _rgb in BIN_COLORS:
        made.append(build_bin(color, label, full=(color == 'red')))
    made += [build_ebike(False), build_ebike(True), build_helmet(), build_gauge(),
             build_sign(), build_heat(True), build_heat(False)]
    print('  已生成 %d 个物料模型:' % len(made))
    for m in made:
        print('    %s' % m)
    print('  国赛 world -> %s' % os.path.relpath(build_world(), WS))
    print('  摆放 %d 件' % len(PUT))
    return 0


if __name__ == '__main__':
    sys.exit(main())
