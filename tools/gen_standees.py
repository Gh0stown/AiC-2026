#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""从官方人员 PNG 生成人偶立牌模型。

素材: ~/复赛资料/人员/
    社区人员/1..16.png      —— 社区人员 (农民/白领/厨师/外卖员/医生/建筑工 ...)
    非社区人员/F1.png F2.png —— 非社区人员 (可疑人物)
    人偶长宽.jpg            —— 实物照(带卷尺), 用于核对尺寸

尺寸 (用户确认 + 照片核对):
    高 15 cm, 宽约 5 cm —— 宽度按各图自身长宽比算 (PNG 长宽比约 0.34 -> 5.1cm)
    厚度 3 mm (实物是薄的模切立牌)

做法:
    * 每张图先按 alpha 裁掉空白边, 保证"高 15cm"量的是人物本体
    * 每个立牌一个模型目录, 自带 materials/(PNG + OGRE 材质脚本), 自包含
    * 材质用 **alpha_rejection** 把透明区直接裁掉 —— 比 alpha 混合干净,
      不会有半透明排序问题, 立牌边缘就是人物轮廓
    * 模型设 <static>true</static>: 立牌是不会动的道具, 静态模型既不会倒、
      也不会被碰到乱飞 (红绿灯是因为要动关节才不能用 static, 立牌没这需求)

用法:
    python3 tools/gen_standees.py              # 生成全部立牌模型
    python3 tools/gen_standees.py --list       # 只打印清单和尺寸
生成后模型在 src/competition_arena/models/standee_*/ , world 里**暂不摆放**。
"""
from __future__ import annotations

import argparse
import os
import shutil
import sys

import yaml
from PIL import Image

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PKG = os.path.join(WS, 'src', 'competition_arena')
SRC = os.path.expanduser('~/复赛资料/人员')
CFG = os.path.join(PKG, 'config', 'standees.yaml')

STANDEE_H = 0.150      # 立牌高 15 cm (官方)
THICK = 0.005          # 厚 5 mm (官方)
# 官方"宽 5 cm"是**外接尺寸** (实物按人物轮廓模切, 各图轮廓比不同)。
#   * 可见板面 = 人物外接框 (零留白, 和实物一致)
#   * 碰撞体 / 雷达反射面 = 官方统一 15 x 5 x 0.5 cm
OFFICIAL_W, OFFICIAL_H = 0.050, 0.150
BASE_H = 0.004         # 底座厚 4 mm (实物立牌下面有个折起来的支撑)
BASE_X = 0.045         # 底座前后伸出 (防止前后倒)

# =============================================================================
#  街区摆放计划
#  依据: 复赛任务示意图 —— A 街区人偶朝 北/南/西 (箭头 ←↓↑), B 街区朝 北/东 (箭头 ↑→)
#        用户要求 —— 每个方向都要有人; 两个非社区人员 F1/F2 必须都摆、不重复, A/B 各一个
#
#  街区矩形: 白线是**车道线**, 街区是车道之间的区域 (由 build_arena 提取)
#      A  x[-1.472, -0.452] y[0.847, 1.468]  1.02 x 0.62 m  (高 62cm ~ 示意图标注 60cm)
#         北面=顶车道, 南面=中车道, 西面=左车道   (东面那条竖车道示意图上没人偶)
#         ★ 2026-09-21 收窄: 原来我按"顶车道和中车道之间的整条横带"取
#           (x 到 +0.843, 2.32m 宽), 但用户标注图指出 A 街区只是左边这一小块
#           (红框 1.03 x 0.70m), 右边那截是别的楼宇。西/北/南三条边贴白线,
#           东边界取用户红框的 x=-0.452 (图上那条分界没画成白线)。
#      B  x[-1.475, -0.407] y[-0.537, 0.197] 1.07 x 0.73 m
#         北面=中车道, 东面=竖车道
# =============================================================================
# ★ 每个街区往内缩多少 (m) —— 直接决定"正对着拍能拍到多少身体":
#     可见高度 ≈ 0.366*d - 0.046  (d = 相机到立牌距离)
#     d=0.435(缩0.20) -> 75%   d=0.535(缩0.26) -> 100%(刚拍全)
#   A 街区深 0.62m, 南北两组各缩 0.26 后还剩 0.10m 间距 (底座不打架);
#   B 街区深 0.73m, 可以缩到 0.30。
BLOCKS = [
    # inset_m: 该街区立牌往边线内缩多少 (直接决定正对拍摄能拍到多少身体)
    # ★ 2026-10-09 官方新规则: 每次摆场 **>= 14 个社区人员 + 2 个非社区人员 (=16)**,
    #   且不允许"一个街区只有 1 个人偶"。本布局 16 个: A 10 (9 社区 + F1) / B 6 (5 社区 + F2)。
    #   各边人数与可用长度 (同组间距 0.090 m, 立牌宽 0.050 m -> 净间隙 4 cm):
    #     A north 5 (span 0.36 <= 0.500)  A south 4 (0.27 <= 0.500)  A west 1 (居中)
    #     B north 4 (span 0.27 <= 0.468)  B east  2 (0.09 <= 0.134)
    #   ★ 为什么把多的人放 A 而不是 B: B 只有 north/east 两条边, 若 north 放 5 个,
    #     它靠东那一端与 east 边那组最近只剩 29 mm (setup_standees 的跨组干涉检查报警)。
    #     A 有 north/south/west 三条边, 多出来的 2 个放 north 不会挤到角上。
    dict(name='A', inset_m=0.26, rect=[-1.472, 0.847, -0.452, 1.468], edges=[
        dict(edge='north', people=['standee_c01', 'standee_c02', 'standee_c03',
                                   'standee_c04', 'standee_c05']),
        dict(edge='south', people=['standee_c06', 'standee_c07', 'standee_c08',
                                   'standee_c09']),
        dict(edge='west',  people=['standee_F1']),
    ]),
    dict(name='B', inset_m=0.30, rect=[-1.475, -0.537, -0.407, 0.197], edges=[
        dict(edge='north', people=['standee_c10', 'standee_c11', 'standee_c12',
                                   'standee_c13']),
        dict(edge='east',  people=['standee_c14', 'standee_F2']),
    ]),
]


def collect():
    """扫描素材目录, 返回 [(模型名, 源文件, 类别, 原图名)]"""
    out = []
    for i in range(1, 17):
        p = os.path.join(SRC, '社区人员', '%d.png' % i)
        if os.path.isfile(p):
            out.append(('standee_c%02d' % i, p, 'community', '%d.png' % i))
    for tag in ('F1', 'F2'):
        p = os.path.join(SRC, '非社区人员', '%s.png' % tag)
        if os.path.isfile(p):
            out.append(('standee_%s' % tag, p, 'non_community', '%s.png' % tag))
    return out


def trim(im):
    """按 alpha 裁掉四周空白, 返回 (裁剪后图, 宽, 高)"""
    im = im.convert('RGBA')
    bbox = im.split()[-1].getbbox()
    if bbox:
        im = im.crop(bbox)
    return im, im.size[0], im.size[1]


def write_model(name, src_png, size_m, mat_name):
    w, h = size_m
    d = os.path.join(PKG, 'models', name)
    tex_d = os.path.join(d, 'materials', 'textures')
    scr_d = os.path.join(d, 'materials', 'scripts')
    os.makedirs(tex_d, exist_ok=True)
    os.makedirs(scr_d, exist_ok=True)

    # ★ 官方尺寸 5x15 cm: 把人物按 contain 居中放进 5:15 画布, 四边留透明
    #   (alpha_rejection 会把透明像素丢掉, 所以板面边距是透明的; 后面那块白底板
    #    就是"板子本体", 于是视觉上仍是一块白色立牌, 尺寸与雷达反射面与官方一致)。
    art = Image.open(src_png).convert('RGBA')
    bb = art.split()[-1].getbbox()
    if bb:
        art = art.crop(bb)
    # 画布比例与人物比例一致 -> 贴图正好铺满板面, 四周零留白
    cw = max(2, int(round(w / h * 600.0))); ch = 600
    art = art.resize((cw, ch), Image.LANCZOS)
    canvas = Image.new('RGBA', (cw, ch), (0, 0, 0, 0))
    canvas.paste(art, ((cw - art.width) // 2, ch - art.height), art)   # 脚踩板底
    canvas.save(os.path.join(tex_d, '%s.png' % name))

    # OGRE 材质: alpha_rejection 直接把透明像素丢掉, 立牌边缘就是人物轮廓
    # ★ 材质脚本的**文件名也必须每个模型唯一**: OGRE 把资源按"文件名"全局注册,
    #   18 个模型都叫 standee.material 时只有一个能加载, 其余的材质名找不到 ->
    #   对应立牌在相机里直接**不渲染** (实测: c01/c02 从头到尾没出现过,
    #   而 A_north 点位拍到的其实是后面 c03/c04 那对)。和 §7.1 车辆那次同源。
    with open(os.path.join(scr_d, '%s.material' % name), 'w') as f:
        f.write('''// 自动生成 (tools/gen_standees.py) —— 人物立牌
material %s
{
  technique
  {
    pass
    {
      ambient 1 1 1 1
      diffuse 1 1 1 1
      specular 0 0 0 0 0
      alpha_rejection greater_equal 128
      texture_unit
      {
        texture %s.png
        filtering anisotropic
        max_anisotropy 8
      }
    }
  }
}
''' % (mat_name, name))

    # 模型原点在**地面**, 水平居中; 板面朝 +x
    z_board = BASE_H + h / 2.0
    sdf = '''<?xml version="1.0"?>
<sdf version="1.7">
  <model name="{name}">
    <!-- 人偶立牌: 高 {h:.3f} m x 宽 {w:.3f} m x 厚 {t:.3f} m (板面尺寸 = 人物轮廓外接框)
         尺寸来源: ~/复赛资料/人员/ —— 官方实物是**按人物轮廓模切**的板子,
         标称 15 x 5 cm 是外接尺寸; 各张素材的轮廓比不同 (15 cm 高时宽 3.9~7.0 cm)。

         ★ 结构 (2026-10-09 改两次后的最终形态):
           board = 尺寸正确、纯白的板材, 正面尺寸**正好等于人物外接框 -> 零留白**
           art   = 板前再贴一层 1.2 mm 的贴图面, 只有这一层带人物图
           col   = **碰撞体按官方 15 x 5 x 0.5 cm 的统一外接尺寸** (雷达反射面也用它)
           为什么这么拆: ① Gazebo 的 box 六个面共用一张贴图, 直接把图贴在 5mm 厚的
           板上 -> 侧面与顶面也印着人物 (5mm 侧面上一条压扁的人);
           ② 若把 5cm 宽当作板面, 窄轮廓的人就会左右留白。
           所以"可见板面 = 人物外接框 (零留白)", "碰撞/雷达 = 官方 5 cm 外接框"。
         朝向: 人物图在 **-x** 面 (实物标定结论, 见 setup_standees.py 的 EDGE 表) -->
    <static>true</static>
    <link name="link">
      <visual name="board">
        <pose>0 0 {zb:.4f} 0 0 0</pose>
        <geometry><box><size>{t:.4f} {w:.4f} {h:.4f}</size></box></geometry>
        <material><ambient>0.94 0.94 0.93 1</ambient><diffuse>0.94 0.94 0.93 1</diffuse></material>
      </visual>
      <visual name="art">
        <pose>{xa:.4f} 0 {zb:.4f} 0 0 0</pose>
        <geometry><box><size>0.0012 {w:.4f} {h:.4f}</size></box></geometry>
        <material>
          <script>
            <uri>model://{name}/materials/scripts</uri>
            <uri>model://{name}/materials/textures</uri>
            <name>{mat}</name>
          </script>
        </material>
      </visual>
      <visual name="base">
        <pose>0 0 {zbh:.4f} 0 0 0</pose>
        <geometry><box><size>{bx:.4f} {w:.4f} {bh:.4f}</size></box></geometry>
        <material><ambient>0.92 0.92 0.92 1</ambient><diffuse>0.92 0.92 0.92 1</diffuse></material>
      </visual>
      <collision name="col">
        <pose>0 0 {zb:.4f} 0 0 0</pose>
        <geometry><box><size>{t:.4f} {cw:.4f} {ch:.4f}</size></box></geometry>
      </collision>
      <collision name="base_c">
        <pose>0 0 {zbh:.4f} 0 0 0</pose>
        <geometry><box><size>{bx:.4f} {w:.4f} {bh:.4f}</size></box></geometry>
      </collision>
    </link>
  </model>
</sdf>
'''.format(name=name, h=h, w=w, t=THICK, zb=z_board, mat=mat_name,
           bx=BASE_X, bh=BASE_H, zbh=BASE_H / 2.0,
           # 贴图面贴在板前 -x 侧, 让 0.6mm 防 z-fighting
           xa=-(THICK / 2.0 + 0.0012 / 2.0),
           # 碰撞体: 官方统一外接尺寸 15 x 5 x 0.5 cm
           cw=OFFICIAL_W, ch=OFFICIAL_H)
    with open(os.path.join(d, 'model.sdf'), 'w') as f:
        f.write(sdf)
    with open(os.path.join(d, 'model.config'), 'w') as f:
        f.write('<?xml version="1.0"?>\n<model>\n  <name>%s</name>\n  <version>1.0</version>\n'
                '  <sdf version="1.7">model.sdf</sdf>\n'
                '  <description>人偶立牌 (官方素材, 高 15cm)</description>\n</model>\n' % name)
    return d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--height', type=float, default=STANDEE_H, help='立牌高 (m), 默认 0.15')
    ap.add_argument('--list', action='store_true', help='只打印清单')
    ap.add_argument('--uniform-w', action='store_true',
                    help='把板面统一成 5 cm 宽 (会在窄轮廓的人两侧留白; 默认按轮廓)')
    a = ap.parse_args()

    if not os.path.isdir(SRC):
        sys.exit('找不到官方素材: %s' % SRC)

    items = collect()
    print('官方人员素材: %d 个立牌 (社区 %d, 非社区 %d)'
          % (len(items), sum(1 for x in items if x[2] == 'community'),
             sum(1 for x in items if x[2] == 'non_community')))
    print()
    print('%-14s %-10s %-16s %-12s %s' % ('模型名', '类别', '原图', '原图像素', '立牌尺寸 (宽x高 m)'))
    print('-' * 78)

    manifest = []
    for name, src, cat, orig in items:
        im = Image.open(src)
        trimmed, pw, ph = trim(im)
        h = a.height
        # ★ 官方统一尺寸: 高 15 cm、宽 5 cm。素材各图轮廓比不同 (15cm 高时 39~70 mm),
        #   所以板面一律 5x15, 人物按 contain 居中放进去 (四边留透明) —— 见 write_model。
        w = OFFICIAL_W if a.uniform_w else h * pw / float(ph)   # 默认: 人物外接框
        mat = 'Standee/%s' % name.replace('standee_', '')
        print('%-14s %-10s %-16s %-12s %.3f x %.3f'
              % (name, '社区' if cat == 'community' else '非社区', orig,
                 '%dx%d' % (pw, ph), w, h))
        if not a.list:
            tmp = os.path.join('/tmp', '%s.png' % name)
            trimmed.save(tmp)
            write_model(name, tmp, (w, h), mat)
            os.remove(tmp)
        manifest.append(dict(name=name, model=name, category=cat,
                             source=orig, width_m=round(w, 4), height_m=h,
                             material=mat))

    if not a.list:
        with open(CFG, 'w') as f:
            f.write('# 人偶立牌清单 (唯一真值源) —— 由 tools/gen_standees.py 生成\n'
                    '#\n'
                    '# 素材: ~/复赛资料/人员/  (社区人员 1~16, 非社区人员 F1/F2)\n'
                    '# 尺寸: 高 %.3f m, 板面宽按人物轮廓 (零留白); 碰撞体 = 官方 15x5x0.5 cm\n'
                    '#\n'
                    '# 摆放: 用 tools/setup_standees.py 按下面的 blocks 计划生成 <include>\n'
                    '#   朝向: 模型板面朝 +x -> yaw 决定面向 (北=+y 90, 南=-y -90, 东=+x 0, 西=-x 180)\n\n'
                    % h)
            # ★ inset_m: 立牌从街区边线往内缩多少。改成 0.20 (原来 0.032) 的原因:
            #   识别是"开到点位正对着拍", 相机只有 0.20m 高且无俯仰, 站得越近切得越多:
            #   拍摄距离 d -> 可见高度 ≈ 0.366*d - 0.046。往里挪 0.20m 后 d 从
            #   0.28m 涨到 ~0.44m, 可见比例 38% -> 75%, 需要的俯仰 17° -> 5°。
            #   (工具: setup_standees.py 读它; 计算见 gen_recognition_points.py)
            yaml.safe_dump({'standees': manifest, 'blocks': BLOCKS},
                           f, allow_unicode=True,
                           default_flow_style=False, sort_keys=False)
        print()
        print('  已生成 %d 个模型 -> src/competition_arena/models/standee_*/' % len(items))
        print('  清单 -> %s' % os.path.relpath(CFG, WS))
        print('  (按用户要求, **不摆进 world**)')


if __name__ == '__main__':
    main()
