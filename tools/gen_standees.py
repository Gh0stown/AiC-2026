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

STANDEE_H = 0.150      # 立牌高 15 cm
THICK = 0.003          # 厚 3 mm
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
    dict(name='A', inset_m=0.26, rect=[-1.472, 0.847, -0.452, 1.468], edges=[
        dict(edge='north', people=['standee_c01', 'standee_c02']),
        dict(edge='south', people=['standee_c03', 'standee_c04']),
        dict(edge='west',  people=['standee_c05', 'standee_F1']),
    ]),
    dict(name='B', inset_m=0.30, rect=[-1.475, -0.537, -0.407, 0.197], edges=[
        dict(edge='north', people=['standee_c06', 'standee_c07']),
        dict(edge='east',  people=['standee_c08', 'standee_F2']),
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

    shutil.copyfile(src_png, os.path.join(tex_d, '%s.png' % name))

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
    <!-- 人偶立牌: 高 {h:.3f} m x 宽 {w:.3f} m x 厚 {t:.3f} m
         尺寸来源: ~/复赛资料/人员/ (用户确认 15x5 cm) -->
    <static>true</static>
    <link name="link">
      <visual name="board">
        <pose>0 0 {zb:.4f} 0 0 0</pose>
        <geometry><box><size>{t:.4f} {w:.4f} {h:.4f}</size></box></geometry>
        <material>
          <script>
            <uri>model://{name}/materials/scripts</uri>
            <uri>model://{name}/materials/textures</uri>
            <name>{mat}</name>
          </script>
        </material>
      </visual>
      <!-- ★ 背面挡板 (放 +x 面 = 透明的那面): Gazebo 的 box 六个面共用一张贴图,
           实拍发现从背后看会**看穿** (-x 面渲染人物, +x 面 alpha 被丢弃)。
           所以: 人物图在 **-x** 面 -> yaw 要让 -x 朝车道/街区外 (见 setup_standees.py
           的 EDGE 表, 已按"整体 +180°"修正); 挡板放 **+x** 挡住背面。
           ⚠ 这块挡板/朝向的组合是**实拍标定**出来的, 推导会推错 (来回折腾过三次)。
             核对: 用 launch 参数把车生成在点位上抓帧 (交接文档 §4.6), 和
             ~/复赛资料/人员/ 的原图对: 车道侧要看到人物正面
             (c01=戴草帽拿修枝剪的园艺工, c02=抱书的眼镜男)。 -->
      <visual name="back">
        <pose>{xb:.4f} 0 {zb:.4f} 0 0 0</pose>
        <geometry><box><size>0.0010 {wb:.4f} {hb:.4f}</size></box></geometry>
        <material><ambient>0.90 0.89 0.86 1</ambient><diffuse>0.90 0.89 0.86 1</diffuse></material>
      </visual>
      <visual name="base">
        <pose>0 0 {zbh:.4f} 0 0 0</pose>
        <geometry><box><size>{bx:.4f} {w:.4f} {bh:.4f}</size></box></geometry>
        <material><ambient>0.92 0.92 0.92 1</ambient><diffuse>0.92 0.92 0.92 1</diffuse></material>
      </visual>
      <collision name="col">
        <pose>0 0 {zb:.4f} 0 0 0</pose>
        <geometry><box><size>{t:.4f} {w:.4f} {h:.4f}</size></box></geometry>
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
           # 背面挡板: 贴板背面 (板厚 t, 面在 +-t/2), 往外让 0.5mm 防 z-fighting,
           # 并比板略大 2mm, 保证从背后看完全遮住贴图
           xb=(THICK / 2.0 + 0.0010), wb=w + 0.002, hb=h + 0.002)
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
        w = h * pw / float(ph)
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
                    '# 尺寸: 高 %.3f m, 宽按各图长宽比 (用户确认实物 15x5 cm)\n'
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
