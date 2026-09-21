#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""车辆 + 车牌: 从官方素材生成模型, 并摆进右下角三个停车位。

素材: ~/复赛资料/车辆识别/
    车牌背景.png   2432x1728  蓝色轿车**车尾视图**
    车牌一/二/三.png 200x81  三张车牌: 苏A·B8Q62 / 鄂D·7B5Q2 / 苏A·PL12A
    车与车牌尺寸.txt: 车背景 34.5 x 25 cm x 5 mm 厚; 车牌 9.5 x 3 cm

做法:
    * 车板: 34.5 x 25 cm x 5 mm 的板子, 正面贴车尾图 (模型板面朝 +x)
    * 车牌: 单独一个小板 9.5 x 3 cm, 贴在车板正面、后保险杠中间
       —— 用**独立视觉**而不是把车牌合成进车尾图: 官方车牌 PNG 只有 200x81 px
          (约 21 px/cm), 合成进 2432px 的车尾图要放大 3.35 倍会糊掉, 影响 OCR。
          独立贴片能保持原始像素密度。
    * 模型 <static>true</static>: 停着的车不会动, 静态模型最稳 (没有关节需求)

停车位 (build_arena 从图里提取, 右下角竖列 x[1.472, 2.076]):
    3号停车位  y[-0.805, -0.197]
    2号停车位  y[-1.423, -0.826]
    1号停车位  y[-2.076, -1.448]
    车头/车尾朝 -x (朝车道), 机器人沿右车道由下往上开时依次看到三块车牌

用法:
    python3 tools/setup_cars.py            # 生成模型 + 插进 world
    python3 tools/setup_cars.py --remove   # 从 world 移除
    python3 tools/setup_cars.py --show     # 只看清单
    python3 tools/setup_cars.py --models-only
"""
from __future__ import annotations

import argparse
import os
import re
import shutil
import sys

import yaml

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PKG = os.path.join(WS, 'src', 'competition_arena')
SRC = os.path.expanduser('~/复赛资料/车辆识别')
WORLD = os.path.join(PKG, 'worlds', 'competition_arena.world')
CFG = os.path.join(PKG, 'config', 'cars.yaml')
BEGIN = '    <!-- ===== 车辆+车牌 (tools/setup_cars.py 生成) ===== -->'
END = '    <!-- ===== 车辆+车牌结束 ===== -->'

# 官方尺寸 (车与车牌尺寸.txt)
BOARD_W, BOARD_H, BOARD_T = 0.345, 0.250, 0.005     # 车板 宽 x 高 x 厚
PLATE_W, PLATE_H, PLATE_T = 0.095, 0.030, 0.003     # 车牌 宽 x 高 x 厚
BASE_T = 0.006                                       # 底座厚
# 车牌贴在车板正面的位置 (相对车板左下角): 水平居中, 高度 4.8cm
#   官方车尾图没画车牌安装位, 按真车习惯放在后保险杠中间; 实测保险杠在
#   z 0.014~0.079 m, 车牌 3cm 高居中放在 0.048 刚好落在保险杠上
PLATE_CZ = 0.048

PARKING = [                                          # (车位名, y0, y1)  由 build_arena 提取
    ('3号停车位', -0.805, -0.197),
    ('2号停车位', -1.423, -0.826),
    ('1号停车位', -2.076, -1.448),
]
LANE_X0, LANE_X1 = 1.472, 2.076                      # 停车位列的 x 范围


def rgba_or_rgb(path):
    return 'RGBA' if path.endswith('.png') else 'RGB'


def write_car_model(name, plate_png, plate_text):
    d = os.path.join(PKG, 'models', name)
    tex = os.path.join(d, 'materials', 'textures')
    scr = os.path.join(d, 'materials', 'scripts')
    os.makedirs(tex, exist_ok=True)
    os.makedirs(scr, exist_ok=True)
    # 车尾图缩小到合理分辨率 (2432px 对 34.5cm 是 70px/cm, 太大; 保留 ~28px/cm 够用)
    from PIL import Image
    bg = Image.open(os.path.join(SRC, '车牌背景.png')).convert('RGB')
    bg = bg.resize((968, 688), Image.LANCZOS)          # 968/(0.345*100) = 28 px/cm
    bg.save(os.path.join(tex, 'car.png'))
    shutil.copyfile(plate_png, os.path.join(tex, 'plate.png'))

    with open(os.path.join(scr, 'car.material'), 'w') as f:
        f.write('''// 自动生成 (tools/setup_cars.py)
// ★ 材质名必须**每个模型都不同**: OGRE 的材质是全局按名字注册的, 三个模型
//   用同一个 Car/Plate 会互相覆盖, 结果三辆车显示同一张车牌 (踩过)。
material Car/Body_%(name)s
{
  technique { pass {
    ambient 1 1 1 1
    diffuse 1 1 1 1
    specular 0 0 0 0 0
    texture_unit { texture car.png filtering anisotropic max_anisotropy 8 }
  } }
}
material Car/Plate_%(name)s
{
  technique { pass {
    ambient 1 1 1 1
    diffuse 1 1 1 1
    specular 0 0 0 0 0
    alpha_rejection greater_equal 128
    texture_unit { texture plate.png filtering anisotropic max_anisotropy 8 }
  } }
}
''' % dict(name=name))

    hz = BASE_T + BOARD_H / 2.0
    sdf = '''<?xml version="1.0"?>
<sdf version="1.7">
  <model name="{name}">
    <!-- 车辆立牌 + 车牌 ({plate})
         车板 {bw:.3f} x {bh:.3f} x {bt:.3f} m (官方 34.5 x 25 cm x 5mm)
         车牌 {pw:.3f} x {ph:.3f} m (官方 9.5 x 3 cm), 贴在正面后保险杠中间 -->
    <static>true</static>
    <link name="link">
      <visual name="body">
        <pose>0 0 {hz:.4f} 0 0 0</pose>
        <geometry><box><size>{bt:.4f} {bw:.4f} {bh:.4f}</size></box></geometry>
        <material><script>
          <uri>model://{name}/materials/scripts</uri>
          <uri>model://{name}/materials/textures</uri>
          <name>Car/Body_{name}</name>
        </script></material>
      </visual>
      <visual name="plate">
        <pose>{px:.4f} 0 {pz:.4f} 0 0 0</pose>
        <geometry><box><size>{pt:.4f} {pw:.4f} {ph:.4f}</size></box></geometry>
        <material><script>
          <uri>model://{name}/materials/scripts</uri>
          <uri>model://{name}/materials/textures</uri>
          <name>Car/Plate_{name}</name>
        </script></material>
      </visual>
      <visual name="base">
        <pose>0 0 {bz:.4f} 0 0 0</pose>
        <geometry><box><size>0.060 {bw:.4f} {bt2:.4f}</size></box></geometry>
        <material><ambient>0.85 0.85 0.85 1</ambient><diffuse>0.85 0.85 0.85 1</diffuse></material>
      </visual>
      <collision name="col">
        <pose>0 0 {hz:.4f} 0 0 0</pose>
        <geometry><box><size>{bt:.4f} {bw:.4f} {bh:.4f}</size></box></geometry>
      </collision>
      <collision name="base_c">
        <pose>0 0 {bz:.4f} 0 0 0</pose>
        <geometry><box><size>0.060 {bw:.4f} {bt2:.4f}</size></box></geometry>
      </collision>
    </link>
  </model>
</sdf>
'''.format(name=name, plate=plate_text, bw=BOARD_W, bh=BOARD_H, bt=BOARD_T,
           pw=PLATE_W, ph=PLATE_H, pt=PLATE_T, hz=hz,
           px=BOARD_T / 2.0 + PLATE_T / 2.0, pz=BASE_T + PLATE_CZ,
           bz=BASE_T / 2.0, bt2=BASE_T)
    open(os.path.join(d, 'model.sdf'), 'w').write(sdf)
    open(os.path.join(d, 'model.config'), 'w').write(
        '<?xml version="1.0"?>\n<model>\n  <name>%s</name>\n  <version>1.0</version>\n'
        '  <sdf version="1.7">model.sdf</sdf>\n'
        '  <description>车辆立牌+车牌 %s</description>\n</model>\n' % (name, plate_text))
    return d


def build_includes(plan):
    out = [BEGIN]
    for c in plan:
        out.append('    <include>')
        out.append('      <uri>model://%s</uri>' % c['model'])
        out.append('      <name>%s</name>' % c['name'])
        # 车尾朝 -x (朝车道) -> yaw = 180 度
        out.append('      <pose>%.4f %.4f 0 0 0 %.4f</pose>'
                   % (c['x'], c['y'], 3.141592653589793))
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


PLATES = [('car_1', '车牌一.png', '苏A·B8Q62'),
          ('car_2', '车牌二.png', '鄂D·7B5Q2'),
          ('car_3', '车牌三.png', '苏A·PL12A')]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--remove', action='store_true')
    ap.add_argument('--show', action='store_true')
    ap.add_argument('--models-only', dest='models_only', action='store_true')
    a = ap.parse_args()

    if not os.path.isdir(SRC):
        sys.exit('找不到官方素材: %s' % SRC)

    # 车牌 N -> N 号停车位; 车位里车板朝 -x, 靠外侧(墙侧)放
    plan = []
    for i, ((model, png, text), (pname, y0, y1)) in enumerate(zip(PLATES, PARKING), 1):
        cy = (y0 + y1) / 2.0
        plan.append(dict(name='%s_%s' % (model, 'p'), model=model, plate=text,
                         parking=pname, x=LANE_X1 - 0.02, y=round(cy, 4)))
        if not a.show:
            write_car_model(model, os.path.join(SRC, png), text)

    if a.show:
        print('  %-8s %-12s %-12s %s' % ('模型', '车牌', '停车位', '位置 (x, y)'))
        for c in plan:
            print('  %-8s %-12s %-12s (%+.3f, %+.3f)  车尾朝 -x' % (c['model'], c['plate'], c['parking'], c['x'], c['y']))
        return

    if not a.models_only:
        # 先清掉旧的车, 再写配置和 world
        patch_world([], remove=True)
        patch_world(plan, remove=False)
        with open(CFG, 'w') as f:
            f.write('# 车辆 + 车牌 (唯一真值源) —— 由 tools/setup_cars.py 生成\n'
                    '#\n'
                    '# 素材: ~/复赛资料/车辆识别/ (车牌背景 = 车尾视图; 车牌一/二/三)\n'
                    '# 尺寸: 车板 34.5 x 25 cm x 5mm, 车牌 9.5 x 3 cm (官方 txt)\n'
                    '# 车牌贴在车板正面后保险杠中间 (官方车尾图没画安装位, 按真车习惯)\n'
                    '#\n'
                    '# 站位: 右下角三个停车位 (x[%.3f, %.3f]), 车尾朝 -x 朝车道\n'
                    % (LANE_X0, LANE_X1))
            yaml.safe_dump({'cars': plan}, f, allow_unicode=True, default_flow_style=False, sort_keys=False)

    print('已生成 %d 辆车:' % len(plan))
    for c in plan:
        print('  %-8s %-12s -> %s @ (%+.3f, %+.3f)' % (c['model'], c['plate'], c['parking'], c['x'], c['y']))
    if not a.models_only:
        print('  已插进 world (车尾朝 -x, 机器人沿右车道由下往上依次看到三块车牌)')
    print('  配置 -> %s' % os.path.relpath(CFG, WS))


if __name__ == '__main__':
    main()
