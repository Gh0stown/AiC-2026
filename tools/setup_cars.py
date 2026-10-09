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
# 车牌贴在车板正面的位置 (相对车板左下角)
#   ★ 位置由用户标注决定: 用户在车尾图上画了红框 (手画, 12.54 x 4.94 cm,
#     中心离车板底 11.65 cm) —— 按实车看车牌是装在后**备箱**上, 不是保险杠。
#     车牌 9.5x3 cm 居中贴在红框中心。
#     (一开始我按真车习惯放在后保险杠中间, 是错的; 官方车尾图上确实不明显)
PLATE_CZ = 0.1165
PLATE_CX = 0.0          # 水平偏移 (红框中心实测偏 -0.28cm, 可忽略)

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
    # 先清空: 旧的 car.png / plate.png 残留会继续占着 OGRE 的全局贴图名
    if os.path.isdir(tex):
        shutil.rmtree(tex)
    os.makedirs(tex, exist_ok=True)
    os.makedirs(scr, exist_ok=True)
    # 车尾图缩小到合理分辨率 (2432px 对 34.5cm 是 70px/cm, 太大; 保留 ~28px/cm 够用)
    from PIL import Image
    bg = Image.open(os.path.join(SRC, '车牌背景.png')).convert('RGB')
    bg = bg.resize((968, 688), Image.LANCZOS)          # 968/(0.345*100) = 28 px/cm
    # ★ 贴图文件名也必须**每个模型都不同**: OGRE 的**贴图**和材质一样是按名字
    #   全局注册的。三个模型都叫 car.png / plate.png 的话, 第一个加载的会被后面
    #   复用 —— 结果三辆车显示同一张车牌。(材质名唯一还不够!)
    bg.save(os.path.join(tex, '%s_car.png' % name))
    shutil.copyfile(plate_png, os.path.join(tex, '%s_plate.png' % name))

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
    texture_unit { texture %(name)s_car.png filtering anisotropic max_anisotropy 8 }
  } }
}
material Car/Plate_%(name)s
{
  technique { pass {
    ambient 1 1 1 1
    diffuse 1 1 1 1
    specular 0 0 0 0 0
    alpha_rejection greater_equal 128
    texture_unit { texture %(name)s_plate.png filtering anisotropic max_anisotropy 8 }
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
        <pose>{px:.4f} {pcx:.4f} {pz:.4f} 0 0 0</pose>
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
           px=BOARD_T / 2.0 + PLATE_T / 2.0, pz=BASE_T + PLATE_CZ, pcx=PLATE_CX,
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


# =============================================================================
#  随机车牌生成 (2026-10-09)
#  官方要求: 每次摆场**至少两块随机车牌**, 且不要直接用复赛资料里给的那三张
#  (那三张是固定的, 谁都能提前背下来 -> 等于没有识别难度)。
#  规则依据: 公安部号牌正则 (见 https://blog.csdn.net/lzl640/article/details/125312808)
#      普通汽车(蓝牌): [省简称][A-Z][A-HJ-NP-Z0-9]{5}
#      即: 省份简称 1 位 + 发牌机关字母 1 位 + 序号 5 位 (数字/字母混合)
#      **字母与数字都排除 I 和 O** (避免与 1/0 混淆)。
#  生成一次即固定: 号码写进 config/cars.yaml (唯一真值源), 每次启动不再变
#  (验收脚本 / 识别核对都按 cars.yaml 比对); 同 --seed 重跑得到同一组号牌。
# =============================================================================
PROVINCES = '京津沪渝冀豫云辽黑湘皖鲁新苏浙赣鄂桂甘晋蒙陕吉闽贵粤青藏川宁琼'
PLATE_LETTERS = 'ABCDEFGHJKLMNPQRSTUVWXYZ'          # 发牌机关代号 (去 I/O)
PLATE_CHARS = 'ABCDEFGHJKLMNPQRSTUVWXYZ0123456789'  # 序号可用字符 (去 I/O)

PLATE_BG = (44, 76, 177)        # 官方蓝底 (取自 车牌背景.png 的中心像素)
PLATE_FG = (255, 255, 255)
PLATE_BORDER = 3                # 白边宽 (px)
PLATE_PX = (200, 81)            # 与官方 PNG 同尺寸 (9.5 x 3 cm)


def gen_plate(rng):
    """生成一块合规的蓝牌号: 省简称 + 字母 + '·' + 5 位"""
    return '%s%s·%s' % (rng.choice(PROVINCES), rng.choice(PLATE_LETTERS),
                        ''.join(rng.choice(PLATE_CHARS) for _ in range(5)))


def render_plate(text, out_png, size=(400, 127)):
    """按官方样式画蓝牌: 号牌底板 + 号牌字体(ChePai) + 省份汉字。

    素材见 config/plate/README.md。做法与真实号牌一致:
      * 底图 = 蓝底白框 + 安装孔 + 中间防伪金点(就是号牌上的分隔点)
      * 省份汉字 = 中文字体 (号牌字体只有字母数字, 与实物同理)
      * 其余字符 = 号牌专用字体 ChePai
    布局按底图上金点的位置分左右两段 (金点约在 31.7% 处): 左 2 字(省+字母), 右 5 位。
    """
    from PIL import Image, ImageDraw, ImageFont
    W, H = size
    base = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..',
                        'src', 'competition_arena', 'config', 'plate')
    tpl = os.path.join(base, 'blue_template.png')
    im = Image.open(tpl).convert('RGB').resize((W, H), Image.LANCZOS) if os.path.isfile(tpl) \
        else Image.new('RGB', (W, H), PLATE_BG)
    d = ImageDraw.Draw(im)

    def plate_font(px):
        fp = os.path.join(base, 'platechar.ttf')
        if os.path.isfile(fp):
            return ImageFont.truetype(fp, px)
        return cjk_font(px)

    def cjk_font(px):
        for fp in ('/mnt/c/Windows/Fonts/msyhbd.ttc', '/mnt/c/Windows/Fonts/simhei.ttf',
                   '/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf'):
            if os.path.isfile(fp):
                try:
                    return ImageFont.truetype(fp, px)
                except Exception:                               # noqa: BLE001
                    pass
        return ImageFont.load_default()

    head, serial = text.split('·')                     # 省 + 字母 | 5 位
    dot_x = int(W * 0.317)                             # 底图上防伪金点的位置
    f_cjk = int(H * 0.54)          # 省份汉字略小于号牌字体的字高, 与实物接近
    f_big = int(H * 0.62)

    def spread(chars, x0, x1, font_of, top, height):
        """在 [x0,x1] 区间内均匀排开几个字 (逐字居中)"""
        n = len(chars)
        step = (x1 - x0) / float(n)
        for i, ch in enumerate(chars):
            f = font_of(ch)
            d.text((x0 + step * (i + 0.5), top + height / 2.0), ch, font=f,
                   fill=PLATE_FG, anchor='mm')

    spread(head, int(W * 0.045), dot_x - int(W * 0.02),
           lambda ch: cjk_font(f_cjk) if ord(ch) > 0x2000 else plate_font(f_big),
           int(H * 0.17), int(H * 0.66))
    spread(serial, dot_x + int(W * 0.03), int(W * 0.965),
           lambda ch: plate_font(f_big), int(H * 0.17), int(H * 0.66))
    im.save(out_png)
    return out_png


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--remove', action='store_true')
    ap.add_argument('--show', action='store_true')
    ap.add_argument('--models-only', dest='models_only', action='store_true')
    ap.add_argument('--random-plates', type=int, default=3, dest='random_plates',
                    help='随机生成几块车牌 (默认 3 = 全部自造; 官方要求至少 2 块随机)')
    ap.add_argument('--seed', type=int, default=20261009, help='随机种子 (同种子得到同一组号牌)')
    ap.add_argument('--official', action='store_true',
                    help='用官方素材里的三张固定车牌图 (默认不用, 改为自己生成)')
    a = ap.parse_args()

    if not os.path.isdir(SRC):
        sys.exit('找不到官方素材: %s' % SRC)

    # ★ 号牌: 默认自己生成 (官方要求至少 2 块随机), 生成结果写进 cars.yaml 当真相
    import random
    rng = random.Random(a.seed)
    plates = []
    for i, (model, png, text) in enumerate(PLATES):
        use_random = (not a.official) and i < max(0, min(a.random_plates, len(PLATES)))
        plates.append((model, png, gen_plate(rng) if use_random else text,
                       use_random))
    print('  号牌 (seed=%d):' % a.seed)
    for model, png, text, rnd in plates:
        print('    %-8s %-12s %s' % (model, text, '自造(随机)' if rnd else '官方素材'))
    tmpdir = os.path.join('/tmp', 'plate_gen')
    os.makedirs(tmpdir, exist_ok=True)

    # 车牌 N -> N 号停车位; 车位里车板朝 -x, 靠外侧(墙侧)放
    plan = []
    for i, ((model, png, text, rnd), (pname, y0, y1)) in enumerate(zip(plates, PARKING), 1):
        cy = (y0 + y1) / 2.0
        plan.append(dict(name='%s_%s' % (model, 'p'), model=model, plate=text,
                         parking=pname, x=LANE_X1 - 0.02, y=round(cy, 4)))
        if not a.show:
            if rnd:
                pp = os.path.join(tmpdir, '%s_plate.png' % model)
                render_plate(text, pp)
            else:
                pp = os.path.join(SRC, png)
            write_car_model(model, pp, text)

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
