#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""临时红绿灯的**安装工具**：写模型文件 + 把两盏灯插进 world。

为什么不用"运行时 spawn":
    实测在 gzserver 跑着的时候用 /gazebo/spawn_sdf_model 动态生成模型,
    **服务器端相机渲染会冻住**(相机还在 9.7 Hz 发消息, 但画面永远不变,
    连把车横移 0.5 m 都不变) —— 这是 Gazebo classic 的坑。
    所以灯必须**跟世界一起加载**(写进 .world), 运行时只切关节 ✓

做两件事:
    1) 生成模型文件  src/competition_arena/models/traffic_light_v|h/{model.config,model.sdf}
    2) 把 <include> 插进 worlds/competition_arena.world 的 </world> 之前 (幂等, 可重复跑)

用法:
    python3 tools/setup_traffic_lights.py            # 用 config/traffic_lights.yaml
    python3 tools/setup_traffic_lights.py --remove   # 把灯从 world 里去掉
    python3 tools/setup_traffic_lights.py --show     # 只看当前配置
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
CFG = os.path.join(PKG, 'config', 'traffic_lights.yaml')
BEGIN = '    <!-- ===== 临时红绿灯 (tools/setup_traffic_lights.py 生成) ===== -->'
END = '    <!-- ===== 红绿灯结束 ===== -->'

# =============================================================================
#  官方尺寸 —— 来源 ~/复赛资料/红绿灯/尺寸图片.png (2026-09-21 整理)
#
#    灯箱总宽 64 cm   灯箱高 14 cm   灯箱下沿离地 34 cm (上沿 48 cm)   支架宽 2.5 cm
#    灯珠直径 8.5 cm、中心间距 22 cm —— 图上没标, 按"64cm 横线 = 1079px"定标量得
#      (三个灯珠等距: 红-黄 373px, 黄-绿 372px)
#
#  颜色直接取自官方 6 张状态参考图 (~/复赛资料/红绿灯/红绿灯图片/):
#    DARK = 灭灯时透镜塑料本身的颜色 (暗红/暗黄/暗绿, 中位色)
#    LIT  = 点亮后的高饱和色 (中位色; 最亮那 8% 像素已经过曝成近白, 不能用)
#
#  ★ 关键认识: 真实红绿灯**三个透镜始终都可见、各自有色**, 只是亮/暗不同。
#    所以模型里三个"暗透镜"是常驻视觉, 亮的那颗是把发光灯珠推到透镜前面。
# =============================================================================
HOUSING_W, HOUSING_H, HOUSING_D = 0.64, 0.14, 0.08   # 灯箱 宽 × 高 × 厚
HOUSING_Z = 0.34                                     # 灯箱下沿离地
LENS_R, LENS_T = 0.0425, 0.006                       # 常驻暗透镜 半径/厚
LAMP_R, LAMP_T = 0.0425, 0.008                       # 发光灯珠   半径/厚
LAMP_PITCH = 0.22                                    # 灯珠中心间距
POST_W = 0.025                                       # 支架方柱边长
# 关节位置: 亮 = 推到透镜前方(露出来); 灭 = 缩回灯箱内部(被不透光灯箱挡住)
ON, OFF = 0.050, 0.010

DARK = {'red':    (0.467, 0.106, 0.122),
        'yellow': (0.588, 0.357, 0.094),
        'green':  (0.094, 0.376, 0.227)}
LIT = {'red':     (0.969, 0.227, 0.086),
       'yellow':  (0.988, 0.976, 0.188),
       'green':   (0.086, 0.941, 0.529)}
HOUSING_C = (0.090, 0.102, 0.110)


def rgba(c, a=1.0):
    return '%.3f %.3f %.3f %.2f' % (c[0], c[1], c[2], a)


def lamp_offsets(layout):
    """三颗灯珠相对**模型原点**(地面中心)的位置。红-黄-绿。"""
    if layout == 'vertical':
        hz = HOUSING_Z + HOUSING_W / 2.0          # 竖排: 灯箱高 = 64cm
        return {'red':    (0.0, 0.0, hz + LAMP_PITCH),
                'yellow': (0.0, 0.0, hz),
                'green':  (0.0, 0.0, hz - LAMP_PITCH)}
    hz = HOUSING_Z + HOUSING_H / 2.0              # 横排: 灯箱高 = 14cm
    # 灯面朝 +x。站在正面(+x 处)往 -x 看时, 右手边是 +y,
    # 所以"红-黄-绿 从左到右"(官方尺寸图里的顺序) = 红在 -y、绿在 +y
    return {'red':    (0.0, -LAMP_PITCH, hz),
            'yellow': (0.0, 0.0, hz),
            'green':  (0.0, +LAMP_PITCH, hz)}


def model_sdf(name, layout, posts='double'):
    """按官方尺寸生成红绿灯模型。

    layout='horizontal': 灯箱 0.64 宽 × 0.14 高, 灯珠沿 +y 排开(红-黄-绿)
    layout='vertical':   同一灯箱转 90° (官方只有横排, 这个仅作备用)

    posts='double': 左右两根支架 (和官方照片一致)
    posts='single': 正中一根支架

    为什么需要 single: 横排灯 64 cm 宽, 若正面朝 +x 摆在顶部墙边, 双支架的
    间距是 59 cm, 靠内的那根会正好落进 60 cm 宽的车道里挡路。单支架只占 2.5 cm,
    摆在 y=2.00 (车道边缘 1.94 之外) 就不挡了。朝 +y 摆的下灯不受此影响。

    模型原点在**地面**、水平居中 -> world 里 <pose>x y 0 yaw</pose> 即可站稳。
    """
    single = (posts == 'single')
    if layout == 'vertical':
        bw, bh = HOUSING_H, HOUSING_W          # 灯箱 宽(y) × 高(z) = 14 × 64 cm
    else:
        bw, bh = HOUSING_W, HOUSING_H          # 灯箱 宽(y) × 高(z) = 64 × 14 cm
    off = lamp_offsets(layout)

    # 灯箱中心 (模型原点在地面, 水平居中)
    hz = HOUSING_Z + bh / 2.0
    # 常驻暗透镜所在的平面: 灯箱正面再往外 0.5mm, 避免与箱面 z-fighting
    fx = HOUSING_D / 2.0 + 0.0005

    o = ['<?xml version="1.0"?>', '<sdf version="1.7">', '  <model name="%s">' % name,
         '    <!-- 按官方尺寸生成, 见 tools/setup_traffic_lights.py 顶部说明 -->',
         '    <static>false</static>', '    <self_collide>false</self_collide>',
         # ★ 灯箱必须**受重力**。
         #   踩过的坑 (2026-09-21): 一开始照旧模型写 <gravity>false</gravity>,
         #   结果灯脚正好落在地面 z=0 上, 一旦有接触力, 没有重力的模型就被顶飞 ——
         #   实测 3 秒漂 0.08 m、23 秒漂 0.26 m (z 一直涨)。
         #   旧模型之所以没这个问题, 是因为它的脚离地 10cm(整个模型是悬空道具),
         #   根本碰不到地面。我们按官方尺寸做成"落地", 就必须老实给重力。
         #   三颗灯珠保持无重力 (由关节托着, 不受力)。
         '    <link name="base">',
         # ★ 灯箱关重力 —— 配合"整只灯离地 2mm 悬放"(见 world 里的 pose z=0.002):
         #   这样灯既不落体、也不与地面产生接触, 因此**完全不会漂**。
         #   踩过的坑: 开着重力时灯脚底面正好压在 z=0 上, 零穿透接触让求解器持续
         #   微推, 实测以 ~0.1mm/s 匀速慢漂(5 分钟能漂 3cm); 而且起步时会有
         #   "落地弹跳"的观感。旧模型之所以没这问题, 是因为它悬空 10cm 碰不到地面。
         '      <gravity>false</gravity>',
         '      <inertial><pose>0 0 %.3f 0 0 0</pose><mass>8</mass><inertia>'
         '<ixx>0.009</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0.139</iyy><iyz>0</iyz>'
         '<izz>0.143</izz></inertia></inertial>' % (HOUSING_Z + HOUSING_H / 2.0),
         # ---- 灯箱 ----
         '      <visual name="housing"><pose>0 0 %.3f 0 0 0</pose><geometry><box>'
         '<size>%.3f %.3f %.3f</size></box></geometry><material><ambient>%s</ambient>'
         '<diffuse>%s</diffuse></material></visual>'
         % (hz, HOUSING_D, bw, bh, rgba(HOUSING_C), rgba(HOUSING_C)),
         '      <collision name="housing_c"><pose>0 0 %.3f 0 0 0</pose><geometry><box>'
         '<size>%.3f %.3f %.3f</size></box></geometry></collision>'
         % (hz, HOUSING_D, bw, bh)]

    # ---- 支架: 默认左右两根 (官方照片); single 则正中一根 ----
    posts_y = [0.0] if (single or layout == 'vertical') else \
              [-(bw / 2 - POST_W), +(bw / 2 - POST_W)]
    for i, py in enumerate(posts_y):
        o.append('      <visual name="post%d"><pose>0 %.3f %.3f 0 0 0</pose><geometry>'
                 '<box><size>%.3f %.3f %.3f</size></box></geometry><material>'
                 '<ambient>%s</ambient><diffuse>%s</diffuse></material></visual>'
                 % (i, py, HOUSING_Z / 2.0, POST_W, POST_W, HOUSING_Z,
                    rgba(HOUSING_C), rgba(HOUSING_C)))
        o.append('      <collision name="post%d_c"><pose>0 %.3f %.3f 0 0 0</pose><geometry>'
                 '<box><size>%.3f %.3f %.3f</size></box></geometry>'
                 '<surface><friction><ode><mu>5.0</mu><mu2>5.0</mu2></ode></friction>'
                 '</surface></collision>'
                 % (i, py, HOUSING_Z / 2.0, POST_W, POST_W, HOUSING_Z))
        # 底脚 (向前后伸出的薄板, 像照片里的支架脚) —— 也要做碰撞体,
        # 否则支撑多边形在 x 方向只有支架的 2.5cm 宽, 容易前后倒
        foot_x = 0.160 if not single else 0.130
        foot_y = 0.050 if not single else 0.110
        o.append('      <visual name="foot%d"><pose>0 %.3f 0.006 0 0 0</pose><geometry>'
                 '<box><size>%.3f %.3f 0.012</size></box></geometry><material>'
                 '<ambient>0.25 0.25 0.26 1</ambient><diffuse>0.25 0.25 0.26 1</diffuse>'
                 '</material></visual>' % (i, py, foot_x, foot_y))
        o.append('      <collision name="foot%d_c"><pose>0 %.3f 0.006 0 0 0</pose><geometry>'
                 '<box><size>%.3f %.3f 0.012</size></box></geometry>'
                 '<surface><friction><ode><mu>5.0</mu><mu2>5.0</mu2></ode></friction>'
                 '</surface></collision>'
                 % (i, py, foot_x, foot_y))

    # ---- 三颗"常驻暗透镜" (真实红绿灯的三个透镜始终可见, 只是亮暗不同) ----
    for c, (ox, oy, oz) in off.items():
        o.append('      <visual name="lens_%s"><pose>%.4f %.3f %.3f 0 1.5708 0</pose>'
                 '<geometry><cylinder><radius>%.4f</radius><length>%.4f</length></cylinder>'
                 '</geometry><material><ambient>%s</ambient><diffuse>%s</diffuse>'
                 '</material></visual>'
                 % (c, fx, oy, oz, LENS_R, LENS_T, rgba(DARK[c]), rgba(DARK[c])))
    o.append('    </link>')

    # ---- 三颗发光灯珠 (挂在滑动关节上) ----
    for c, (ox, oy, oz) in off.items():
        o.append('    <link name="lamp_%s"><gravity>false</gravity>'
                 '<pose>%.4f %.3f %.3f 0 0 0</pose>'
                 '<inertial><mass>0.02</mass><inertia><ixx>1e-5</ixx><ixy>0</ixy><ixz>0</ixz>'
                 '<iyy>1e-5</iyy><iyz>0</iyz><izz>1e-5</izz></inertia></inertial>'
                 '<visual name="lamp"><pose>0 0 0 0 1.5708 0</pose><geometry><cylinder>'
                 '<radius>%.4f</radius><length>%.4f</length></cylinder></geometry><material>'
                 '<ambient>%s</ambient><diffuse>%s</diffuse><emissive>%s</emissive>'
                 '</material></visual></link>'
                 % (c, ox, oy, oz, LAMP_R, LAMP_T,
                    rgba(LIT[c]), rgba(LIT[c]), rgba(LIT[c])))
        o.append('    <joint name="j_%s" type="prismatic"><parent>base</parent>'
                 '<child>lamp_%s</child><pose>0 0 0 0 0 0</pose><axis><xyz>1 0 0</xyz>'
                 '<limit><lower>%.3f</lower><upper>%.3f</upper><effort>1</effort>'
                 '<velocity>1</velocity></limit></axis></joint>'
                 % (c, c, 0.0, ON + 0.005))
    o += ['  </model>', '</sdf>', '']
    return '\n'.join(o)


def model_name_of(L):
    """配置文件里的 model 是基名; 单支架版本加 _single 后缀 (两盏灯可能共用模型)。"""
    base = L['model']
    return base if L.get('posts', 'double') == 'double' else '%s_single' % base


def write_models(lights):
    made = []
    seen = set()
    for L in lights:
        model_name = model_name_of(L)
        if model_name in seen:
            continue
        seen.add(model_name)
        layout = L.get('layout', 'horizontal')
        posts = L.get('posts', 'double')
        d = os.path.join(PKG, 'models', model_name)
        os.makedirs(d, exist_ok=True)
        open(os.path.join(d, 'model.sdf'), 'w').write(model_sdf(model_name, layout, posts))
        open(os.path.join(d, 'model.config'), 'w').write(
            '<?xml version="1.0"?>\n<model>\n  <name>%s</name>\n  <version>1.0</version>\n'
            '  <sdf version="1.7">model.sdf</sdf>\n  <description>红绿灯(物理切换, 按官方尺寸)</description>\n'
            '</model>\n' % model_name)
        made.append(d)
    return made


def world_includes(lights):
    out = [BEGIN]
    for L in lights:
        out.append('    <include>')
        out.append('      <uri>model://%s</uri>' % model_name_of(L))
        out.append('      <name>%s</name>' % L['name'])
        out.append('      <pose>%.4f %.4f %.4f 0 0 %.4f</pose>'
                   % (L['x'], L['y'], L['z'], L.get('yaw', 0.0)))
        out.append('    </include>')
    out.append(END)
    return '\n'.join(out) + '\n'


def patch_world(lights, remove=False):
    s = open(WORLD).read()
    s = re.sub(re.escape(BEGIN) + r'.*?' + re.escape(END) + r'\n?', '', s, flags=re.S)
    if not remove:
        if '</world>' not in s:
            raise SystemExit('world 文件里没有 </world>')
        s = s.replace('</world>', world_includes(lights) + '  </world>', 1)
    open(WORLD, 'w').write(s)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--remove', action='store_true')
    ap.add_argument('--show', action='store_true')
    ap.add_argument('--models-only', dest='models_only', action='store_true',
                    help='只重新生成模型文件, 不动 world (调模型时用)')
    a = ap.parse_args()
    lights = yaml.safe_load(open(CFG))['lights']
    if a.show:
        for L in lights:
            print('  %-8s %-18s (%+.2f, %+.2f, %.2f) yaw=%.2f %s'
                  % (L['name'], L['model'], L['x'], L['y'], L['z'], L.get('yaw', 0), L['layout']))
        return
    if a.models_only:
        for d in write_models(lights):
            print('  模型 ->', os.path.relpath(d, WS))
        return
    patch_world(lights, remove=a.remove)
    if a.remove:
        print('已从 world 里移除红绿灯')
    else:
        for d in write_models(lights):
            print('  模型 ->', os.path.relpath(d, WS))
        print('  已把 %d 盏灯插进 %s' % (len(lights), os.path.relpath(WORLD, WS)))
        print('  注意: 改了 world 要重启仿真才生效')


if __name__ == '__main__':
    main()
