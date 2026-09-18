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

# ---- 尺寸 (m), 和相机几何匹配: 灯心 0.55 m, 相机高 0.20 m -> 1 m 外可识别 ----
LAMP_R, LAMP_T, LENS_R = 0.050, 0.012, 0.056
BOX_D, BOX_W, BOX_H = 0.080, 0.300, 0.440
ON, OFF = 0.05, -0.03
COLORS = {'red': '1 0.05 0.05 1', 'yellow': '1 0.75 0.0 1', 'green': '0.05 1 0.15 1'}
GRAY = '0.13 0.13 0.14 1'


def offsets(layout):
    d = 0.135 if layout == 'vertical' else 0.145
    if layout == 'vertical':
        return {'red': (0, 0, d), 'yellow': (0, 0, 0), 'green': (0, 0, -d)}
    return {'red': (0, d, 0), 'yellow': (0, 0, 0), 'green': (0, -d, 0)}


def model_sdf(name, layout):
    w, h = (BOX_W, BOX_H) if layout == 'vertical' else (BOX_H, BOX_W)
    off = offsets(layout)
    o = ['<?xml version="1.0"?>', '<sdf version="1.7">', '  <model name="%s">' % name,
         '    <static>false</static>', '    <self_collide>false</self_collide>',
         '    <link name="base">', '      <gravity>false</gravity>',
         '      <inertial><mass>5</mass><inertia><ixx>0.05</ixx><ixy>0</ixy><ixz>0</ixz>'
         '<iyy>0.05</iyy><iyz>0</iyz><izz>0.05</izz></inertia></inertial>',
         '      <visual name="box"><pose>0 0 0 0 0 0</pose><geometry><box>'
         '<size>%.3f %.3f %.3f</size></box></geometry><material><ambient>%s</ambient>'
         '<diffuse>%s</diffuse></material></visual>' % (BOX_D, w, h, GRAY, GRAY),
         '      <visual name="pole"><pose>0 0 -0.30 0 0 0</pose><geometry><cylinder>'
         '<radius>0.018</radius><length>0.30</length></cylinder></geometry>'
         '<material><ambient>0.35 0.35 0.36 1</ambient><diffuse>0.35 0.35 0.36 1</diffuse>'
         '</material></visual>',
         '      <visual name="foot"><pose>0 0 -0.44 0 0 0</pose><geometry><cylinder>'
         '<radius>0.09</radius><length>0.02</length></cylinder></geometry>'
         '<material><ambient>0.25 0.25 0.26 1</ambient><diffuse>0.25 0.25 0.26 1</diffuse>'
         '</material></visual>']
    for c, (ox, oy, oz) in off.items():
        o.append('      <visual name="lens_%s"><pose>%.3f %.3f %.3f 0 1.5708 0</pose>'
                 '<geometry><cylinder><radius>%.3f</radius><length>0.01</length></cylinder>'
                 '</geometry><material><ambient>0.03 0.03 0.03 1</ambient>'
                 '<diffuse>0.03 0.03 0.03 1</diffuse></material></visual>'
                 % (c, ox + BOX_D / 2.0, oy, oz, LENS_R))
    o.append('    </link>')
    for c, (ox, oy, oz) in off.items():
        o.append('    <link name="lamp_%s"><gravity>false</gravity>'
                 '<pose>%.3f %.3f %.3f 0 0 0</pose>'
                 '<inertial><mass>0.02</mass><inertia><ixx>1e-5</ixx><ixy>0</ixy><ixz>0</ixz>'
                 '<iyy>1e-5</iyy><iyz>0</iyz><izz>1e-5</izz></inertia></inertial>'
                 '<visual name="lamp"><pose>0 0 0 0 1.5708 0</pose><geometry><cylinder>'
                 '<radius>%.3f</radius><length>%.3f</length></cylinder></geometry><material>'
                 '<ambient>%s</ambient><diffuse>%s</diffuse><emissive>%s</emissive></material>'
                 '</visual></link>'
                 % (c, ox, oy, oz, LAMP_R, LAMP_T, COLORS[c], COLORS[c], COLORS[c]))
        o.append('    <joint name="j_%s" type="prismatic"><parent>base</parent>'
                 '<child>lamp_%s</child><pose>0 0 0 0 0 0</pose><axis><xyz>1 0 0</xyz>'
                 '<limit><lower>%.3f</lower><upper>%.3f</upper><effort>1</effort>'
                 '<velocity>1</velocity></limit></axis></joint>' % (c, c, OFF, ON))
    o += ['  </model>', '</sdf>', '']
    return '\n'.join(o)


def write_models(lights):
    made = []
    for L in lights:
        name = L['model']                       # traffic_light_v / traffic_light_h
        d = os.path.join(PKG, 'models', name)
        os.makedirs(d, exist_ok=True)
        open(os.path.join(d, 'model.sdf'), 'w').write(model_sdf(name, L['layout']))
        open(os.path.join(d, 'model.config'), 'w').write(
            '<?xml version="1.0"?>\n<model>\n  <name>%s</name>\n  <version>1.0</version>\n'
            '  <sdf version="1.7">model.sdf</sdf>\n  <description>临时红绿灯(物理切换)</description>\n'
            '</model>\n' % name)
        made.append(d)
    return made


def world_includes(lights):
    out = [BEGIN]
    for L in lights:
        out.append('    <include>')
        out.append('      <uri>model://%s</uri>' % L['model'])
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
    a = ap.parse_args()
    lights = yaml.safe_load(open(CFG))['lights']
    if a.show:
        for L in lights:
            print('  %-8s %-18s (%+.2f, %+.2f, %.2f) yaw=%.2f %s'
                  % (L['name'], L['model'], L['x'], L['y'], L['z'], L.get('yaw', 0), L['layout']))
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
