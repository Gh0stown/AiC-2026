#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gen_robot.py —— 从 config/robot_params.yaml 生成 urdf/competition_robot.urdf

设计原则 (sim2ros):
  * 参数只有一份 = config/robot_params.yaml
  * 本脚本把参数"编译"成 URDF, 不做任何自己的假设
  * 真机驱动读同一份 YAML, 于是仿真和真车的几何 / 运动学 / topic / frame 一致

用法:
    python3 scripts/gen_robot.py                 # 用默认参数表
    python3 scripts/gen_robot.py --params xxx.yaml --out yyy.urdf
    python3 scripts/gen_robot.py --print          # 只打印摘要, 不写文件
"""

from __future__ import annotations

import argparse
import datetime
import math
import os
import sys
import xml.etree.ElementTree as ET

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
sys.path.insert(0, HERE)

from mecanum import MecanumKinematics, from_params as mecanum_from_params  # noqa: E402


# --------------------------------------------------------------------------- utils
def f(v):
    """紧凑浮点格式化。"""
    s = '%.6f' % float(v)
    s = s.rstrip('0').rstrip('.')
    return s if s not in ('', '-0') else '0'


def vec(v):
    return ' '.join(f(x) for x in v)


def rpy_from_z(direction):
    """求把局部 +Z 轴旋到 direction 的 (roll, pitch, yaw)。

    圆柱体在 URDF 里默认沿 +Z, 用它来摆放麦克纳姆辊子。
    """
    x, y, z = direction
    n = math.sqrt(x * x + y * y + z * z)
    if n < 1e-12:
        return 0.0, 0.0, 0.0
    x, y, z = x / n, y / n, z / n
    z = max(-1.0, min(1.0, z))
    pitch = math.acos(z)
    yaw = math.atan2(y, x) if math.sin(pitch) > 1e-9 else 0.0
    return 0.0, pitch, yaw


def box_inertia(mass, size, origin=(0, 0, 0)):
    x, y, z = size
    return (
        '      <inertial>\n'
        '        <origin xyz="%s" rpy="0 0 0"/>\n'
        '        <mass value="%s"/>\n'
        '        <inertia ixx="%s" ixy="0" ixz="0" iyy="%s" iyz="0" izz="%s"/>\n'
        '      </inertial>\n'
        % (vec(origin), f(mass),
           f(mass * (y * y + z * z) / 12.0),
           f(mass * (x * x + z * z) / 12.0),
           f(mass * (x * x + y * y) / 12.0))
    )


def cyl_inertia(mass, radius, length, origin=(0, 0, 0), axis='z'):
    a = mass * (3 * radius * radius + length * length) / 12.0
    c = mass * radius * radius / 2.0
    if axis == 'z':       # 轴向为 Z
        ixx, iyy, izz = a, a, c
    elif axis == 'y':     # 轴向为 Y
        ixx, izz, iyy = a, a, c
    else:                 # 轴向为 X
        iyy, izz, ixx = a, a, c
    return (
        '      <inertial>\n'
        '        <origin xyz="%s" rpy="0 0 0"/>\n'
        '        <mass value="%s"/>\n'
        '        <inertia ixx="%s" ixy="0" ixz="0" iyy="%s" iyz="0" izz="%s"/>\n'
        '      </inertial>\n'
        % (vec(origin), f(mass), f(ixx), f(iyy), f(izz))
    )


# --------------------------------------------------------------------------- pieces
def wheel_xml(tag, wheel_r, wheel_w, roller_count, roller_angle_deg, mats,
              wheel_mass_sim=0.4):
    """一个麦克纳姆轮: 轮毂圆柱 + 一圈 45° 辊子 (辊子只做外观)。"""
    hub_r = wheel_r * 0.72
    roll_r = wheel_r * 0.22
    roll_l = wheel_w * 1.06
    s = []
    s.append('    <visual>\n'
             '      <origin xyz="0 0 0" rpy="1.5707963 0 0"/>\n'
             '      <geometry><cylinder radius="%s" length="%s"/></geometry>\n'
             '      <material name="wheel_hub"/>\n'
             '    </visual>\n' % (f(hub_r), f(wheel_w)))
    # 辊子: 绕 Y 轴(轮轴)均布, 每个辊子轴与轮轴成 roller_angle
    ra = math.radians(roller_angle_deg)
    for i in range(roller_count):
        th = 2.0 * math.pi * i / roller_count
        # 辊子中心在圆周上
        cx = wheel_r * 0.86 * math.cos(th)
        cz = wheel_r * 0.86 * math.sin(th)
        # 切线方向 与 轮轴(Y) 合成 -> 辊子轴向
        tangent = (-math.sin(th), 0.0, math.cos(th))
        axis_y = (0.0, 1.0, 0.0)
        direction = (math.cos(ra) * tangent[0] + math.sin(ra) * axis_y[0],
                     math.cos(ra) * tangent[1] + math.sin(ra) * axis_y[1],
                     math.cos(ra) * tangent[2] + math.sin(ra) * axis_y[2])
        r, p, y = rpy_from_z(direction)
        s.append('    <visual>\n'
                 '      <origin xyz="%s" rpy="%s"/>\n'
                 '      <geometry><cylinder radius="%s" length="%s"/></geometry>\n'
                 '      <material name="wheel_roller"/>\n'
                 '    </visual>\n'
                 % (vec((cx, 0.0, cz)), vec((r, p, y)), f(roll_r), f(roll_l)))
    # 碰撞体用一个光滑圆柱 (辊子不参与物理)
    s.append('    <collision>\n'
             '      <origin xyz="0 0 0" rpy="1.5707963 0 0"/>\n'
             '      <geometry><cylinder radius="%s" length="%s"/></geometry>\n'
             '    </collision>\n' % (f(wheel_r), f(wheel_w)))
    s.append(cyl_inertia(wheel_mass_sim, wheel_r, wheel_w, axis='y'))
    return ''.join(s)


def wheel_collision_xml(wheel_r, wheel_w):
    """轮子的碰撞体用一个光滑圆柱 (STL 做碰撞又慢又不稳)。"""
    return ('    <collision>\n'
            '      <origin xyz="0 0 0" rpy="1.5707963 0 0"/>\n'
            '      <geometry><cylinder radius="%s" length="%s"/></geometry>\n'
            '    </collision>\n' % (f(wheel_r), f(wheel_w)))


def mesh_visual(rel_path, pkg, material, origin='0 0 0', rpy='0 0 0'):
    """用实车导出的 STL 做视觉。碰撞体仍然用简化几何体 (快且稳)。"""
    return ('    <visual>\n'
            '      <origin xyz="%s" rpy="%s"/>\n'
            '      <geometry><mesh filename="package://%s/meshes/%s"/></geometry>\n'
            '      <material name="%s"/>\n'
            '    </visual>\n' % (origin, rpy, pkg, rel_path, material))


# 轮子位置 -> 实车导出的 mesh 文件名
WHEEL_MESH = {
    'fl': 'front_left_wheel',
    'fr': 'front_right_wheel',
    'rl': 'back_left_wheel',
    'rr': 'back_right_wheel',
}

MATERIALS = """
  <material name="chassis"><color rgba="0.22 0.24 0.28 1.0"/></material>
  <material name="wheel_hub"><color rgba="0.85 0.45 0.10 1.0"/></material>
  <material name="wheel_roller"><color rgba="0.20 0.20 0.22 1.0"/></material>
  <material name="sensor"><color rgba="0.15 0.65 0.85 1.0"/></material>
"""


# --------------------------------------------------------------------------- main build
def build(params):
    robot = params['robot']
    c = params['chassis']
    m = params['mecanum']
    enc = params.get('encoder', {})
    sen = params['sensors']
    sim = params.get('simulation', {})

    name = robot['name']
    F = robot['frames']

    # 视觉用实车导出的 STL, 还是用简化几何体
    use_mesh = str(sim.get('visual_style', 'mesh')).lower() == 'mesh'

    kin = mecanum_from_params(params)

    # --- 仿真用的轮子质量 ---------------------------------------------------
    # 见 robot_params.yaml 里 light_wheels 的说明
    wheel_mass = float(m.get('wheel_mass', 0.4))
    light = bool(sim.get('light_wheels', False))
    wheel_mass_sim = float(sim.get('wheel_mass_sim', 0.01)) if light else wheel_mass
    chassis_mass = float(c['mass']) + (4.0 * (wheel_mass - wheel_mass_sim) if light else 0.0)

    # --- 关键几何 -----------------------------------------------------------
    # 和实车 URDF 保持一致: base_footprint 与 base_link 重合, 都在**地面**原点。
    # 于是轮心高度 = 轮半径, 传感器安装位置就是"离地高度", 可以直接和实车对照。
    wheel_r = float(m['wheel_radius'])
    wheel_z = wheel_r
    center_off = c.get('center_offset', [0.0, 0.0, wheel_r + float(c['height']) / 2.0])
    wpos = kin.wheel_positions()

    out = []
    out.append('<?xml version="1.0"?>\n')
    out.append('<!--\n')
    out.append('  %s —— 由 scripts/gen_robot.py 自动生成, 请勿手改。\n' % name)
    out.append('  改参数请编辑 config/robot_params.yaml, 然后重新运行:\n')
    out.append('      python3 scripts/gen_robot.py\n')
    out.append('\n')
    out.append('  生成时间   : %s\n' % datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S'))
    out.append('  底盘构型   : 四轮麦克纳姆 (X 型布局)\n')
    out.append('  轮径/轮宽  : %.4f / %.4f m\n' % (wheel_r, m['wheel_width']))
    out.append('  轴距/轮距  : %.4f / %.4f m\n' % (kin.wheelbase, kin.track))
    out.append('  前/后轮 x  : %+.4f / %+.4f m%s\n'
               % (kin.xf, kin.xr,
                  '' if abs(kin.k_front - kin.k_rear) < 1e-9 else
                  '  (前后不对称, 自转系数 %.4f / %.4f)' % (kin.k_front, kin.k_rear)))
    out.append('  整车质量   : %.2f kg (车体) + 4 x %.2f kg (轮) = %.2f kg\n'
               % (chassis_mass, wheel_mass_sim, chassis_mass + 4 * wheel_mass_sim))
    out.append('  base_link  : 与 base_footprint 重合, 都在地面 (和实车一致)\n')
    out.append('  车体中心   : 相对 base_link 偏移 %s\n'
               % ' '.join('%.4f' % v for v in center_off))
    out.append('  驱动方式   : %s\n' % sim.get('drive_mode', 'planar_move'))
    out.append('-->\n')
    out.append('<robot name="%s">\n' % name)
    out.append(MATERIALS)

    # --- base_footprint -----------------------------------------------------
    out.append('\n  <!-- 贴地投影, 导航/定位用 -->\n')
    out.append('  <link name="%s"/>\n' % F['base_footprint'])
    out.append('  <joint name="base_joint" type="fixed">\n')
    out.append('    <parent link="%s"/>\n' % F['base_footprint'])
    out.append('    <child link="%s"/>\n' % F['base_link'])
    out.append('    <origin xyz="0 0 0" rpy="0 0 0"/>\n')
    out.append('  </joint>\n')

    # --- 车体 ---------------------------------------------------------------
    com = c.get('com_offset', [0.0, 0.0])
    out.append('\n  <!-- 车体 -->\n')
    out.append('  <link name="%s">\n' % F['base_link'])
    co = vec(center_off)
    if use_mesh:
        # 实车 STL 的坐标原点就是 base_link, 偏移已经做在模型里了, 不用再加
        out.append(mesh_visual('mecanum/base_link.STL', name, 'chassis'))
    else:
        out.append('    <visual>\n'
                   '      <origin xyz="%s" rpy="0 0 0"/>\n'
                   '      <geometry><box size="%s %s %s"/></geometry>\n'
                   '      <material name="chassis"/>\n'
                   '    </visual>\n'
                   % (co, f(c['length']), f(c['width']), f(c['height'])))
    out.append('    <collision>\n'
               '      <origin xyz="%s" rpy="0 0 0"/>\n'
               '      <geometry><box size="%s %s %s"/></geometry>\n'
               '    </collision>\n'
               % (co, f(c['length']), f(c['width']), f(c['height'])))
    out.append(box_inertia(chassis_mass, (c['length'], c['width'], c['height']),
                           (center_off[0] + com[0], center_off[1] + com[1], center_off[2])))
    out.append('  </link>\n')

    # --- 四个麦轮 -----------------------------------------------------------
    out.append('\n  <!-- ================= 麦克纳姆轮 ================= -->\n')
    out.append('  <!-- 轮心相对 base_link: z=%s (轮心离地 %.4f m) -->\n'
               % (f(wheel_z), wheel_r))
    for tag in kin.WHEELS:
        x, y = wpos[tag]
        out.append('\n  <link name="%s_wheel_link">\n' % tag)
        if use_mesh:
            out.append(mesh_visual('mecanum/%s.STL' % WHEEL_MESH[tag], name, 'wheel_hub'))
            out.append(wheel_collision_xml(wheel_r, float(m['wheel_width'])))
            out.append(cyl_inertia(wheel_mass_sim, wheel_r, float(m['wheel_width']),
                                   axis='y'))
        else:
            out.append(wheel_xml(tag, wheel_r, float(m['wheel_width']),
                                 int(m.get('roller_count', 9)),
                                 float(m.get('roller_angle', 45.0)), None,
                                 wheel_mass_sim))
        out.append('  </link>\n')
        out.append('  <joint name="%s_wheel_joint" type="continuous">\n' % tag)
        out.append('    <parent link="%s"/>\n' % F['base_link'])
        out.append('    <child link="%s_wheel_link"/>\n' % tag)
        out.append('    <origin xyz="%s %s %s" rpy="0 0 0"/>\n' % (f(x), f(y), f(wheel_z)))
        out.append('    <axis xyz="0 1 0"/>\n')
        out.append('    <dynamics damping="0.02" friction="0.0"/>\n')
        out.append('  </joint>\n')
        out.append('  <gazebo reference="%s_wheel_link">\n' % tag)
        # 不覆盖材质: 让 Gazebo 用 URDF 里的颜色, 和 RViz 显示一致
        out.append('    <!-- 麦轮各向异性摩擦: 沿滚动方向(+x)高摩擦 -> 能驱动;\n')
        out.append('         沿轮轴方向(+y)低摩擦 -> 允许横移。mu1=fdir1 方向, mu2=垂直方向。\n')
        out.append('         不加这个的话, planar_move 发的 vy 会被轮子侧向刮地摩擦直接吃掉。 -->\n')
        wf = sim.get('wheel_friction', {})
        out.append('    <mu1>%s</mu1>\n' % f(wf.get('rolling', 1.2)))
        out.append('    <mu2>%s</mu2>\n' % f(wf.get('lateral', 0.005)))
        out.append('    <fdir1>1 0 0</fdir1>\n')
        out.append('    <kp>1000000.0</kp><kd>100.0</kd>\n')
        out.append('  </gazebo>\n')

    # --- 传感器 -------------------------------------------------------------
    def fixed_sensor_link(tag, link_name, geometry_xml, inert, joint_name, mount, rpy):
        s = '\n  <link name="%s">\n%s%s  </link>\n' % (link_name, geometry_xml, inert)
        s += '  <joint name="%s" type="fixed">\n' % joint_name
        s += '    <parent link="%s"/>\n' % F['base_link']
        s += '    <child link="%s"/>\n' % link_name
        s += '    <origin xyz="%s" rpy="%s"/>\n' % (vec(mount), vec(rpy))
        s += '  </joint>\n'
        return s

    sensors_summary = []

    # 2D 单线激光雷达 (实车 N10_P)
    l2 = sen.get('lidar2d', {})
    if l2.get('enabled'):
        full = bool(l2.get('use_full_circle', True))
        amin, amax = (-math.pi, math.pi) if full else (-math.pi / 2.0, math.pi / 2.0)
        out.append('\n  <!-- ================= 2D 激光雷达 (%s) ================= -->\n'
                   % l2.get('model', ''))
        geo = ('    <visual>\n'
               '      <origin xyz="0 0 0" rpy="0 0 0"/>\n'
               '      <geometry><cylinder radius="0.05" length="0.05"/></geometry>\n'
               '      <material name="sensor"/>\n'
               '    </visual>\n'
               '    <collision>\n'
               '      <origin xyz="0 0 0" rpy="0 0 0"/>\n'
               '      <geometry><cylinder radius="0.05" length="0.05"/></geometry>\n'
               '    </collision>\n')
        geo = mesh_visual('sensor/laser_link.STL', name, 'sensor') if use_mesh else geo
        out.append(fixed_sensor_link('lidar2d', F['laser'], geo,
                                     cyl_inertia(float(l2.get('mass', 0.2)), 0.05, 0.05),
                                     'laser_joint', l2['mount'], l2['mount_rpy']))
        out.append('  <gazebo reference="%s">\n' % F['laser'])
        # 不覆盖材质: 让 Gazebo 用 URDF 里的颜色, 和 RViz 显示一致
        out.append('    <sensor type="ray" name="lidar2d">\n')
        out.append('      <always_on>true</always_on>\n')
        out.append('      <visualize>false</visualize>\n')
        out.append('      <update_rate>%s</update_rate>\n' % f(l2['update_rate']))
        out.append('      <ray>\n        <scan>\n          <horizontal>\n')
        out.append('            <samples>%d</samples>\n' % int(l2['samples']))
        out.append('            <resolution>1</resolution>\n')
        out.append('            <min_angle>%s</min_angle>\n' % f(amin))
        out.append('            <max_angle>%s</max_angle>\n' % f(amax))
        out.append('          </horizontal>\n        </scan>\n')
        out.append('        <range>\n          <min>%s</min>\n          <max>%s</max>\n'
                   % (f(l2['range'][0]), f(l2['range'][1])))
        out.append('          <resolution>0.01</resolution>\n        </range>\n')
        out.append('        <noise><type>gaussian</type><mean>0.0</mean>'
                   '<stddev>0.01</stddev></noise>\n')
        out.append('      </ray>\n')
        out.append('      <plugin name="lidar2d_plugin" filename="libgazebo_ros_laser.so">\n')
        out.append('        <topicName>%s</topicName>\n' % l2['topic'])
        out.append('        <frameName>%s</frameName>\n' % l2.get('frame', F['laser']))
        out.append('      </plugin>\n    </sensor>\n  </gazebo>\n')
        sensors_summary.append('2D 雷达   %s, %d 点/圈 @ %.0f Hz, %s -> /%s'
                               % (l2.get('model', ''), int(l2['samples']),
                                  l2['update_rate'],
                                  '360°' if full else '前 180°', l2['topic']))

    # 3D 激光雷达 (可选)
    lidar = sen.get('lidar3d', {})
    if lidar.get('enabled'):
        vfov = lidar['vertical_fov_deg']
        vmin, vmax = math.radians(vfov[0]), math.radians(vfov[1])
        rings = int(lidar['rings'])
        hpts = int(lidar['horizontal_samples'])
        out.append('\n  <!-- ================= 3D 激光雷达 (%d 线) ================= -->\n' % rings)
        geo = ('    <visual>\n'
               '      <origin xyz="0 0 0" rpy="0 0 0"/>\n'
               '      <geometry><cylinder radius="0.05" length="0.07"/></geometry>\n'
               '      <material name="sensor"/>\n'
               '    </visual>\n'
               '    <collision>\n'
               '      <origin xyz="0 0 0" rpy="0 0 0"/>\n'
               '      <geometry><cylinder radius="0.05" length="0.07"/></geometry>\n'
               '    </collision>\n')
        geo = mesh_visual('sensor/laser_link.STL', name, 'sensor') if use_mesh else geo
        # ★ 3D 雷达用**自己的** frame (lidar3d_link), 不和 2D 雷达抢 laser_link。
        #   实车只有 2D 雷达, 所以 laser_link 留给它, 这样仿真和实车的 scan frame 一致。
        l3 = F.get('laser3d', 'lidar3d_link')
        out.append(fixed_sensor_link('lidar3d', l3, geo,
                                     cyl_inertia(float(lidar.get('mass', 0.9)), 0.05, 0.07),
                                     'lidar3d_joint', lidar['mount'], lidar['mount_rpy']))
        out.append('  <gazebo reference="%s">\n' % l3)
        # 不覆盖材质: 让 Gazebo 用 URDF 里的颜色, 和 RViz 显示一致
        out.append('    <sensor type="ray" name="lidar3d">\n')
        out.append('      <pose>0 0 0 0 0 0</pose>\n')
        out.append('      <visualize>false</visualize>\n')
        out.append('      <update_rate>%s</update_rate>\n' % f(lidar['update_rate']))
        out.append('      <ray>\n        <scan>\n')
        out.append('          <horizontal>\n')
        out.append('            <samples>%d</samples>\n' % hpts)
        out.append('            <resolution>1</resolution>\n')
        out.append('            <min_angle>%s</min_angle>\n' % f(-math.pi))
        out.append('            <max_angle>%s</max_angle>\n' % f(math.pi))
        out.append('          </horizontal>\n')
        out.append('          <vertical>\n')
        out.append('            <samples>%d</samples>\n' % rings)
        out.append('            <resolution>1</resolution>\n')
        out.append('            <min_angle>%s</min_angle>\n' % f(vmin))
        out.append('            <max_angle>%s</max_angle>\n' % f(vmax))
        out.append('          </vertical>\n')
        out.append('        </scan>\n')
        out.append('        <range>\n')
        out.append('          <min>%s</min>\n          <max>%s</max>\n'
                   % (f(lidar['range'][0]), f(lidar['range'][1])))
        out.append('          <resolution>0.01</resolution>\n')
        out.append('        </range>\n')
        out.append('        <noise><type>gaussian</type><mean>0.0</mean>'
                   '<stddev>0.01</stddev></noise>\n')
        out.append('      </ray>\n')
        out.append('      <plugin name="lidar3d_plugin" filename="libgazebo_ros_block_laser.so">\n')
        out.append('        <topicName>%s</topicName>\n' % lidar['topic'])
        out.append('        <frameName>%s</frameName>\n' % l3)
        out.append('        <robotNamespace>/</robotNamespace>\n')
        out.append('      </plugin>\n')
        out.append('    </sensor>\n')
        out.append('  </gazebo>\n')
        sensors_summary.append('3D 雷达   %2d 线 x %4d 点/线 @ %.0f Hz -> /%s  (%d 条射线)'
                               % (rings, hpts, lidar['update_rate'], lidar['topic'],
                                  rings * hpts))

    # 相机 (可以用深度相机发 RGB+深度, 也可以只用普通相机发 RGB)
    cam = sen.get('camera', sen.get('depth_camera', {}))
    if cam.get('enabled'):
        hfov = math.radians(cam['horizontal_fov_deg'])
        out.append('\n  <!-- ================= 相机 ================= -->\n')
        geo = ('    <visual>\n'
               '      <origin xyz="0 0 0" rpy="0 0 0"/>\n'
               '      <geometry><box size="0.03 0.10 0.03"/></geometry>\n'
               '      <material name="sensor"/>\n'
               '    </visual>\n'
               '    <collision>\n'
               '      <origin xyz="0 0 0" rpy="0 0 0"/>\n'
               '      <geometry><box size="0.03 0.10 0.03"/></geometry>\n'
               '    </collision>\n')
        geo = mesh_visual('sensor/camera_link.STL', name, 'sensor') if use_mesh else geo
        out.append(fixed_sensor_link('depth_camera', F['camera'], geo,
                                     box_inertia(float(cam.get('mass', 0.08)), (0.03, 0.10, 0.03)),
                                     'camera_joint', cam['mount'], cam['mount_rpy']))
        out.append('  <gazebo reference="%s">\n' % F['camera'])
        # 不覆盖材质: 让 Gazebo 用 URDF 里的颜色, 和 RViz 显示一致
        pub_depth = bool(cam.get('publish_depth', True))
        if pub_depth:
            out.append('    <sensor type="depth" name="depth_camera">\n')
        else:
            out.append('    <!-- 只发 RGB, 不做深度 -->\n')
            out.append('    <sensor type="camera" name="camera">\n')
        out.append('      <always_on>true</always_on>\n')
        out.append('      <update_rate>%s</update_rate>\n' % f(cam['update_rate']))
        out.append('      <camera>\n')
        out.append('        <horizontal_fov>%s</horizontal_fov>\n' % f(hfov))
        out.append('        <image><width>%d</width><height>%d</height>'
                   '<format>R8G8B8</format></image>\n' % (cam['width'], cam['height']))
        out.append('        <clip><near>%s</near><far>%s</far></clip>\n'
                   % (f(cam['range'][0]), f(cam['range'][1])))
        out.append('      </camera>\n')
        if pub_depth:
            out.append('      <plugin name="depth_camera_plugin" '
                       'filename="libgazebo_ros_openni_kinect.so">\n')
        else:
            out.append('      <plugin name="camera_plugin" '
                       'filename="libgazebo_ros_camera.so">\n')
        out.append('        <cameraName>%s</cameraName>\n' % cam['topic'])
        out.append('        <frameName>%s</frameName>\n' % F['camera'])
        out.append('        <imageTopicName>rgb/image_raw</imageTopicName>\n')
        out.append('        <cameraInfoTopicName>rgb/camera_info</cameraInfoTopicName>\n')
        if pub_depth:
            out.append('        <depthImageTopicName>depth/image_raw</depthImageTopicName>\n')
            out.append('        <pointCloudTopicName>depth/points</pointCloudTopicName>\n')
            out.append('        <depthImageCameraInfoTopicName>depth/camera_info'
                       '</depthImageCameraInfoTopicName>\n')
            out.append('        <pointCloudCutoff>%s</pointCloudCutoff>\n'
                       % f(cam['range'][0]))
            out.append('        <pointCloudCutoffMax>%s</pointCloudCutoffMax>\n'
                       % f(cam['range'][1]))
        out.append('      </plugin>\n')
        out.append('    </sensor>\n')
        out.append('  </gazebo>\n')
        sensors_summary.append('%s %dx%d @ %.0f Hz -> /%s/rgb/...'
                               % ('深度相机' if pub_depth else 'RGB 相机',
                                  cam['width'], cam['height'], cam['update_rate'],
                                  cam['topic']))

    # IMU
    imu = sen.get('imu', {})
    if imu.get('enabled'):
        out.append('\n  <!-- ================= IMU ================= -->\n')
        geo = ('    <visual>\n'
               '      <origin xyz="0 0 0" rpy="0 0 0"/>\n'
               '      <geometry><box size="0.04 0.04 0.015"/></geometry>\n'
               '      <material name="sensor"/>\n'
               '    </visual>\n'
               '    <collision>\n'
               '      <origin xyz="0 0 0" rpy="0 0 0"/>\n'
               '      <geometry><box size="0.04 0.04 0.015"/></geometry>\n'
               '    </collision>\n')
        geo = mesh_visual('sensor/imu_link.STL', name, 'sensor') if use_mesh else geo
        out.append(fixed_sensor_link('imu', F['imu'], geo,
                                     box_inertia(float(imu.get('mass', 0.02)), (0.04, 0.04, 0.015)),
                                     'imu_joint', imu['mount'], imu['mount_rpy']))
        out.append('  <gazebo reference="%s">\n' % F['imu'])
        out.append('    <sensor name="imu_sensor" type="imu">\n')
        out.append('      <always_on>true</always_on>\n')
        out.append('      <update_rate>%s</update_rate>\n' % f(imu['update_rate']))
        out.append('      <visualize>false</visualize>\n')
        out.append('      <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">\n')
        out.append('        <topicName>%s</topicName>\n' % imu['topic'])
        out.append('        <bodyName>%s</bodyName>\n' % F['imu'])
        out.append('        <frameName>%s</frameName>\n' % F['imu'])
        out.append('        <updateRateHZ>%s</updateRateHZ>\n' % f(imu['update_rate']))
        out.append('        <gaussianNoise>0.0</gaussianNoise>\n')
        out.append('        <xyzOffset>0 0 0</xyzOffset>\n')
        out.append('        <rpyOffset>0 0 0</rpyOffset>\n')
        out.append('      </plugin>\n')
        out.append('    </sensor>\n')
        out.append('  </gazebo>\n')
        sensors_summary.append('IMU       @ %.0f Hz -> /%s' % (imu['update_rate'], imu['topic']))

    # --- 驱动 ---------------------------------------------------------------
    out.append('\n  <!-- ================= 驱动 ================= -->\n')
    drive = sim.get('drive_plugin', 'holonomic')
    if drive == 'holonomic':
        vm = sim.get('velocity_mode', 'canonical')
        out.append('  <!-- 全向驱动: 自研 C++ 插件, 替代 gazebo_ros_planar_move。\n')
        out.append('       planar_move 实测实际自转只有指令的 0.637 倍, 本插件解决这个问题。\n')
        out.append('       velocityMode 可选 canonical / all_rigid / model (见插件源码注释)。 -->\n')
        out.append('  <gazebo>\n')
        out.append('    <plugin name="holonomic_drive" '
                   'filename="libcompetition_holonomic_drive.so">\n')
        out.append('      <commandTopic>cmd_vel</commandTopic>\n')
        out.append('      <odometryTopic>odom_groundtruth</odometryTopic>\n')
        out.append('      <odometryFrame>odom_groundtruth</odometryFrame>\n')
        out.append('      <odometryRate>%s</odometryRate>\n'
                   % f(sim.get('odom_rate', 50.0)))
        out.append('      <robotBaseFrame>%s</robotBaseFrame>\n' % F['base_footprint'])
        out.append('      <robotNamespace>/</robotNamespace>\n')
        out.append('      <cmdTimeout>%s</cmdTimeout>\n'
                   % f(sim.get('cmd_timeout', 0.5)))
        out.append('      <velocityMode>%s</velocityMode>\n' % vm)
        out.append('      <publishOdometry>true</publishOdometry>\n')
        out.append('    </plugin>\n')
        out.append('  </gazebo>\n')
    else:
        out.append('  <!-- 备用: gazebo 自带的 planar_move (自转只有指令的 ~0.637 倍) -->\n')
        out.append('  <gazebo>\n')
        out.append('    <plugin name="planar_move" '
                   'filename="libgazebo_ros_planar_move.so">\n')
        out.append('      <commandTopic>cmd_vel</commandTopic>\n')
        out.append('      <odometryTopic>odom_groundtruth</odometryTopic>\n')
        out.append('      <odometryFrame>odom_groundtruth</odometryFrame>\n')
        out.append('      <odometryRate>%s</odometryRate>\n'
                   % f(sim.get('odom_rate', 50.0)))
        out.append('      <robotBaseFrame>%s</robotBaseFrame>\n' % F['base_footprint'])
        out.append('      <robotNamespace>/</robotNamespace>\n')
        out.append('      <cmdTimeout>%s</cmdTimeout>\n'
                   % f(sim.get('cmd_timeout', 0.5)))
        out.append('    </plugin>\n')
        out.append('  </gazebo>\n')

    # --- 编码器 (轮子关节状态) ----------------------------------------------
    out.append('\n  <!-- ================= 轮式编码器 ================= -->\n')
    out.append('  <!-- 仿真里机器人由 planar_move 直接驱动, 轮子大多在打滑, Gazebo 的\n')
    out.append('       关节角度不能当编码器用。所以编码器由节点模拟:\n')
    out.append('         scripts/sim_wheel_encoders.py  用同一套麦轮逆运动学积分出四个\n')
    out.append('         轮子转角, 发到 /joint_states (= 理想编码器)\n')
    out.append('         scripts/mecanum_odometry.py    读 /joint_states 算 /odom\n')
    out.append('       真机上第二个节点原样复用, 第一个节点换成真实编码器读取。 -->\n')

    out.append('\n</robot>\n')
    xml = ''.join(out)

    # planar_move 只对 canonical link 调 SetAngularVel, 车轮通过关节把角速度
    # 重新分配, 所以 实际自转/指令自转 = I_base/(I_base+I_wheels)。
    wheel_w = float(m['wheel_width'])
    i_base = chassis_mass * (float(c['length']) ** 2 + float(c['width']) ** 2) / 12.0
    a_w = wheel_mass_sim * (3.0 * wheel_r ** 2 + wheel_w ** 2) / 12.0
    # 每个轮子相对原点的距离不同 (前后不对称), 逐个算
    i_wheels = sum(a_w + wheel_mass_sim * (wx * wx + wy * wy)
                   for (wx, wy) in kin.wheel_positions().values())
    wz_gain = i_base / (i_base + i_wheels)

    # 质量账目 (便于核对总重)
    mass_items = [('车体(含电机/电池/工控机)', chassis_mass),
                  ('车轮 x4', 4.0 * wheel_mass_sim)]
    for k, v in (('lidar3d', lidar), ('lidar2d', l2), ('camera', cam), ('imu', imu)):
        if v.get('enabled'):
            mass_items.append((k, float(v.get('mass', 0.0))))
    total_mass = sum(m for _, m in mass_items)

    summary = {
        'kinematics': kin,
        'mass_items': mass_items,
        'total_mass': total_mass,
        'chassis_mass': chassis_mass,
        'wheel_mass_sim': wheel_mass_sim,
        'light_wheels': light,
        'wz_gain': wz_gain,
        'center_offset': center_off,
        'wheel_z': wheel_z,
        'wheel_positions': wpos,
        'sensors': sensors_summary,
        'encoder_cpr': MecanumKinematics.counts_per_wheel_rev(
            enc.get('counts_per_rev', 0), enc.get('gear_ratio', 0),
            enc.get('quadrature', 1)),
    }
    return xml, summary


# --------------------------------------------------------------------------- cli
def main():
    ap = argparse.ArgumentParser(description='从 robot_params.yaml 生成机器人 URDF')
    ap.add_argument('--params', default=os.path.join(PKG, 'config', 'robot_params.yaml'))
    ap.add_argument('--out', default=os.path.join(PKG, 'urdf', 'competition_robot.urdf'))
    ap.add_argument('--print', dest='dry', action='store_true', help='只打印摘要')
    args = ap.parse_args()

    with open(args.params) as fp:
        params = yaml.safe_load(fp)

    xml, s = build(params)
    # 语法自检: 生成的东西必须是合法 XML
    ET.fromstring(xml)

    kin = s['kinematics']
    print('=' * 74)
    print('  机器人           : %s' % params['robot']['name'])
    print('  构型             : 四轮麦克纳姆 (X 型)')
    print('  轮径 / 轮宽      : %.4f / %.4f m' % (kin.r, params['mecanum']['wheel_width']))
    print('  轴距 / 轮距      : %.4f / %.4f m' % (kin.wheelbase, kin.track))
    print('  自转系数         : 前 %.4f / 后 %.4f m%s'
          % (kin.k_front, kin.k_rear,
             '' if abs(kin.k_front - kin.k_rear) < 1e-9
             else '   ← 前后不对称 (实车就是这样)'))
    print('  质量账目         :')
    for name, m in s['mass_items']:
        print('      %-26s %.3f kg' % (name, m))
    print('      %-26s %.3f kg  (仿真里整车总重)' % ('合计', s['total_mass']))
    print('  planar_move自转保真: %.3f  (插件固有: 实际自转 ≈ 指令 × 该系数, 见 README)\n'
          % s['wz_gain'])
    for line in s['sensors']:
        print('  %s' % line)
    print('=' * 74)

    # 静止时四个轮速都应为 0, 原地左转应满足左轮反转/右轮正转
    w = kin.inverse(0.0, 0.0, 1.0)
    print('  自检 原地左转 wz=1.0 -> 轮速 [%.2f %.2f %.2f %.2f] (FL FR RL RR)'
          % w)
    ok = (w[0] < 0 and w[1] > 0 and w[2] < 0 and w[3] > 0)
    print('  自检 左轮反转/右轮正转: %s' % ('PASS' if ok else 'FAIL'))
    print('=' * 74)

    if not args.dry:
        os.makedirs(os.path.dirname(args.out), exist_ok=True)
        with open(args.out, 'w') as fp:
            fp.write(xml)
        print('  -> %s  (%d 字节)' % (args.out, len(xml)))
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
