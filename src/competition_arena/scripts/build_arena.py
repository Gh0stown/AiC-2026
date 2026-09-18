#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
build_arena.py -- 由俯视平面图 (map_source.jpg) 生成 Gazebo 比赛场地。

输入:  一张黑白俯视地图 (白色=墙体/障碍, 黑色=可通行区域)
输出:
  materials/textures/arena_floor.png   Gazebo 地面贴图 (裁剪到场地边界)
  maps/arena_map.pgm / .yaml           ROS 栅格地图 (白=可通行, 黑=障碍)
  worlds/competition_arena.world       Gazebo 世界文件
  tools/arena_geometry.json            提取出的几何数据 (便于二次开发)

用法:
  python3 scripts/build_arena.py                       # 默认: 场地外沿 4.2m, 墙高 0.5m
  python3 scripts/build_arena.py --arena 4.2 --wall-height 0.5
  python3 scripts/build_arena.py --stripes raised --stripe-height 0.03
"""
import argparse
import json
import os
import sys

import numpy as np
from PIL import Image

# ----------------------------------------------------------------------------- paths
HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
SRC_IMG = os.path.join(PKG, 'tools', 'map_source.jpg')

# Outer edge of the drawn arena wall in source-image pixels.
# Left x=29, right x=1251 (inclusive) -> exclusive edge 1252
# Top  y=31, bottom y=1253 (inclusive) -> exclusive edge 1254
ARENA_PX = (29, 31, 1252, 1254)          # (x0, y0, x1_excl, y1_excl)
EDGE_TRIM = 3                            # strip the 2px figure-border artefact


# ----------------------------------------------------------------------------- geometry extraction
def load_mask(path):
    gray = np.asarray(Image.open(path).convert('L'))
    mask = gray > 128
    mask[:EDGE_TRIM, :] = False
    mask[-EDGE_TRIM:, :] = False
    mask[:, :EDGE_TRIM] = False
    mask[:, -EDGE_TRIM:] = False
    return mask


def row_runs(row):
    idx = np.flatnonzero(row)
    if idx.size == 0:
        return []
    out, start, prev = [], idx[0], idx[0]
    for v in idx[1:]:
        if v == prev + 1:
            prev = v
        else:
            out.append((int(start), int(prev)))
            start = prev = v
    out.append((int(start), int(prev)))
    return out


def vectorize(mask):
    """Merge horizontal runs down consecutive rows -> list of axis-aligned rects."""
    rects, open_rects = [], []
    for y in range(mask.shape[0]):
        cur = row_runs(mask[y])
        new_open, used = [], set()
        for (x0, x1) in cur:
            hit = None
            for i, r in enumerate(open_rects):
                if i not in used and abs(r[0] - x0) <= 1 and abs(r[1] - x1) <= 1:
                    hit = i
                    break
            if hit is not None:
                used.add(hit)
                r = open_rects[hit]
                r[0] = min(r[0], x0)
                r[1] = max(r[1], x1)
                r[3] = y
                new_open.append(r)
            else:
                new_open.append([x0, x1, y, y])
        for i, r in enumerate(open_rects):
            if i not in used:
                rects.append(tuple(r))
        open_rects = new_open
    rects.extend(tuple(r) for r in open_rects)
    return [r for r in rects if (r[1] - r[0] + 1) >= 5 and (r[3] - r[2] + 1) >= 5]


def _run(mask, fixed, start, step, limit, axis):
    """Length of the consecutive bright run starting at `start` (inclusive)."""
    n, i = 0, start
    while (i < limit) if step > 0 else (i >= limit):
        if not (mask[i, fixed] if axis == 'col' else mask[fixed, i]):
            break
        n += 1
        i += step
    return n


def boundary_rects(mask, arena_px):
    """The 4 rectangles forming the outer wall frame only.

    Each side's thickness is measured from the drawing (median over many scan
    lines), so this adapts to a different map instead of being hard-coded.
    """
    x0, y0, x1, y1 = arena_px
    xs = list(range(x0 + 60, x1 - 60, 30))
    ys = list(range(y0 + 60, y1 - 60, 30))
    tt = int(np.median([_run(mask, x, y0, +1, y1, 'col') for x in xs]))
    tb = int(np.median([_run(mask, x, y1 - 1, -1, y0, 'col') for x in xs]))
    tl = int(np.median([_run(mask, y, x0, +1, x1, 'row') for y in ys]))
    tr = int(np.median([_run(mask, y, x1 - 1, -1, x0, 'row') for y in ys]))
    return [(x0, x1 - 1, y0, y0 + tt - 1),          # top
            (x0, x1 - 1, y1 - tb, y1 - 1),          # bottom
            (x0, x0 + tl - 1, y0, y1 - 1),          # left
            (x1 - tr, x1 - 1, y0, y1 - 1)]          # right


def classify(rects):
    """Split extracted rectangles into solid walls vs. the two striped floor patches."""
    walls, top_stripes, mid_stripes = [], [], []
    for r in rects:
        x0, x1, y0, y1 = r
        w, h = x1 - x0 + 1, y1 - y0 + 1
        # top patch: 6 wide/short bars around x 658..718, y 42..206
        if 640 <= x0 and x1 <= 740 and y0 >= 38 and y1 <= 212 and w > h * 2:
            top_stripes.append(r)
        # middle patch: 6 tall/narrow bars around x 525..690, y 809..869
        elif 500 <= x0 and x1 <= 700 and 800 <= y0 and y1 <= 880 and h > w * 2:
            mid_stripes.append(r)
        else:
            walls.append(r)
    return walls, top_stripes, mid_stripes


# ----------------------------------------------------------------------------- px -> metre
class Frame(object):
    def __init__(self, arena_px, arena_m):
        self.x0, self.y0, self.x1, self.y1 = arena_px
        self.size = float(arena_m)
        self.scale = self.size / (self.x1 - self.x0)      # metres per pixel
        self.half = self.size / 2.0

    def px2x(self, px):
        return (px - self.x0) * self.scale - self.half

    def px2y(self, py):
        # image y grows downward, Gazebo +y is "up" on a top-down view
        return self.half - (py - self.y0) * self.scale

    def rect(self, r):
        """(x0,x1,y0,y1) inclusive px  ->  (cx, cy, sx, sy) metres."""
        x0, x1, y0, y1 = r
        xa, xb = self.px2x(x0), self.px2x(x1 + 1)
        ya, yb = self.px2y(y0), self.px2y(y1 + 1)
        return ((xa + xb) / 2.0, (ya + yb) / 2.0, abs(xb - xa), abs(yb - ya))


# ----------------------------------------------------------------------------- assets
def write_floor_texture(img, frame, out_png, rotation_deg=90):
    """Crop the map to the arena box and pre-rotate it.

    Gazebo maps a box's top-face UV so that texture +u runs along world -Y and
    texture +v along world -X (verified by rendering), while the walls are placed
    with source +x -> world +X and source +y -> world -Y.  Rotating the texture
    90 deg counter-clockwise reconciles the two, so the floor image lines up
    exactly under the extruded wall geometry.
    """
    crop = img.crop((frame.x0, frame.y0, frame.x1, frame.y1))
    crop.save(out_png.replace('.png', '_unrotated_reference.png'))
    k = int(round(rotation_deg / 90.0)) % 4
    if k:
        crop = crop.transpose([Image.ROTATE_90, Image.ROTATE_180, Image.ROTATE_270][k - 1])
    crop.save(out_png)
    return crop


def write_ros_map(mask, frame, res, out_pgm, out_yaml, pad=8):
    """ROS map_server map.  White(254)=free, Black(0)=occupied."""
    x0, y0, x1, y1 = frame.x0, frame.y0, frame.x1, frame.y1
    aria = mask[y0:y1, x0:x1]
    n = int(round(frame.size / res))                      # target cells per side
    # area-average downsample then threshold: thin walls stay solid, never vanish
    small = np.asarray(
        Image.fromarray((aria * 255).astype(np.uint8), 'L')
        .resize((n, n), Image.BOX), dtype=np.float32) / 255.0
    occ = small > 0.25                                    # any real coverage -> occupied

    grid = np.full((n + 2 * pad, n + 2 * pad), 254, np.uint8)
    grid[pad:pad + n, pad:pad + n] = np.where(occ, 0, 254)
    grid[:pad, :] = 0
    grid[-pad:, :] = 0
    grid[:, :pad] = 0
    grid[:, -pad:] = 0
    Image.fromarray(grid, 'L').save(out_pgm)

    origin_x = -frame.size / 2.0 - pad * res
    origin_y = -frame.size / 2.0 - pad * res
    with open(out_yaml, 'w') as f:
        f.write('image: arena_map.pgm\n')
        f.write('resolution: %.6f\n' % res)
        f.write('origin: [%.6f, %.6f, 0.000000]\n' % (origin_x, origin_y))
        f.write('negate: 0\n')
        f.write('occupied_thresh: 0.65\n')
        f.write('free_thresh: 0.196\n')
    return grid.shape


# ----------------------------------------------------------------------------- world
WORLD_TMPL = '''<?xml version="1.0" ?>
<!--
  4.2 m x 4.2 m 比赛场地  (auto-generated by scripts/build_arena.py)
  场地外沿尺寸 : {arena} x {arena} m
  墙体高度     : {wall_h} m
  墙体厚度     : 由原图线条宽度决定 (约 {wall_mm:.1f} mm)
  墙体模式     : {wall_mode}
  地面贴图     : 原图裁剪, 逆时针预旋转 {tex_rot} deg 以匹配 Gazebo 的 box UV
  坐标系       : 场地中心为原点, +x 向右, +y 向上 (俯视), z=0 为地面
-->
<sdf version="1.6">
  <world name="competition_arena">

    <include><uri>model://sun</uri></include>

    <!-- ================= 场地地面 (使用比赛地图贴图) ================= -->
    <model name="arena_floor">
      <static>true</static>
      <pose>0 0 {floor_z} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{arena} {arena} {floor_t}</size></box></geometry>
          <surface>
            <friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction>
            <contact><ode><kp>1e10</kp><kd>1</kd></ode></contact>
          </surface>
        </collision>
        <visual name="visual">
          <geometry><box><size>{arena} {arena} {floor_t}</size></box></geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/arena.material</uri>
              <name>Arena/Floor</name>
            </script>
          </material>
        </visual>
      </link>
    </model>

    <!-- ================= 场地四周 (场地外的大地面, 压低 2mm 避免与场地贴图 z-fighting) ================= -->
    <include>
      <uri>model://ground_plane</uri>
      <pose>0 0 -0.002 0 0 0</pose>
    </include>

    <!-- ================= 墙体 ================= -->
    <model name="arena_walls">
      <static>true</static>
      <link name="walls">
{wall_geom}
      </link>
    </model>
{stripe_model}
  </world>
</sdf>
'''

WALL_GEOM_TMPL = '''        <collision name="{name}">
          <pose>{cx:.6f} {cy:.6f} {cz:.6f} 0 0 0</pose>
          <geometry><box><size>{sx:.6f} {sy:.6f} {sz:.6f}</size></box></geometry>
        </collision>
        <visual name="{name}">
          <pose>{cx:.6f} {cy:.6f} {cz:.6f} 0 0 0</pose>
          <geometry><box><size>{sx:.6f} {sy:.6f} {sz:.6f}</size></box></geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/White</name>
            </script>
          </material>
        </visual>'''

STRIPE_MODEL_TMPL = '''
    <!-- 条纹区域 (原图中的斜纹/条纹块) 以 {h} m 高的低矮凸起建模 -->
    <model name="floor_stripes">
      <static>true</static>
      <link name="stripes">
{geom}
      </link>
    </model>'''


def build_world(walls, stripes, frame, args):
    parts = []
    for i, r in enumerate(walls):
        cx, cy, sx, sy = frame.rect(r)
        parts.append(WALL_GEOM_TMPL.format(
            name='wall_%02d' % i, cx=cx, cy=cy, cz=args.wall_height / 2.0,
            sx=sx, sy=sy, sz=args.wall_height))
    wall_geom = '\n'.join(parts)

    stripe_model = ''
    if args.stripes == 'raised':
        parts = []
        for i, r in enumerate(stripes):
            cx, cy, sx, sy = frame.rect(r)
            parts.append(WALL_GEOM_TMPL.format(
                name='stripe_%02d' % i, cx=cx, cy=cy, cz=args.stripe_height / 2.0,
                sx=sx, sy=sy, sz=args.stripe_height))
        stripe_model = STRIPE_MODEL_TMPL.format(h=args.stripe_height,
                                                geom='\n'.join(parts))

    widths = [min(r[1] - r[0] + 1, r[3] - r[2] + 1) for r in walls]
    return WORLD_TMPL.format(
        arena='%.3f' % args.arena,
        wall_h='%.3f' % args.wall_height,
        wall_mm=float(np.median(widths)) * frame.scale * 1000.0,
        wall_mode=('仅外沿围墙' if args.walls == 'boundary' else '全部墙体 (含内部隔墙)'),
        tex_rot=args.texture_rotation % 360,
        floor_t='%.4f' % args.floor_thickness,
        floor_z='%.5f' % (-args.floor_thickness / 2.0),
        wall_geom=wall_geom,
        stripe_model=stripe_model)


MATERIAL_TMPL = '''// Auto-generated: 比赛场地地面贴图
material Arena/Floor
{
  technique
  {
    pass
    {
      ambient 1 1 1 1
      diffuse 1 1 1 1
      specular 0 0 0 0 0
      texture_unit
      {
        texture arena_floor.png
        filtering anisotropic
        max_anisotropy 8
      }
    }
  }
}
'''


# ----------------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description='由平面图生成 Gazebo 比赛场地')
    ap.add_argument('--arena', type=float, default=4.2,
                    help='场地尺寸 (m), 默认 4.2  ->  4.2 x 4.2')
    ap.add_argument('--wall-height', type=float, default=0.5, help='墙高 (m), 默认 0.5')
    ap.add_argument('--floor-thickness', type=float, default=0.02, help='地板厚度 (m)')
    ap.add_argument('--stripes', choices=['flat', 'raised'], default='flat',
                    help='地面条纹块: flat=仅贴图; raised=做成低矮凸起')
    ap.add_argument('--stripe-height', type=float, default=0.03, help='条纹凸起高度 (m)')
    ap.add_argument('--map-resolution', type=float, default=0.01, help='ROS 栅格地图分辨率 (m/cell)')
    ap.add_argument('--walls', choices=['boundary', 'all'], default='boundary',
                    help='boundary=只做外沿围墙 (默认); all=把图中所有线条都做成墙')
    ap.add_argument('--texture-rotation', type=int, default=90, choices=[0, 90, 180, 270],
                    help='地面贴图逆时针预旋转角度, 用于对齐 Gazebo 的 box UV (默认 90)')
    args = ap.parse_args()

    if not os.path.isfile(SRC_IMG):
        sys.exit('找不到原始地图: %s' % SRC_IMG)

    img = Image.open(SRC_IMG).convert('RGB')
    mask = load_mask(SRC_IMG)
    rects = vectorize(mask)
    walls, top_s, mid_s = classify(rects)
    if args.walls == 'boundary':
        walls = boundary_rects(mask, ARENA_PX)
    frame = Frame(ARENA_PX, args.arena)

    # ---- sanity: reconstruction quality
    recon = np.zeros_like(mask)
    for (x0, x1, y0, y1) in rects:
        recon[y0:y1 + 1, x0:x1 + 1] = True
    iou = (recon & mask).sum() / float((recon | mask).sum())

    print('=' * 66)
    print('  原始图片      : %s  %s' % (os.path.basename(SRC_IMG), img.size))
    print('  场地像素范围  : x[%d,%d) y[%d,%d)  = %d x %d px'
          % (frame.x0, frame.x1, frame.y0, frame.y1, frame.x1 - frame.x0, frame.y1 - frame.y0))
    print('  比例尺        : %.4f mm / px' % (frame.scale * 1000))
    print('  场地尺寸      : %.3f x %.3f m (外沿)' % (args.arena, args.arena))
    print('  墙体模式      : %s' % args.walls)
    print('  提取矩形      : %d 个 (墙 %d, 顶部条纹 %d, 中部条纹 %d)'
          % (len(rects), len(walls), len(top_s), len(mid_s)))
    print('  矢量化还原 IoU: %.4f' % iou)
    print('  墙体厚度      : %.1f mm' % (np.median(
        [min(r[1] - r[0] + 1, r[3] - r[2] + 1) for r in walls]) * frame.scale * 1000))
    print('=' * 66)

    # ---- write assets
    # Gazebo only auto-registers <pkg>/media/materials/{scripts,textures} on OGRE's
    # resource path, so the OGRE assets must live under media/materials/.
    tex_dir = os.path.join(PKG, 'media', 'materials', 'textures')
    scr_dir = os.path.join(PKG, 'media', 'materials', 'scripts')
    os.makedirs(tex_dir, exist_ok=True)
    os.makedirs(scr_dir, exist_ok=True)
    floor_png = os.path.join(tex_dir, 'arena_floor.png')
    write_floor_texture(img, frame, floor_png, args.texture_rotation)
    with open(os.path.join(scr_dir, 'arena.material'), 'w') as f:
        f.write(MATERIAL_TMPL)

    os.makedirs(os.path.join(PKG, 'maps'), exist_ok=True)
    shape = write_ros_map(mask, frame, args.map_resolution,
                          os.path.join(PKG, 'maps', 'arena_map.pgm'),
                          os.path.join(PKG, 'maps', 'arena_map.yaml'))

    world = build_world(walls, top_s + mid_s, frame, args)
    world_path = os.path.join(PKG, 'worlds', 'competition_arena.world')
    os.makedirs(os.path.dirname(world_path), exist_ok=True)
    with open(world_path, 'w') as f:
        f.write(world)

    geo = {
        'arena_m': args.arena,
        'scale_m_per_px': frame.scale,
        'arena_px': ARENA_PX,
        'wall_height_m': args.wall_height,
        'stripes_mode': args.stripes,
        'walls_px': walls,
        'walls_m': [frame.rect(r) for r in walls],
        'top_stripes_px': top_s,
        'mid_stripes_px': mid_s,
        'reconstruction_iou': iou,
    }
    with open(os.path.join(PKG, 'tools', 'arena_geometry.json'), 'w') as f:
        json.dump(geo, f, indent=1)

    print('  -> media/materials/textures/arena_floor.png')
    print('  -> media/materials/scripts/arena.material')
    print('  -> maps/arena_map.pgm  %s' % (shape,))
    print('  -> %s' % os.path.relpath(world_path, PKG))
    print('  -> tools/arena_geometry.json')


if __name__ == '__main__':
    main()
