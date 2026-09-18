#!/usr/bin/env bash
# Verify that the floor texture and the 3-D wall geometry agree with the source map.
#
#   usage: verify_alignment.sh
#
# Renders the arena twice with the same camera -- once with only the floor texture
# visible, once with only the walls visible -- and scores both against the source
# map's wall mask over all 8 axis-aligned orientations.  If both pick the SAME
# orientation, the texture sits exactly under the extruded walls.
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
WS="$(cd "$HERE/.." && pwd)"
PKG="$WS/src/competition_arena"
V="$WS/.verify"
mkdir -p "$V"

python3 - "$PKG/worlds/competition_arena.world" "$V" <<'PY'
import sys, re
world, out = sys.argv[1], sys.argv[2]
w = open(world).read()
open(out + '/flooronly.world', 'w').write(
    re.sub(r'<model name="arena_walls">.*?</model>', '', w, flags=re.S))
open(out + '/wallsonly.world', 'w').write(
    w.replace('<name>Arena/Floor</name>', '<name>Gazebo/Black</name>'))
PY

bash "$HERE/render_topdown.sh" "$V/flooronly.world" "$V/floor.png" 30.0 0.15
bash "$HERE/render_topdown.sh" "$V/wallsonly.world" "$V/walls.png" 30.0 0.15

python3 - "$PKG" "$V" <<'PY'
import sys
import numpy as np
from PIL import Image
PKG, V = sys.argv[1], sys.argv[2]
ARENA_PX = (29, 31, 1252, 1254); N = 900; FOV = 0.15; CZ = 30.0

g = np.asarray(Image.open(PKG + '/tools/map_source.jpg').convert('L'))
src = g[ARENA_PX[1]:ARENA_PX[3], ARENA_PX[0]:ARENA_PX[2]] > 128
ref = np.asarray(Image.fromarray((src * 255).astype(np.uint8)).resize((N, N), Image.BILINEAR)) > 100

def load(p):
    a = np.asarray(Image.open(p).convert('L')).astype(np.float32)
    H, W = a.shape
    half = (4.2 / (2 * CZ * np.tan(FOV / 2))) * W / 2
    c = W / 2
    x0, x1 = int(round(c - half)), int(round(c + half))
    m = a[x0:x1, x0:x1] > 60
    return np.asarray(Image.fromarray((m * 255).astype(np.uint8)).resize((N, N), Image.BILINEAR)) > 100

def dil(m, k):
    o = m.copy()
    for dy in range(-k, k + 1):
        for dx in range(-k, k + 1):
            o |= np.roll(np.roll(m, dy, 0), dx, 1)
    return o
ref_d = dil(ref, 5)

names = ['identity', 'rot90', 'rot180', 'rot270', 'flipv', 'flipv+rot90', 'flipv+rot180', 'flipv+rot270']
def variants(m):
    return [m, np.rot90(m, 1), np.rot90(m, 2), np.rot90(m, 3), m[::-1, :],
            np.rot90(m[::-1, :], 1), np.rot90(m[::-1, :], 2), np.rot90(m[::-1, :], 3)]

def best(m):
    rows = []
    for nm, v in zip(names, variants(m)):
        p = (v & ref_d).sum() / max(v.sum(), 1)
        r = (ref & dil(v, 5)).sum() / max(ref.sum(), 1)
        rows.append((2 * p * r / max(p + r, 1e-9), nm))
    return sorted(rows, reverse=True)

bf = best(load(V + '/floor.png'))
bw = best(load(V + '/walls.png'))
print('\n=== floor texture vs source map ===')
for f, nm in bf[:3]: print('   %-16s F1 = %.3f' % (nm, f))
print('=== wall geometry vs source map ===')
for f, nm in bw[:3]: print('   %-16s F1 = %.3f' % (nm, f))
ok = bf[0][1] == bw[0][1]
print('\n>>> %s   (floor: %s @ %.3f, walls: %s @ %.3f)' % (
    'ALIGNED' if ok else 'MISALIGNED -- texture and walls disagree!',
    bf[0][1], bf[0][0], bw[0][1], bw[0][0]))
sys.exit(0 if ok else 1)
PY
