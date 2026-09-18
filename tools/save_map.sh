#!/usr/bin/env bash
# 保存当前建图结果 (★ 建图的 launch 必须还开着, Ctrl+C 之后地图就没了)
#
#   bash tools/save_map.sh                    # -> maps/map_20260917_0330.pgm/.yaml
#   bash tools/save_map.sh karto_perfect      # -> maps/karto_perfect.pgm/.yaml
#
# 存完会自动跑一次地图打分 (tools/check_map_quality.py)
set -u
WS="$(cd "$(dirname "$0")/.." && pwd)"
NAME="${1:-map_$(date +%Y%m%d_%H%M)}"
source /opt/ros/noetic/setup.bash
[ -f "$WS/devel/setup.bash" ] && source "$WS/devel/setup.bash"

# 先看看 /map 在不在 —— 不在就说明建图没开 (或者已经 Ctrl+C 了)
if ! rostopic list 2>/dev/null | grep -qx "/map"; then
  echo "✗ 没有 /map 话题。先开着建图再存:"
  echo "    roslaunch competition_robot slam_karto.launch     (或 slam_gmapping.launch)"
  exit 1
fi
# 地图里得有东西再存, 不然存出来是空的
n=$(timeout 10 rostopic echo -n1 /map/info/width 2>/dev/null | head -1)
[ -n "$n" ] && echo "  当前地图宽度: $n 格"

mkdir -p "$WS/maps"
OUT="$WS/maps/$NAME"
rm -f "$OUT.pgm" "$OUT.yaml"
echo "  存到 $OUT.pgm ..."
if ! rosrun map_server map_saver -f "$OUT" >/dev/null 2>&1; then
  echo "✗ map_saver 失败 (map_server 装了吗? sudo apt install ros-noetic-map-server)"
  exit 1
fi

# yaml 里的 image 改成相对文件名, 这样整个 maps/ 目录可以随便拷贝/改名
python3 - "$OUT.yaml" <<'PY'
import sys
p = sys.argv[1]
out = []
for line in open(p):
    if line.startswith('image:'):
        line = 'image: %s\n' % p.rsplit('/', 1)[-1].replace('.yaml', '.pgm')
    out.append(line)
open(p, 'w').writelines(out)
PY

echo "  ✓ 已保存:"
echo "      $OUT.pgm"
echo "      $OUT.yaml"
echo "    以后要用这张图 (定位/导航):"
echo "      rosrun map_server map_server $OUT.yaml"
echo
python3 "$WS/tools/check_map_quality.py" "$OUT.pgm"
