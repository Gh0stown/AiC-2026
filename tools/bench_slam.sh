#!/usr/bin/env bash
# 同一个"2x2 m 方框"路径 + 同一套参数, 横向对比不同建图算法。
#
#   usage:  bash tools/bench_slam.sh gmapping
#           bash tools/bench_slam.sh karto
#           bash tools/bench_slam.sh gmapping 2      # 走两圈
#
# 做完会: 存图到 maps/bench_<算法>.pgm/.yaml, 并自动打分 (tools/check_map_quality.py)
# 路径刻意走场地中间的小方框, 不撞墙 —— 撞墙时雷达前方是盲区, 各家算法都会画歪,
# 那是"撞墙"造成的, 不是算法好坏, 对比时要避免。
set -u
ALGO="${1:-gmapping}"
LAPS="${2:-1}"
WS="$(cd "$(dirname "$0")/.." && pwd)"
export HOME="${DSH_SIM_HOME:-$HOME}"; mkdir -p "$HOME/.ros"
[ -n "${DSH_SIM_HOME:-}" ] && export LIBGL_ALWAYS_SOFTWARE=1
source /opt/ros/noetic/setup.bash; source "$WS/devel/setup.bash"
export ROS_LOG_DIR="$HOME/.ros/log"
LAUNCH="$WS/src/competition_robot/launch/slam_${ALGO}.launch"
[ -f "$LAUNCH" ] || { echo "没有 $LAUNCH"; exit 1; }
export ROS_MASTER_URI=http://localhost:12750 GAZEBO_MASTER_URI=http://localhost:12760
mkdir -p "$WS/.verify" "$WS/maps"
pkill -9 -f "[g]zserver"; pkill -9 -f "[r]osmaster"; sleep 2

roslaunch --port=12750 competition_robot "slam_${ALGO}.launch" gui:=false rviz:=false \
  > "$WS/.verify/bench_${ALGO}.log" 2>&1 & LP=$!
sleep 40
kill -0 $LP 2>/dev/null || { echo "启动失败"; tail -20 "$WS/.verify/bench_${ALGO}.log"; exit 1; }

drv() { timeout "$1" rostopic pub -r 20 /cmd_vel geometry_msgs/Twist \
  "{linear: {x: $2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: $3}}" >/dev/null 2>&1; }

for i in $(seq 1 "$LAPS"); do
  echo "--- 第 $i 圈 ---"
  drv 13 0.35 0.0      # ~2 m 直行
  drv 5.5 0.0 0.7      # ~90 度
  drv 13 0.35 0.0
  drv 5.5 0.0 0.7
  drv 13 0.35 0.0
  drv 5.5 0.0 0.7
  drv 13 0.35 0.0
  drv 5.5 0.0 0.7
done
sleep 3
OUT="$WS/maps/bench_${ALGO}"
timeout 25 rosrun map_server map_saver -f "$OUT" 2>&1 | tail -1
kill -INT $LP 2>/dev/null; sleep 3; kill -9 $LP 2>/dev/null
pkill -9 -f "[g]zserver"; pkill -9 -f "[r]osmaster"
echo "=== $ALGO 地图打分 ==="
python3 "$WS/tools/check_map_quality.py" "$OUT.pgm"
