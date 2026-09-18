#!/usr/bin/env bash
# 端到端验证: 出生点 -> gmapping 建图 -> 地图有没有长出来
WS="$(cd "$(dirname "$0")/.." && pwd)"
source /opt/ros/noetic/setup.bash
source "$WS/devel/setup.bash"
[ -n "${DSH_SIM_HOME:-}" ] && { export HOME="$DSH_SIM_HOME"; mkdir -p "$HOME/.ros"; export LIBGL_ALWAYS_SOFTWARE=1; }
export ROS_LOG_DIR="${HOME}/.ros/log"; export DISPLAY=:0
pkill -9 -f "[g]zserver"; pkill -9 -f "[r]osmaster"; sleep 2
mkdir -p "$WS/.verify"
export ROS_MASTER_URI=http://localhost:11810
export GAZEBO_MASTER_URI=http://localhost:11820

roslaunch --port=11810 competition_robot robot_gazebo.launch gui:=false rviz:=false > "$WS/.verify/map_sim.log" 2>&1 & SIM=$!
sleep 40
kill -0 $SIM 2>/dev/null || { echo "仿真没起来"; tail -15 "$WS/.verify/map_sim.log"; exit 1; }
echo "仿真 OK"

echo "=== 出生点 ==="
timeout 6 rosservice call /gazebo/get_model_state "{model_name: 'competition_robot'}" 2>/dev/null \
 | grep -A5 "^pose:" | grep -E "x:|y:" | head -2 | sed 's/^/  /'

echo "=== 启动 gmapping ==="
rosrun gmapping slam_gmapping _base_frame:=base_footprint _odom_frame:=odom \
  _map_frame:=map _delta:=0.05 _maxUrange:=6.0 _particles:=30 \
  _xmin:=-5.0 _xmax:=5.0 _ymin:=-5.0 _ymax:=5.0 \
  _linearUpdate:=0.05 _angularUpdate:=0.05 \
  > "$WS/.verify/gmapping.log" 2>&1 & GM=$!
sleep 12
kill -0 $GM 2>/dev/null && echo "  gmapping 在跑 ✅" || { echo "  gmapping 挂了:"; tail -10 "$WS/.verify/gmapping.log"; }

echo "=== 让车走一圈 (先原地转, 再往前走) ==="
timeout 8 rostopic pub -r 20 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.7}}" >/dev/null 2>&1
timeout 16 rostopic pub -r 20 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.35, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null 2>&1
sleep 1
timeout 12 rostopic pub -r 20 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.9}}" >/dev/null 2>&1
sleep 3

echo "=== /map 内容 ==="
timeout 40 python3 "$WS/tools/check_map_content.py" 2>/dev/null | sed 's/^/  /'

echo "=== TF: map -> base_footprint ==="
timeout 10 rosrun tf tf_echo map base_footprint 2>/dev/null | head -6 | sed 's/^/  /'

kill -INT $GM 2>/dev/null; kill -INT $SIM 2>/dev/null; sleep 3
kill -9 $GM $SIM 2>/dev/null
pkill -9 -f "[g]zserver"; pkill -9 -f "[r]osmaster"
echo "=== done ==="
