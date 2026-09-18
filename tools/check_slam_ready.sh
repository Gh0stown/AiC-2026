#!/usr/bin/env bash
WS="$(cd "$(dirname "$0")/.." && pwd)"
source /opt/ros/noetic/setup.bash; source "$WS/devel/setup.bash"
[ -n "${DSH_SIM_HOME:-}" ] && { export HOME="$DSH_SIM_HOME"; mkdir -p "$HOME/.ros"; export LIBGL_ALWAYS_SOFTWARE=1; }
export ROS_LOG_DIR="${HOME}/.ros/log"; export DISPLAY=:0
pkill -9 -f "[g]zserver"; pkill -9 -f "[r]osmaster"; sleep 2
mkdir -p "$WS/.verify"
export ROS_MASTER_URI=http://localhost:11910
export GAZEBO_MASTER_URI=http://localhost:11920
roslaunch --port=11910 competition_robot robot_gazebo.launch gui:=false rviz:=false > "$WS/.verify/sr.log" 2>&1 & SIM=$!
sleep 40
echo "=== /scan 关键字段 (SLAM 依赖这些) ==="
timeout 10 rostopic echo -n1 /scan 2>/dev/null | grep -E "^(  )?(frame_id|angle_min|angle_max|angle_increment|range_min|range_max|scan_time):" | sed 's/^/  /'
echo "=== /scan 的 TF 链能不能通 (map 先不管, 看 base_footprint -> laser_link) ==="
timeout 10 rosrun tf tf_echo base_footprint laser_link 2>/dev/null | head -8 | sed 's/^/  /'
echo "=== odom -> base_footprint ==="
timeout 10 rosrun tf tf_echo odom base_footprint 2>/dev/null | head -8 | sed 's/^/  /'
echo "=== 给 SLAM 用的话题都在吗 ==="
for t in /scan /odom /tf /tf_static /points; do
  rostopic list 2>/dev/null | grep -qx "$t" && echo "  [OK] $t" || echo "  [缺失] $t"
done
echo "=== 出生点 ==="
timeout 6 rosservice call /gazebo/get_model_state "{model_name: 'competition_robot'}" 2>/dev/null | grep -A4 "^pose:" | head -5 | sed 's/^/  /'
kill -INT $SIM 2>/dev/null; sleep 3; kill -9 $SIM 2>/dev/null
pkill -9 -f "[g]zserver"; pkill -9 -f "[r]osmaster"
echo "=== done ==="
