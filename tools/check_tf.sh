#!/usr/bin/env bash
WS="$(cd "$(dirname "$0")/.." && pwd)"
source /opt/ros/noetic/setup.bash; source "$WS/devel/setup.bash"
export HOME="$WS/.sim_home"; mkdir -p "$HOME/.ros" "$HOME/.gazebo"
export ROS_LOG_DIR="$HOME/.ros/log"; export DISPLAY=:0
export LIBGL_ALWAYS_SOFTWARE=1
pkill -9 -f "[g]zserver"; pkill -9 -f "[r]osmaster"; sleep 2
mkdir -p "$WS/.verify"
export ROS_MASTER_URI=http://localhost:11610
export GAZEBO_MASTER_URI=http://localhost:11620
roslaunch --port=11610 competition_robot robot_gazebo.launch gui:=false rviz:=false > "$WS/.verify/tf.log" 2>&1 & LP=$!
sleep 40

echo "=== /joint_states 内容 ==="
timeout 8 rostopic echo -n1 /joint_states 2>/dev/null | head -20

echo
echo "=== TF 树 (谁发谁) ==="
timeout 12 rostopic echo /tf 2>/dev/null | grep -E "frame_id:|child_frame_id:" | paste - - | sort -u | head -20

echo
echo "=== /tf_static ==="
timeout 12 rostopic echo /tf_static 2>/dev/null | grep -E "frame_id:|child_frame_id:" | paste - - | sort -u | head -20

echo
echo "=== TF 相关警告 ==="
grep -iE "tf|transform|extrapolat|old data|queue" "$WS/.verify/tf.log" | grep -viE "rosparam|/tf:" | head -12

kill -INT $LP 2>/dev/null; sleep 3; kill -9 $LP 2>/dev/null
pkill -9 -f "[g]zserver"; pkill -9 -f "[r]osmaster"
echo "=== done ==="
