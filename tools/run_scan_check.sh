#!/usr/bin/env bash
WS="$(cd "$(dirname "$0")/.." && pwd)"
source /opt/ros/noetic/setup.bash; source "$WS/devel/setup.bash"
[ -n "${DSH_SIM_HOME:-}" ] && { export HOME="$DSH_SIM_HOME"; mkdir -p "$HOME/.ros"; export LIBGL_ALWAYS_SOFTWARE=1; }
export ROS_LOG_DIR="${HOME}/.ros/log"; export DISPLAY=:0
pkill -9 -f "[g]zserver"; pkill -9 -f "[r]osmaster"; sleep 2
mkdir -p "$WS/.verify"
export ROS_MASTER_URI=http://localhost:11850
export GAZEBO_MASTER_URI=http://localhost:11860
roslaunch --port=11850 competition_robot robot_gazebo.launch gui:=false rviz:=false > "$WS/.verify/sc.log" 2>&1 & SIM=$!
sleep 40
timeout 40 python3 "$WS/tools/check_scan_geometry.py" 2>/dev/null | sed 's/^/  /'
kill -INT $SIM 2>/dev/null; sleep 2; kill -9 $SIM 2>/dev/null
pkill -9 -f "[g]zserver"; pkill -9 -f "[r]osmaster"
