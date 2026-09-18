#!/usr/bin/env bash
WS="$(cd "$(dirname "$0")/.." && pwd)"
source /opt/ros/noetic/setup.bash; source "$WS/devel/setup.bash"
[ -n "${DSH_SIM_HOME:-}" ] && { export HOME="$DSH_SIM_HOME"; mkdir -p "$HOME/.ros"; export LIBGL_ALWAYS_SOFTWARE=1; }
export ROS_LOG_DIR="${HOME}/.ros/log"; export DISPLAY=:0
pkill -9 -f "[g]zserver"; pkill -9 -f "[r]osmaster"; sleep 2
mkdir -p "$WS/.verify"
export ROS_MASTER_URI=http://localhost:11710
export GAZEBO_MASTER_URI=http://localhost:11720
roslaunch --port=11710 competition_robot robot_gazebo.launch gui:=false rviz:=false > "$WS/.verify/lt.log" 2>&1 & LP=$!
sleep 40
echo "=== 话题消息类型 (RViz 显示类要匹配这个) ==="
rostopic type /points 2>/dev/null | sed 's/^/  \/points -> /'
rostopic type /scan   2>/dev/null | sed 's/^/  \/scan   -> /'
echo
echo "=== 对应的 RViz 显示类 ==="
echo "  sensor_msgs/PointCloud  -> rviz/PointCloud   (老的, Gazebo block_laser 发的)"
echo "  sensor_msgs/LaserScan   -> rviz/LaserScan"
echo
echo "=== 两个话题都在发吗 ==="
for t in /points /scan; do
  r=$(timeout 8 rostopic hz --window=8 $t 2>/dev/null | grep -m1 "average rate" | grep -oE "[0-9.]+$")
  printf "  %-9s %s Hz\n" "$t" "${r:-无数据}"
done
kill -INT $LP 2>/dev/null; sleep 3; kill -9 $LP 2>/dev/null
pkill -9 -f "[g]zserver"; pkill -9 -f "[r]osmaster"
echo "=== done ==="
