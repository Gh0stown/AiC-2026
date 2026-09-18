#!/usr/bin/env bash
WS="$(cd "$(dirname "$0")/.." && pwd)"
source /opt/ros/noetic/setup.bash; source "$WS/devel/setup.bash"
export HOME="$WS/.sim_home"; mkdir -p "$HOME/.ros" "$HOME/.gazebo"
export ROS_LOG_DIR="$HOME/.ros/log"; export DISPLAY=:0
export LIBGL_ALWAYS_SOFTWARE=1
pkill -9 -f "[g]zserver"; pkill -9 -f "[r]osmaster"; sleep 2
mkdir -p "$WS/.verify"
export ROS_MASTER_URI=http://localhost:11530
export GAZEBO_MASTER_URI=http://localhost:11540
roslaunch --port=11530 competition_robot robot_gazebo.launch gui:=false rviz:=false > "$WS/.verify/mesh.log" 2>&1 & LP=$!
sleep 40
kill -0 $LP 2>/dev/null || { echo "启动失败"; tail -20 "$WS/.verify/mesh.log"; exit 1; }

echo "=== 模型是否成功 spawn ==="
timeout 8 rosservice call /gazebo/get_world_properties 2>/dev/null | tr ',' '\n' | grep -oE "competition_robot" | head -1 | sed 's/^/  /'

echo "=== 加载 mesh 有没有报错 ==="
grep -iE "mesh|STL|collada|assimp|cannot|unable|error" "$WS/.verify/mesh.log" | grep -viE "rospack|dconf" | head -8 || echo "  没有 mesh 相关错误 ✅"

echo "=== TF 树 (检查 base_footprint 是否只有一个父节点) ==="
timeout 12 rostopic echo /tf 2>/dev/null | grep -E "frame_id:|child_frame_id:" | paste - - | sort -u | sed 's/^/  /'

echo "=== 各 link 是否都有 TF ==="
timeout 10 rosrun tf tf_monitor 2>/dev/null | grep -E "^Frame:" | sed 's/^/  /' | head -12

echo "=== 话题 ==="
for t in /cmd_vel /odom /scan /points /imu /camera/depth/image_raw; do
  rostopic list 2>/dev/null | grep -qx "$t" && echo "  [OK] $t" || echo "  [缺失] $t"
done

kill -INT $LP 2>/dev/null; sleep 3; kill -9 $LP 2>/dev/null
pkill -9 -f "[g]zserver"; pkill -9 -f "[r]osmaster"
echo "=== done ==="
