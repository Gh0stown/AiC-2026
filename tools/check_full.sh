#!/usr/bin/env bash
WS="$(cd "$(dirname "$0")/.." && pwd)"
source /opt/ros/noetic/setup.bash; source "$WS/devel/setup.bash"
[ -n "${DSH_SIM_HOME:-}" ] && { export HOME="$DSH_SIM_HOME"; mkdir -p "$HOME/.ros"; export LIBGL_ALWAYS_SOFTWARE=1; }
export ROS_LOG_DIR="${HOME}/.ros/log"; export DISPLAY=:0
pkill -9 -f "[g]zserver"; pkill -9 -f "[r]osmaster"; sleep 2
mkdir -p "$WS/.verify"
export ROS_MASTER_URI=http://localhost:11950
export GAZEBO_MASTER_URI=http://localhost:11960
roslaunch --port=11950 competition_robot slam_gmapping.launch rviz:=false > "$WS/.verify/full.log" 2>&1 & LP=$!
sleep 45
kill -0 $LP 2>/dev/null || { echo "启动失败"; tail -20 "$WS/.verify/full.log"; exit 1; }
echo "=== 关键话题 ==="
for t in /map /scan /odom /points /camera/rgb/image_raw /camera/depth/image_raw /cmd_vel; do
  if rostopic list 2>/dev/null | grep -qx "$t"; then
    ty=$(rostopic type $t 2>/dev/null)
    r=$(timeout 6 rostopic hz --window=5 $t 2>/dev/null | grep -m1 "average rate" | grep -oE "[0-9.]+$")
    echo "  [有] $t  ($ty)  ${r:-无数据} Hz"
  else
    echo "  [无] $t"
  fi
done
echo "=== 相机图像内容 ==="
timeout 10 rostopic echo -n1 /camera/rgb/image_raw 2>/dev/null | head -8 | sed 's/^/  /'
echo "=== TF 树里有没有 map ==="
timeout 10 rostopic echo /tf 2>/dev/null | grep -E "frame_id:|child_frame_id:" | paste - - | sort -u | sed 's/^/  /'
echo "=== 发指令车动不动 ==="
timeout 8 rosservice call /gazebo/get_model_state "{model_name: 'competition_robot'}" 2>/dev/null | grep -A3 position | head -4 | sed 's/^/  前: /'
timeout 6 rostopic pub -r 20 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null 2>&1
sleep 1
timeout 8 rosservice call /gazebo/get_model_state "{model_name: 'competition_robot'}" 2>/dev/null | grep -A3 position | head -4 | sed 's/^/  后: /'
kill -INT $LP 2>/dev/null; sleep 3; kill -9 $LP 2>/dev/null
pkill -9 -f "[g]zserver"; pkill -9 -f "[r]osmaster"
echo "=== done ==="
