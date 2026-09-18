#!/usr/bin/env bash
WS="$(cd "$(dirname "$0")/.." && pwd)"
source /opt/ros/noetic/setup.bash; source "$WS/devel/setup.bash"
[ -n "${DSH_SIM_HOME:-}" ] && { export HOME="$DSH_SIM_HOME"; mkdir -p "$HOME/.ros";
  export LIBGL_ALWAYS_SOFTWARE=1; }   # 只在无 GPU 的沙箱里生效
export ROS_LOG_DIR="${HOME}/.ros/log"; export DISPLAY="${DISPLAY:-:1}"
pkill -9 -f "[g]zserver" 2>/dev/null; pkill -9 -f "[r]osmaster" 2>/dev/null; sleep 2
mkdir -p "$WS/.verify"
export ROS_MASTER_URI=http://localhost:11590
export GAZEBO_MASTER_URI=http://localhost:11600
roslaunch --port=11590 competition_robot robot_gazebo.launch gui:=false rviz:=false > "$WS/.verify/cam.log" 2>&1 & LP=$!
sleep 38
kill -0 $LP 2>/dev/null || { echo "启动失败"; tail -20 "$WS/.verify/cam.log"; exit 1; }

echo "=== 深度相机话题 ==="
for t in /camera/rgb/image_raw /camera/rgb/camera_info /camera/depth/image_raw /camera/depth/points /camera/depth/camera_info; do
  rostopic list 2>/dev/null | grep -qx "$t" && echo "  [OK] $t" || echo "  [缺失] $t"
done
echo "=== 图像尺寸 / 编码 ==="
timeout 8 rostopic echo -n1 /camera/rgb/image_raw 2>/dev/null | grep -E "^  (width|height|encoding|step):" | head -4 | sed 's/^/  rgb  /'
timeout 8 rostopic echo -n1 /camera/depth/image_raw 2>/dev/null | grep -E "^  (width|height|encoding|step):" | head -4 | sed 's/^/  depth/'
echo "=== 深度点云点数 ==="
timeout 10 rostopic echo -n1 /camera/depth/points/width 2>/dev/null | sed 's/^/  点数 /'
echo
echo "=== 轮子到底转不转 (Gazebo 真实关节速度, 发 vx=0.30) ==="
timeout 8 rostopic pub -r 20 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.30, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null 2>&1 &
sleep 3
for j in fl fr rl rr; do
  timeout 6 rosservice call /gazebo/get_joint_properties "{joint_name: '${j}_wheel_joint'}" 2>/dev/null \
   | python3 -c "
import sys,re
t=sys.stdin.read()
p=re.search(r'position:\s*([-\d.eE]+)',t); v=re.search(r'velocity:\s*([-\d.eE]+)',t)
print('  %s_wheel_joint  position=%-12s velocity=%s' % ('$j', p.group(1)[:10] if p else '?', v.group(1)[:10] if v else '?'))"
done
echo "  期望: 前进 vx=0.30 -> 轮速 = 0.30/0.0485 = 6.19 rad/s, 四轮同向"
sleep 4
echo
echo "=== 自转时轮子 (发 wz=0.5), 期望左轮反转右轮正转 ==="
timeout 8 rostopic pub -r 20 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}" >/dev/null 2>&1 &
sleep 3
for j in fl fr rl rr; do
  timeout 6 rosservice call /gazebo/get_joint_properties "{joint_name: '${j}_wheel_joint'}" 2>/dev/null \
   | python3 -c "
import sys,re
t=sys.stdin.read()
v=re.search(r'velocity:\s*([-\d.eE]+)',t)
print('  %s velocity=%s' % ('$j', v.group(1)[:12] if v else '?'))"
done
sleep 4
kill -INT $LP 2>/dev/null; sleep 3; kill -9 $LP 2>/dev/null
pkill -9 -f "[g]zserver" 2>/dev/null; pkill -9 -f "[r]osmaster" 2>/dev/null
echo "=== done ==="
