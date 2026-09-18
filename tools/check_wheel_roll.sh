#!/usr/bin/env bash
WS="$(cd "$(dirname "$0")/.." && pwd)"
source /opt/ros/noetic/setup.bash; source "$WS/devel/setup.bash"
[ -n "${DSH_SIM_HOME:-}" ] && { export HOME="$DSH_SIM_HOME"; mkdir -p "$HOME/.ros";
  export LIBGL_ALWAYS_SOFTWARE=1; }   # 只在无 GPU 的沙箱里生效
export ROS_LOG_DIR="${HOME}/.ros/log"; export DISPLAY="${DISPLAY:-:1}"
pkill -9 -f "[g]zserver" 2>/dev/null; pkill -9 -f "[r]osmaster" 2>/dev/null; sleep 2
mkdir -p "$WS/.verify"
export ROS_MASTER_URI=http://localhost:11570
export GAZEBO_MASTER_URI=http://localhost:11580
roslaunch --port=11570 competition_robot robot_gazebo.launch gui:=false rviz:=false > "$WS/.verify/wr.log" 2>&1 & LP=$!
sleep 38
wheels() {
  for j in fl fr rl rr; do
    timeout 6 rosservice call /gazebo/get_joint_properties "{joint_name: '${j}_wheel_joint'}" 2>/dev/null \
     | python3 -c "
import sys,re
t=sys.stdin.read()
p=re.search(r'position:\s*\[([-\d.eE]+)\]',t)
r=re.search(r'rate:\s*\[([-\d.eE]+)\]',t)
print('  %-3s pos=%-12.4f rate=%+.4f rad/s' % ('$j', float(p.group(1)), float(r.group(1)))) if p and r else print('  $j 解析失败')"
  done
}
echo "=== 静止时 ==="; wheels
echo "=== 发 vx=0.30 (期望 rate = 0.30/0.0485 = 6.19, 四轮同号) ==="
timeout 8 rostopic pub -r 20 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.30, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null 2>&1 &
sleep 3; wheels; sleep 4
echo "=== 发 wz=0.5 (期望前轮 ∓5.62 / 后轮 ∓4.64, 左右反号) ==="
timeout 8 rostopic pub -r 20 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}" >/dev/null 2>&1 &
sleep 3; wheels; sleep 4
kill -INT $LP 2>/dev/null; sleep 3; kill -9 $LP 2>/dev/null
pkill -9 -f "[g]zserver" 2>/dev/null; pkill -9 -f "[r]osmaster" 2>/dev/null
echo "=== done ==="
