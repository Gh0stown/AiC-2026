#!/usr/bin/env bash
# 最终验收: 全向运动 + 角速度保真度 + 里程计精度
WS="$(cd "$(dirname "$0")/.." && pwd)"
source /opt/ros/noetic/setup.bash; source "$WS/devel/setup.bash"
[ -n "${DSH_SIM_HOME:-}" ] && { export HOME="$DSH_SIM_HOME"; mkdir -p "$HOME/.ros";
  export LIBGL_ALWAYS_SOFTWARE=1; }   # 只在无 GPU 的沙箱里生效
export ROS_LOG_DIR="${HOME}/.ros/log"; export DISPLAY="${DISPLAY:-:1}"
pkill -9 -f "[g]zserver" 2>/dev/null; pkill -9 -f "[r]osmaster" 2>/dev/null; pkill -9 -f "[r]oslaunch" 2>/dev/null; sleep 2
mkdir -p "$WS/.verify"
export ROS_MASTER_URI=http://localhost:11510
export GAZEBO_MASTER_URI=http://localhost:11520
roslaunch --port=11510 competition_robot robot_gazebo.launch gui:=false rviz:=false > "$WS/.verify/final.log" 2>&1 & LP=$!
sleep 38
kill -0 $LP 2>/dev/null || { echo "roslaunch 挂了"; tail -20 "$WS/.verify/final.log"; exit 1; }

echo "=== 话题 ==="
for t in /cmd_vel /odom /odom_groundtruth /joint_states /imu /camera/rgb/image_raw /camera/depth/image_raw; do
  rostopic list 2>/dev/null | grep -qx "$t" && echo "  [OK] $t" || echo "  [缺失] $t"
done
# 雷达: 3D 发 /points, 2D 发 /scan, 装哪个算哪个
for t in /points /scan; do
  rostopic list 2>/dev/null | grep -qx "$t" && echo "  [OK] $t  (雷达)"
done

reset() { timeout 8 rosservice call /gazebo/set_model_state "{model_state: {model_name: 'competition_robot', pose: {position: {x: 0.0, y: 0.0, z: 0.02}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}, reference_frame: 'world'}}" >/dev/null 2>&1
         rosservice call /mecanum_odometry/reset >/dev/null 2>&1; sleep 2; }
gt() { timeout 6 rostopic echo -n1 /odom_groundtruth/twist/twist 2>/dev/null | python3 -c "
import sys,re
t=sys.stdin.read()
l=re.search(r'linear:\s*\n\s*x:\s*([-\d.eE]+)\s*\n\s*y:\s*([-\d.eE]+)',t)
a=re.search(r'angular:\s*\n\s*x:\s*[-\d.eE]+\s*\n\s*y:\s*[-\d.eE]+\s*\n\s*z:\s*([-\d.eE]+)',t)
print('  实际 vx=%+.4f vy=%+.4f wz=%+.4f'%(float(l.group(1)),float(l.group(2)),float(a.group(1)))) if l and a else print('  解析失败')"; }
truth() { timeout 6 rosservice call /gazebo/get_model_state "{model_name: 'competition_robot'}" 2>/dev/null | python3 -c "
import sys,re,math
t=sys.stdin.read()
p=re.search(r'position:\s*\n\s*x:\s*([-\d.eE]+)\s*\n\s*y:\s*([-\d.eE]+)',t)
o=re.search(r'orientation:\s*\n\s*x:\s*([-\d.eE]+)\s*\n\s*y:\s*([-\d.eE]+)\s*\n\s*z:\s*([-\d.eE]+)\s*\n\s*w:\s*([-\d.eE]+)',t)
if not p or not o: print('nan nan nan'); sys.exit()
qx,qy,qz,qw=[float(o.group(i)) for i in (1,2,3,4)]
print('%.4f %.4f %.4f'%(float(p.group(1)),float(p.group(2)),math.atan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz))))"; }
odomp() { timeout 6 rostopic echo -n1 /odom/pose/pose 2>/dev/null | python3 -c "
import sys,re,math
m=[float(v) for v in re.findall(r'[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?',sys.stdin.read())]
if len(m)<7: print('nan nan nan'); sys.exit()
print('%.4f %.4f %.4f'%(m[0],m[1],math.atan2(2*(m[6]*m[5]+m[3]*m[4]),1-2*(m[4]**2+m[5]**2))))"; }

echo "=== 速度保真度 (指令 vs 实际) ==="
for spec in "0.30 0.0 0.0" "0.0 0.30 0.0" "0.0 0.0 0.50" "0.20 0.20 0.40"; do
  set -- $spec; reset
  echo "--- 指令 vx=$1 vy=$2 wz=$3 ---"
  timeout 7 rostopic pub -r 20 /cmd_vel geometry_msgs/Twist "{linear: {x: $1, y: $2, z: 0.0}, angular: {x: 0.0, y: 0.0, z: $3}}" >/dev/null 2>&1 &
  sleep 3; gt; sleep 5
done

echo "=== 里程计精度 (短距, 停机后采样) ==="
for spec in "0.25 0.00 0.0 2 前进" "0.00 0.25 0.0 2 横移" "0.00 0.00 0.5 2 自转" "0.20 0.10 0.4 2 边走边转"; do
  set -- $spec; reset
  timeout $(($4+2)) rostopic pub -r 20 /cmd_vel geometry_msgs/Twist "{linear: {x: $1, y: $2, z: 0.0}, angular: {x: 0.0, y: 0.0, z: $3}}" >/dev/null 2>&1
  sleep 2.5
  read -r TX TY TA <<< "$(truth)"; read -r OX OY OA <<< "$(odomp)"
  python3 -c "
tx,ty,ta=$TX,$TY,$TA; ox,oy,oa=$OX,$OY,$OA
d=((ox-tx)**2+(oy-ty)**2)**0.5; r=(tx*tx+ty*ty)**0.5
print('  %-10s 真值(%+.4f,%+.4f,%+.4f)  里程(%+.4f,%+.4f,%+.4f)  位置差 %.4f m%s  角度差 %+.4f rad'
      % ('$5',tx,ty,ta,ox,oy,oa,d,(' (%.1f%%)'%(100*d/r)) if r>0.05 else '', oa-ta))"
done
kill -INT $LP 2>/dev/null; sleep 3; kill -9 $LP 2>/dev/null
pkill -f "[g]zserver" 2>/dev/null; pkill -f "[r]osmaster" 2>/dev/null
echo "=== 错误 ==="; grep -iE "\[ERROR\]|\[FATAL\]|died" "$WS/.verify/final.log" | grep -v rospack | head -6
echo "=== done ==="
