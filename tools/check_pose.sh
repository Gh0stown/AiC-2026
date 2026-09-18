#!/usr/bin/env bash
WS="$(cd "$(dirname "$0")/.." && pwd)"
source /opt/ros/noetic/setup.bash; source "$WS/devel/setup.bash"
[ -n "${DSH_SIM_HOME:-}" ] && { export HOME="$DSH_SIM_HOME"; mkdir -p "$HOME/.ros"; export LIBGL_ALWAYS_SOFTWARE=1; }
export ROS_LOG_DIR="${HOME}/.ros/log"; export DISPLAY=:0
pkill -9 -f "[g]zserver"; pkill -9 -f "[r]osmaster"; sleep 2
mkdir -p "$WS/.verify"
export ROS_MASTER_URI=http://localhost:11870
export GAZEBO_MASTER_URI=http://localhost:11880
roslaunch --port=11870 competition_robot robot_gazebo.launch gui:=false rviz:=false > "$WS/.verify/pose.log" 2>&1 & SIM=$!
sleep 40
echo "=== 模型完整位姿 ==="
timeout 8 rosservice call /gazebo/get_model_state "{model_name: 'competition_robot'}" 2>/dev/null | sed -n '/^pose:/,/^twist:/p' | head -12 | sed 's/^/  /'
echo "=== 换算成 yaw ==="
timeout 8 rosservice call /gazebo/get_model_state "{model_name: 'competition_robot'}" 2>/dev/null | python3 -c "
import sys,re,math
t=sys.stdin.read()
o=re.search(r'orientation:\s*\n\s*x:\s*([-\d.eE]+)\s*\n\s*y:\s*([-\d.eE]+)\s*\n\s*z:\s*([-\d.eE]+)\s*\n\s*w:\s*([-\d.eE]+)',t)
if o:
    qx,qy,qz,qw=[float(o.group(i)) for i in (1,2,3,4)]
    yaw=math.atan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz))
    print('  yaw = %.4f rad = %.1f deg' % (yaw, math.degrees(yaw)))
"
echo "=== TF: odom -> laser_link (看雷达在世界里的朝向) ==="
timeout 10 rosrun tf tf_echo odom laser_link 2>/dev/null | head -10 | sed 's/^/  /'
echo "=== /scan 几个关键方向 ==="
timeout 12 rostopic echo -n1 /scan/ranges 2>/dev/null | python3 -c "
import sys,re,math
v=[float(x) for x in re.findall(r'inf|[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?', sys.stdin.read())]
n=len(v)
for d in (0,90,180,270):
    i=int(round(math.radians(d)/math.radians(1.0)))%n
    print('  scan %3d deg -> %.3f m' % (d, v[i]))
"
kill -INT $SIM 2>/dev/null; sleep 2; kill -9 $SIM 2>/dev/null
pkill -9 -f "[g]zserver"; pkill -9 -f "[r]osmaster"
