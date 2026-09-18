#!/usr/bin/env bash
WS="$(cd "$(dirname "$0")/.." && pwd)"
source /opt/ros/noetic/setup.bash; source "$WS/devel/setup.bash"
[ -n "${DSH_SIM_HOME:-}" ] && { export HOME="$DSH_SIM_HOME"; mkdir -p "$HOME/.ros"; export LIBGL_ALWAYS_SOFTWARE=1; }
export ROS_LOG_DIR="${HOME}/.ros/log"; export DISPLAY=:0
pkill -9 -f "[g]zserver"; pkill -9 -f "[r]osmaster"; sleep 2
mkdir -p "$WS/.verify"
export ROS_MASTER_URI=http://localhost:11830
export GAZEBO_MASTER_URI=http://localhost:11840
roslaunch --port=11830 competition_robot robot_gazebo.launch gui:=false rviz:=false > "$WS/.verify/v_sim.log" 2>&1 & SIM=$!
sleep 40
rosrun gmapping slam_gmapping _base_frame:=base_footprint _odom_frame:=odom _map_frame:=map \
  _delta:=0.05 _maxUrange:=6.0 _particles:=30 _xmin:=-5.0 _xmax:=5.0 _ymin:=-5.0 _ymax:=5.0 \
  _linearUpdate:=0.05 _angularUpdate:=0.05 > "$WS/.verify/v_gm.log" 2>&1 & GM=$!
sleep 12
echo "=== /scan 单帧看一眼 (是不是正常) ==="
timeout 10 rostopic echo -n1 /scan/ranges 2>/dev/null | python3 -c "
import sys, re, collections
v=[float(x) for x in re.findall(r'inf|[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?', sys.stdin.read())]
real=sorted(x for x in v if x!='inf' and x<100)
print('  bin 总数 %d, 有限值 %d' % (len(v), len(real)))
if real: print('  最小 %.2f 中位 %.2f 最大 %.2f' % (real[0], real[len(real)//2], real[-1]))
h=collections.Counter(min(int(x),6) for x in real)
for k in sorted(h): print('    %d~%dm: %s' % (k,k+1,'#'*min(h[k],50)))
"
echo "=== 原地转一圈 ==="
timeout 20 rostopic pub -r 20 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.6}}" >/dev/null 2>&1
sleep 3
echo "=== 存图 ==="
timeout 25 rosrun map_server map_saver -f "$WS/.verify/built_map" > "$WS/.verify/saver.log" 2>&1
ls -la "$WS/.verify/built_map.pgm" 2>/dev/null | awk '{print "  pgm: "$5" 字节"}'
echo "=== map -> odom 的 TF 在不在 ==="
timeout 8 rostopic echo /tf 2>/dev/null | grep -B1 -A1 '"map"' | head -8 | sed 's/^/  /'
kill -INT $GM 2>/dev/null; kill -INT $SIM 2>/dev/null; sleep 3; kill -9 $GM $SIM 2>/dev/null
pkill -9 -f "[g]zserver"; pkill -9 -f "[r]osmaster"
echo "=== done ==="
