#!/usr/bin/env bash
WS="$(cd "$(dirname "$0")/.." && pwd)"
source /opt/ros/noetic/setup.bash; source "$WS/devel/setup.bash"
[ -n "${DSH_SIM_HOME:-}" ] && { export HOME="$DSH_SIM_HOME"; mkdir -p "$HOME/.ros"; export LIBGL_ALWAYS_SOFTWARE=1; }
export ROS_LOG_DIR="${HOME}/.ros/log"; export DISPLAY=:0
pkill -9 -f "[g]zserver"; pkill -9 -f "[r]osmaster"; sleep 2
mkdir -p "$WS/.verify"
export ROS_MASTER_URI=http://localhost:11910
export GAZEBO_MASTER_URI=http://localhost:11920
roslaunch --port=11910 competition_robot robot_gazebo.launch gui:=false rviz:=false > "$WS/.verify/m2_sim.log" 2>&1 & SIM=$!
sleep 40
rosrun gmapping slam_gmapping _base_frame:=base_footprint _odom_frame:=odom _map_frame:=map \
  _delta:=0.05 _maxUrange:=6.0 _particles:=40 _xmin:=-4.0 _xmax:=4.0 _ymin:=-4.0 _ymax:=4.0 \
  _linearUpdate:=0.05 _angularUpdate:=0.05 _lasamplerange:=0.005 _lasamplestep:=0.005 \
  > "$WS/.verify/m2_gm.log" 2>&1 & GM=$!
sleep 12
drive() {  # vx wz 秒
  timeout $(($3+1)) rostopic pub -r 20 /cmd_vel geometry_msgs/Twist \
    "{linear: {x: $1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: $2}}" >/dev/null 2>&1
}
echo "=== 走一个方形轨迹, 把场地扫一遍 ==="
drive 0.30 0.0 4
drive 0.0 0.8 4
drive 0.30 0.0 4
drive 0.0 0.8 4
drive 0.30 0.0 4
drive 0.0 0.8 4
drive 0.30 0.0 4
drive 0.0 0.8 4
sleep 3
echo "=== 存图 ==="
timeout 25 rosrun map_server map_saver -f "$WS/.verify/map2" > /dev/null 2>&1
cat "$WS/.verify/map2.yaml" 2>/dev/null | head -4 | sed 's/^/  /'
kill -INT $GM 2>/dev/null; kill -INT $SIM 2>/dev/null; sleep 3; kill -9 $GM $SIM 2>/dev/null
pkill -9 -f "[g]zserver"; pkill -9 -f "[r]osmaster"
echo "=== done ==="
