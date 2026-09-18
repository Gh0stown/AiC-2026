#!/usr/bin/env bash
# 对比三种速度施加方式的自转保真度, 找出能修好 0.637 的那个。
WS="$(cd "$(dirname "$0")/.." && pwd)"
URDF="$WS/src/competition_robot/urdf/competition_robot.urdf"
source /opt/ros/noetic/setup.bash; source "$WS/devel/setup.bash"
[ -n "${DSH_SIM_HOME:-}" ] && { export HOME="$DSH_SIM_HOME"; mkdir -p "$HOME/.ros";
  export LIBGL_ALWAYS_SOFTWARE=1; }   # 只在无 GPU 的沙箱里生效
export ROS_LOG_DIR="${HOME}/.ros/log"; export DISPLAY="${DISPLAY:-:1}"
mkdir -p "$WS/.verify"

for MODE in canonical all_rigid model planar_move; do
  pkill -9 -f "[g]zserver" 2>/dev/null; pkill -9 -f "[r]osmaster" 2>/dev/null; sleep 2
  if [ "$MODE" = "planar_move" ]; then
    python3 - "$URDF" <<'PY'
import sys,re
p=sys.argv[1]; s=open(p).read()
s=re.sub(r'<velocityMode>\w+</velocityMode>','<velocityMode>canonical</velocityMode>',s)
s=s.replace('libcompetition_holonomic_drive.so','libgazebo_ros_planar_move.so')
s=re.sub(r'\s*<publishOdometry>true</publishOdometry>','',s)
open(p,'w').write(s)
PY
  else
    python3 - "$URDF" "$MODE" <<'PY'
import sys,re
p,mode=sys.argv[1],sys.argv[2]; s=open(p).read()
s=s.replace('libgazebo_ros_planar_move.so','libcompetition_holonomic_drive.so')
if '<velocityMode>' in s:
    s=re.sub(r'<velocityMode>\w+</velocityMode>','<velocityMode>%s</velocityMode>'%mode,s)
else:
    s=s.replace('</plugin>','      <velocityMode>%s</velocityMode>\n    </plugin>'%mode,1)
if '<publishOdometry>' not in s:
    s=s.replace('</plugin>','      <publishOdometry>true</publishOdometry>\n    </plugin>',1)
open(p,'w').write(s)
PY
  fi

  export ROS_MASTER_URI=http://localhost:11630
export GAZEBO_MASTER_URI=http://localhost:11640
roslaunch --port=11630 competition_robot robot_gazebo.launch gui:=false rviz:=false > "$WS/.verify/dm_$MODE.log" 2>&1 & LP=$!
  sleep 36
  if ! kill -0 $LP 2>/dev/null; then echo "  $MODE: 启动失败"; continue; fi

  reset() { timeout 8 rosservice call /gazebo/set_model_state "{model_state: {model_name: 'competition_robot', pose: {position: {x: 0.0, y: 0.0, z: 0.02}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}, reference_frame: 'world'}}" >/dev/null 2>&1; sleep 2; }
  gt() { timeout 6 rostopic echo -n1 /odom_groundtruth/twist/twist 2>/dev/null | python3 -c "
import sys,re
t=sys.stdin.read()
l=re.search(r'linear:\s*\n\s*x:\s*([-\d.eE]+)\s*\n\s*y:\s*([-\d.eE]+)',t)
a=re.search(r'angular:\s*\n\s*x:\s*[-\d.eE]+\s*\n\s*y:\s*[-\d.eE]+\s*\n\s*z:\s*([-\d.eE]+)',t)
print('%.4f %.4f %.4f'%(float(l.group(1)),float(l.group(2)),float(a.group(1)))) if l and a else print('nan nan nan')"; }

  R=""
  for spec in "0.30 0.0 0.0 vx" "0.0 0.30 0.0 vy" "0.0 0.0 0.50 wz"; do
    set -- $spec; reset
    timeout 7 rostopic pub -r 20 /cmd_vel geometry_msgs/Twist "{linear: {x: $1, y: $2, z: 0.0}, angular: {x: 0.0, y: 0.0, z: $3}}" >/dev/null 2>&1 &
    sleep 3
    read -r ax ay az <<< "$(gt)"
    R="$R $4=$(python3 -c "print('%.3f'%(($az if '$4'=='wz' else ($ay if '$4'=='vy' else $ax))/max(abs($1+$2+$3),1e-9)))")"
    sleep 4
  done
  echo "  mode=$MODE  ->$R"
  kill -INT $LP 2>/dev/null; sleep 2; kill -9 $LP 2>/dev/null
done
pkill -9 -f "[g]zserver"; pkill -9 -f "[r]osmaster"
python3 - "$URDF" <<'PY'
import sys,re
p=sys.argv[1]; s=open(p).read()
s=s.replace('libgazebo_ros_planar_move.so','libcompetition_holonomic_drive.so')
if '<velocityMode>' not in s:
    s=s.replace('</plugin>','      <velocityMode>canonical</velocityMode>\n    </plugin>',1)
if '<publishOdometry>' not in s:
    s=s.replace('</plugin>','      <publishOdometry>true</publishOdometry>\n    </plugin>',1)
open(p,'w').write(s)
PY
echo "=== done (URDF 已恢复成 holonomic 插件) ==="
