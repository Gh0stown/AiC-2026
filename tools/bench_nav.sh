#!/usr/bin/env bash
# 对比局部规划器: 同样的"一条直线走 3 m"任务, 看谁走得直、转弯丝滑。
#
#   bash tools/bench_nav.sh traj          # 本机自带的老规划器 (基准)
#   bash tools/bench_nav.sh teb           # 需要 sudo apt install ros-noetic-teb-local-planner
#   bash tools/bench_nav.sh dwa           # 需要 sudo apt install ros-noetic-dwa-local-planner
#   bash tools/bench_nav.sh teb -1.5 -1.5 0    # 也可以自己指定目标点
#
# 默认测法: 车出生在 (1.5, 0) 朝 -x, 目标 (-1.5, 0) 朝向不变 —— 一条正前方的
#       3 m 直线, 中间没有障碍。任何横向晃动都是规划器自己画出来的。
# 带转弯的测法: bash tools/bench_nav.sh teb -1.5 -1.5 0   (要走 45° 斜线 + 端头对朝向)
# 指标含义见 tools/analyze_nav_run.py
set -u
PLANNER="${1:-traj}"
GX="${2:--1.5}"
GY="${3:-0.0}"
GYAW="${4:-3.1416}"
GPLAN="${5:-navfn}"
# 出生点也可以覆盖: SX=1.5 SY=0 SYAW=3.1416 bash tools/bench_nav.sh ...
SX="${SX:-1.5}"; SY="${SY:-0.0}"; SYAW="${SYAW:-3.1416}"
# 换地图: MAP=/abs/xxx.yaml bash tools/bench_nav.sh ...
MAPARG=""
[ -n "${MAP:-}" ] && MAPARG="map:=$MAP"
WS="$(cd "$(dirname "$0")/.." && pwd)"
export HOME="${DSH_SIM_HOME:-$HOME}"; mkdir -p "$HOME/.ros"
[ -n "${DSH_SIM_HOME:-}" ] && export LIBGL_ALWAYS_SOFTWARE=1
source /opt/ros/noetic/setup.bash
[ -f "$WS/devel/setup.bash" ] && source "$WS/devel/setup.bash"
export ROS_LOG_DIR="$HOME/.ros/log"
mkdir -p "$WS/.verify"
export ROS_MASTER_URI=http://localhost:13550 GAZEBO_MASTER_URI=http://localhost:13560

pkill -9 -f "[g]zserver"; pkill -9 -f "[r]osmaster"; pkill -9 -f "[r]ec_navbench"; sleep 2
roslaunch --port=13550 competition_robot navigation.launch \
    planner:="$PLANNER" gplan:="$GPLAN" $MAPARG gui:=false rviz:=false \
    x:="$SX" y:="$SY" yaw:="$SYAW" \
    > "$WS/.verify/bench_nav_${PLANNER}.log" 2>&1 & LP=$!
sleep 45
if ! kill -0 $LP 2>/dev/null; then
  echo "启动失败 (装了吗? sudo apt install ros-noetic-teb-local-planner)"; tail -15 "$WS/.verify/bench_nav_${PLANNER}.log"; exit 1
fi

cat > "$WS/.verify/rec_navbench.py" <<'PY'
import atexit, math, pickle, sys, rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseWithCovarianceStamped
D={'gt': [], 'amcl': [], 'goal': (float(sys.argv[2]), float(sys.argv[3])), 'plan': [], 'plan_topic': ''}
def y(q): return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
def cb(m):
    p=m.pose.pose; D['gt'].append((m.header.stamp.to_sec(), p.position.x, p.position.y, y(p.orientation)))
def cb_amcl(m):
    p=m.pose.pose; D['amcl'].append((m.header.stamp.to_sec(), p.position.x, p.position.y, y(p.orientation)))
def cb_plan(m):
    if len(m.poses) > 5 and D['plan'] == []:
        D['plan'] = [(q.pose.position.x, q.pose.position.y) for q in m.poses]
        D['plan_topic'] = m.header.frame_id
atexit.register(lambda: pickle.dump(D, open(sys.argv[1], 'wb')))
rospy.init_node('rec_navbench', anonymous=True)
rospy.Subscriber('/odom_groundtruth', Odometry, cb, queue_size=5000)
rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, cb_amcl, queue_size=5000)
for t in ('/move_base/NavfnROS/plan', '/move_base/GlobalPlanner/plan'):
    rospy.Subscriber(t, Path, cb_plan, queue_size=2)
rospy.spin()
PY
python3 "$WS/.verify/rec_navbench.py" "$WS/.verify/bench_nav_${PLANNER}.pkl" "$GX" "$GY" >"$WS/.verify/rec_navbench_${PLANNER}.log" 2>&1 & RP=$!
sleep 2
echo "=== planner=$PLANNER / global=$GPLAN / map=${MAP:-默认} : 目标 ($GX, $GY, yaw=$GYAW) ==="
timeout 90 python3 "$WS/tools/go_to.py" "$GX" "$GY" "$GYAW" --timeout 70 2>&1 | grep -E "✓|✗|超时"
sleep 2
kill -INT $RP 2>/dev/null; sleep 3; kill -9 $RP 2>/dev/null
python3 "$WS/tools/analyze_nav_run.py" "$WS/.verify/bench_nav_${PLANNER}.pkl"
grep -c "Rotate recovery" "$WS/.verify/bench_nav_${PLANNER}.log" | sed 's/^/  rotate recovery 次数: /'
kill -INT $LP 2>/dev/null; sleep 3; kill -9 $LP 2>/dev/null
pkill -9 -f "[g]zserver"; pkill -9 -f "[r]osmaster"
echo "=== done ==="
