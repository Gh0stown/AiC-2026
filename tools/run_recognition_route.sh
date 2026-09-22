#!/usr/bin/env bash
# =============================================================================
#  一键跑"最终巡检路线" (航点 + 沿途识别点) 并出轨迹图
#
#  做三件事:
#    1) 起仿真 + 定位/导航 (navigation.launch)
#    2) 用 tools/patrol.py 跑 config/recognition_route.yaml
#       —— 每个识别点: 先 cmd_vel 原地转向 -> move_base 直线过去 -> 到点再原地
#          转到拍照朝向 (patrol.py 的原生流程, 不需要额外节点)
#    3) 存轨迹, 并把轨迹叠加到 docs/recognition_route.png 上
#
#  用法:
#      tools/run_recognition_route.sh              # 无头全自动
#      tools/run_recognition_route.sh --gui        # 带 Gazebo/RViz 界面
#      tools/run_recognition_route.sh --keep       # 跑完不关仿真
#      tools/run_recognition_route.sh --no-plot    # 不重画轨迹图
#      tools/run_recognition_route.sh --route <yaml>
#      tools/run_recognition_route.sh --capture           # 每个识别点拍照, 存 captures/<时间戳>/
#      tools/run_recognition_route.sh --capture --frames 5
#      tools/run_recognition_route.sh --no-park            # 不做倒车入库
# =============================================================================
set -u

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
GUI=false; KEEP=false; PLOT=true; CAPTURE=false; FRAMES=3; PARK=true
ROUTE="$ROOT/src/competition_robot/config/recognition_route.yaml"
while [ $# -gt 0 ]; do
  case "$1" in
    --gui)     GUI=true ;;
    --keep)    KEEP=true ;;
    --no-plot) PLOT=false ;;
    --capture) CAPTURE=true ;;
    --no-park) PARK=false ;;
    --frames)  FRAMES="$2"; shift ;;
    --route)   ROUTE="$2"; shift ;;
    *) echo "未知参数: $1"; exit 2 ;;
  esac
  shift
done

# --- 和 acceptance.sh 同一套环境约定 (固定端口, HOME 只读时退到工作区) ---
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11500}"
export ROS_HOME="${ROS_HOME:-$ROOT/.cache/ros}"
export GAZEBO_MODEL_DATABASE_URI=""
# 无 GPU 的沙箱/CI 里 gzserver 会因为 GL 初始化偶发段错误, 强制软件渲染能少踩
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
export SVGA_VGPU10=0
if touch "$HOME/.gazebo_wtest" 2>/dev/null; then
  rm -f "$HOME/.gazebo_wtest"
else
  export HOME="$ROOT/.cache/home"
fi
mkdir -p "$ROS_HOME" "$HOME"

set +u
source /opt/ros/noetic/setup.bash
source "$ROOT/devel/setup.bash"
set -u

LOGD="$ROOT/.cache/route"; mkdir -p "$LOGD"
TRACE="$LOGD/recognition_trace.csv"
SIM_PID=""
cleanup() {
  if [ -n "$SIM_PID" ] && ! $KEEP; then
    echo; echo "关闭仿真..."
    kill -INT "$SIM_PID" 2>/dev/null; sleep 3; kill -9 "$SIM_PID" 2>/dev/null
    pkill -9 -f "[g]zserver" 2>/dev/null
    pkill -9 -f "[r]osmaster" 2>/dev/null
  fi
}
trap cleanup EXIT

hdr() { echo; echo "── $1 ─────────────────────────────────────────"; }

# 预清理: 上一轮崩溃可能留下 gzserver/rosmaster/node, 会干扰新一次启动
pkill -9 -f "[g]zserver" 2>/dev/null
pkill -9 -f "[r]osmaster" 2>/dev/null
pkill -9 -f "[c]ompetition_robot/scripts" 2>/dev/null
pkill -9 -f "[c]ompetition_arena/scripts" 2>/dev/null
sleep 2

hdr "启动仿真 + 导航"
GUI_ARG="gui:=false"; RV_ARG="rviz:=false"
$GUI && { GUI_ARG="gui:=true"; RV_ARG="rviz:=true"; }

# gzserver 在无 GPU 环境会偶发段错误 (交接文档 §5 记过) -> 启动重试
SIM_PID=""
for attempt in 1 2 3; do
  echo "  第 $attempt 次启动..."
  roslaunch competition_robot navigation.launch $GUI_ARG $RV_ARG \
      > "$LOGD/sim.log" 2>&1 &
  SIM_PID=$!
  ok=false
  for i in $(seq 1 90); do
    if rosnode list 2>/dev/null | grep -q "^/move_base$"; then ok=true; break; fi
    if ! kill -0 "$SIM_PID" 2>/dev/null; then break; fi
    sleep 2
  done
  # Gazebo 服务真就绪才算成功 (光有 move_base 会跑太早)
  if $ok; then
    for i in $(seq 1 25); do
      if timeout 8 python3 -c "
import rospy
from gazebo_msgs.srv import GetWorldProperties
rospy.init_node('route_probe', anonymous=True, disable_signals=True)
try:
    s = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
    s.wait_for_service(timeout=8)
    raise SystemExit(0 if len(s().model_names) > 0 else 1)
except Exception:
    raise SystemExit(1)
" 2>/dev/null; then
        echo "  Gazebo 就绪 (第 $attempt 次)"
        break 2
      fi
      kill -0 "$SIM_PID" 2>/dev/null || break
      sleep 2
    done
  fi
  echo "  第 $attempt 次没起来 (gzserver 可能段错误了), 清理重试"
  kill -9 "$SIM_PID" 2>/dev/null
  pkill -9 -f "[g]zserver" 2>/dev/null
  pkill -9 -f "[r]osmaster" 2>/dev/null
  sleep 5
  SIM_PID=""
done
if [ -z "$SIM_PID" ]; then
  echo "!! 三次都没起来, 看 $LOGD/sim.log"; exit 1
fi
echo "  等 AMCL 收敛 (12s)..."
sleep 12

if $CAPTURE; then
  hdr "跑路线 + 每个识别点拍照"
  echo "  输出: captures/<时间戳>/   (每个识别点 $FRAMES 帧)"
  python3 "$ROOT/tools/capture_points.py" --route "$ROUTE" --frames "$FRAMES" \
          2>&1 | tee "$LOGD/capture.log"
  CAPDIR=$(ls -td "$ROOT"/captures/*/ 2>/dev/null | head -1)
  hdr "结果"
  if [ -n "$CAPDIR" ]; then
    echo "  本次图片: ${CAPDIR#$ROOT/}"
    echo "  索引    : ${CAPDIR#$ROOT/}index.csv"
    ls "$CAPDIR" | head -20 | sed 's/^/    /'
    if [ -f "$CAPDIR/index.csv" ]; then
      echo "  ── index.csv ──"
      column -s, -t < "$CAPDIR/index.csv" 2>/dev/null | cut -c1-150 | sed 's/^/    /' || cat "$CAPDIR/index.csv"
    fi
    if $PLOT; then
      python3 "$ROOT/tools/show_captures.py" "$CAPDIR" >/dev/null 2>&1 \
        && echo "  总览图  : ${CAPDIR#$ROOT/}overview.png"
    fi
    echo; echo "  拍照完成 ✅"
    exit 0
  fi
  echo; echo "  没拍到图片 ❌  详见 $LOGD/capture.log"
  exit 1
fi

hdr "跑最终路线 ($(basename "$ROUTE"))"
echo "  每个识别点: 原地转向 -> 直线过去 -> 到点转到拍照朝向"
# ★ issue #6: 不再硬编码 --no-park —— 路线 yaml 里带 reverse_park 时, 航点跑完
#   会自动接倒车入库 (patrol.py 的原生流程)。想跳过加 --no-park。
PARK_ARG=""
$PARK || PARK_ARG="--no-park"
python3 "$ROOT/tools/patrol.py" --file "$ROUTE" $PARK_ARG \
        --save-trace "$TRACE" 2>&1 | tee "$LOGD/patrol.log" | grep -E "✓|✗|一圈|摆正|到点|入位" || true

hdr "结果"
OK=$(grep -c "✓" "$LOGD/patrol.log" 2>/dev/null | head -1); OK=${OK:-0}
BAD=$(grep -c "✗" "$LOGD/patrol.log" 2>/dev/null | head -1); BAD=${BAD:-0}
LAST=$(grep -o "一圈跑完: [0-9]*/[0-9]* 个航点成功" "$LOGD/patrol.log" 2>/dev/null | tail -1)
echo "  到点成功 $OK 个, 失败/超时 $BAD 个   ${LAST:+($LAST)}"
if [ "$BAD" != "0" ]; then
  echo "  ✗ 有失败的点:"; grep "✗" "$LOGD/patrol.log" | tail -10
fi
if [ -f "$TRACE" ]; then
  echo "  轨迹 -> $TRACE  ($(wc -l < "$TRACE") 行)"
fi

if $PLOT && [ -f "$TRACE" ]; then
  hdr "重画轨迹图"
  python3 "$ROOT/tools/gen_recognition_route.py" --trace "$TRACE" || true
fi

if [ "$BAD" = "0" ] && [ "$OK" != "0" ]; then
  echo; echo "  全部到点 ✅"
  exit 0
fi
echo; echo "  有失败/超时 ❌  详见 $LOGD/patrol.log"
exit 1
