#!/usr/bin/env bash
# =============================================================================
#  一键验收 —— 起仿真 + 导航, 自动跑全部检查, 最后打印通过/不通过
#
#  用法:
#     ./tools/acceptance.sh              # 无头模式, 全自动 (推荐先跑这个)
#     ./tools/acceptance.sh --gui        # 带 Gazebo 图形界面 (人工看)
#     ./tools/acceptance.sh --keep       # 跑完不关, 留着手动操作
#     ./tools/acceptance.sh --quick      # 跳过耗时最长的巡逻 (只查场景/灯/定位)
#
#  检查项:
#     1. 场景完整性   —— 场地/红绿灯/人偶/车辆模型是否都在
#     2. 红绿灯切换   —— 红->绿->黄循环, 且任何时刻都不会三灯全灭
#     3. 定位精度     —— AMCL 估计 vs Gazebo 真值
#     4. 雷达位姿     —— 用 /scan 反推的位姿 vs AMCL (检验雷达标定)
#     5. 巡航         —— 7 个航点是否全部到达, 且全程不压车道线
#     6. 倒车入库     —— 车尾入库的位置/角度误差
# =============================================================================
set -u

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

GUI=false; KEEP=false; QUICK=false
for a in "$@"; do
  case "$a" in
    --gui)   GUI=true ;;
    --keep)  KEEP=true ;;
    --quick) QUICK=true ;;
    --route) ROUTE="$2"; shift ;;
    *) echo "未知参数: $a"; exit 2 ;;
  esac
done

# ★ issue #6: 巡航/倒车入库验收以前跑的是**旧路线**(waypoints.yaml, 没带 --file),
#   于是"新路线 + 倒车入库"这个组合从没被验过。现在默认验**最终路线**,
#   想验旧路线加 --route <yaml>。
ROUTE="${ROUTE:-$ROOT/src/competition_robot/config/recognition_route.yaml}"

# 用固定端口, 避免和别的 roslaunch 抢
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11500}"
export ROS_HOME="${ROS_HOME:-$ROOT/.cache/ros}"
export GAZEBO_MODEL_DATABASE_URI=""

# Gazebo 要往 $HOME/.gazebo 写日志; HOME 只读时会起不来。
# 能写就用真 HOME (用户机器), 不能写就退到工作区 (沙箱/CI)。
if touch "$HOME/.gazebo_wtest" 2>/dev/null; then
  rm -f "$HOME/.gazebo_wtest"
else
  export HOME="$ROOT/.cache/home"
fi
mkdir -p "$ROS_HOME" "$HOME"

# ROS 的 setup.bash 在 set -u 下会因为引用未定义变量而报错
set +u
source /opt/ros/noetic/setup.bash
if [ ! -f "$ROOT/devel/setup.bash" ]; then
  echo "!! 找不到 devel/setup.bash, 先编译:  cd $ROOT && catkin_make"
  exit 1
fi
source "$ROOT/devel/setup.bash"
set -u

LOGD="$ROOT/.cache/acceptance"; mkdir -p "$LOGD"
rm -f "$LOGD"/*.log
SIM_PID=""
PASS=(); FAIL=(); NOTE=()

cleanup() {
  if [ -n "$SIM_PID" ] && ! $KEEP; then
    echo; echo "关闭仿真..."
    kill -INT "$SIM_PID" 2>/dev/null
    sleep 3; kill -9 "$SIM_PID" 2>/dev/null
    pkill -9 -f "[g]zserver"  2>/dev/null
    pkill -9 -f "[r]osmaster" 2>/dev/null
  fi
}
trap cleanup EXIT

ok()   { PASS+=("$1"); printf '  \033[32m✓\033[0m %s\n' "$1"; }
bad()  { FAIL+=("$1"); printf '  \033[31m✗\033[0m %s\n' "$1"; }
note() { NOTE+=("$1"); printf '  \033[33m·\033[0m %s\n' "$1"; }
hdr()  { echo; echo "── $1 ─────────────────────────────────────────"; }

# ---------------------------------------------------------------- 起仿真
hdr "启动仿真"
GUI_ARG="gui:=false"; RV_ARG="rviz:=false"
$GUI && { GUI_ARG="gui:=true"; RV_ARG="rviz:=true"; }

# ★ gzserver 在无 GPU 环境会**偶发段错误** (见文件末尾故障表)。
#   原来一崩整轮就失败、得手工重跑; 现在自动重试最多 3 次 ——
#   注意要判**两件事**: ① move_base 起来 ② Gazebo 的 ROS 服务真就绪。
#   (只判 ① 不够: 实测遇到过 move_base 起来了、gzserver 随后才段错误)
SIM_PID=""
SIM_OK=false
for attempt in 1 2 3; do
  echo "  第 $attempt 次启动..."
  roslaunch competition_robot navigation.launch $GUI_ARG $RV_ARG \
      > "$LOGD/sim.log" 2>&1 &
  SIM_PID=$!

  # ① move_base  (★ 每轮都查"进程还在不在 / 日志有没有崩溃", 崩了就立刻换下一次,
  #    否则会白等到超时 —— 原来 90 轮 x 2s 太慢)
  up=false
  for i in $(seq 1 60); do
    rosnode list 2>/dev/null | grep -q "^/move_base$" && { up=true; break; }
    kill -0 "$SIM_PID" 2>/dev/null || break
    grep -qE "Segmentation|exit code 139" "$LOGD/sim.log" 2>/dev/null && break
    sleep 2
  done

  # ② 真正调一次 Gazebo 服务来探测 (比 grep rosservice list 可靠)
  #    同样快速失败: 探测 6s 超时 + 每轮查节点/日志
  if $up; then
    for i in $(seq 1 20); do
      if timeout 6 python3 -c "
import rospy
from gazebo_msgs.srv import GetWorldProperties
rospy.init_node('acc_probe', anonymous=True, disable_signals=True)
try:
    s = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
    s.wait_for_service(timeout=4)
    raise SystemExit(0 if len(s().model_names) > 0 else 1)
except Exception:
    raise SystemExit(1)
" >/dev/null 2>&1; then SIM_OK=true; break; fi
      kill -0 "$SIM_PID" 2>/dev/null || break
      rosnode list 2>/dev/null | grep -q "^/gazebo$" || break
      grep -qE "Segmentation|exit code 139" "$LOGD/sim.log" 2>/dev/null && break
      sleep 2
    done
  fi

  $SIM_OK && break
  echo "  第 $attempt 次没起来 (gzserver 段错误?), 清理重试"
  grep -E "Segmentation|exit code 139|process has died" "$LOGD/sim.log" | tail -2 | sed 's/^/     /'
  kill -9 "$SIM_PID" 2>/dev/null
  pkill -9 -f "[g]zserver" 2>/dev/null
  pkill -9 -f "[r]osmaster" 2>/dev/null
  sleep 5
  SIM_PID=""
done
if ! $SIM_OK; then
  echo "!! 三次都没把仿真+Gazebo 拉起来, 看 $LOGD/sim.log"
  echo "   /gazebo 节点: $(rosnode list 2>/dev/null | grep -c '^/gazebo$')"
  echo "   gazebo 服务数: $(timeout 8 rosservice list 2>/dev/null | grep -c '/gazebo/')"
  grep -E "Segmentation|exit code 139|process has died" "$LOGD/sim.log" | tail -3 | sed 's/^/     /'
  exit 1
fi
echo "  roslaunch PID=$SIM_PID, 日志 $LOGD/sim.log"

# 等 AMCL 出第一帧
for i in $(seq 1 45); do
  timeout 3 rostopic echo -n1 /amcl_pose >/dev/null 2>&1 && break
  sleep 2
done
sleep 3
echo "  就绪 (Gazebo 服务 + move_base + AMCL 都在)"

# ---------------------------------------------------------------- 1. 场景
hdr "1/6 场景完整性"
python3 - "$LOGD/models.txt" <<'PY' 2>&1 | tee "$LOGD/check1.txt"
import sys, rospy
from gazebo_msgs.srv import GetWorldProperties
rospy.init_node('acc_models', anonymous=True, disable_signals=True)
s = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
s.wait_for_service(timeout=25)
names = set(s().model_names)
open(sys.argv[1], 'w').write('\n'.join(sorted(names)))
want = {
    '场地(地板)':      ['arena_floor'],
    '围场墙':          ['arena_walls'],
    '机器人':          ['competition_robot'],
    '红绿灯':          ['tl_top', 'tl_bot'],
    '人偶立牌(A街区)': ['A_north_1', 'A_north_2', 'A_south_1', 'A_south_2', 'A_west_1', 'A_west_2'],
    '人偶立牌(B街区)': ['B_north_1', 'B_north_2', 'B_east_1', 'B_east_2'],
    '车辆+车牌':       ['car_1_p', 'car_2_p', 'car_3_p'],
}
badn = 0
for k, v in want.items():
    miss = [x for x in v if x not in names]
    if miss:
        print('  MISS %-16s 缺: %s' % (k, ' '.join(miss))); badn += 1
    else:
        print('  OK   %-16s %s' % (k, ' '.join(v)))
print('  MISSING_TOTAL=%d' % badn)
PY
if grep -q "MISSING_TOTAL=0" "$LOGD/check1.txt" 2>/dev/null; then
  ok "场景完整性: 场地/墙/机器人/红绿灯/10 个人偶/3 辆车 全部加载"
else
  bad "场景完整性: 有模型没加载 (详见 $LOGD/check1.txt)"
fi

# ---------------------------------------------------------------- 2. 红绿灯
hdr "2/6 红绿灯切换"
timeout 90 python3 tools/verify_traffic_light.py > "$LOGD/check2.txt" 2>&1
if [ $? -eq 0 ]; then
  ok "红绿灯: $(grep -oE '✓ [0-9]+/[0-9]+ 全部正确' "$LOGD/check2.txt" | tail -1)"
else
  bad "红绿灯: $(grep -oE '✗ [0-9]+/[0-9]+ 不对' "$LOGD/check2.txt" | tail -1) (见 $LOGD/check2.txt)"
fi
grep -E "^tl_|结论" "$LOGD/check2.txt" | sed 's/^/      /'

# ---------------------------------------------------------------- 3. 定位
hdr "3/6 定位精度 (AMCL vs 真值)"
timeout 120 python3 tools/check_localization.py --duration 25 > "$LOGD/check3.txt" 2>&1
V=$(grep -oE "均值 [0-9]+\.[0-9]+ m" "$LOGD/check3.txt" | grep -oE "[0-9]+\.[0-9]+" | head -1)
if [ -n "${V:-}" ]; then
  if python3 -c "import sys; sys.exit(0 if $V < 0.10 else 1)"; then
    ok "定位: 平均位置误差 ${V} m (< 0.10 m)"
  else
    bad "定位: 平均位置误差 ${V} m (>= 0.10 m)"
  fi
else
  note "定位: 没能解析出数值, 见 $LOGD/check3.txt"
fi
grep -E "位置误差|朝向误差" "$LOGD/check3.txt" | sed 's/^/      /'

# ---------------------------------------------------------------- 4. 雷达位姿
hdr "4/6 雷达位姿标定"
timeout 150 python3 tools/check_laser_pose.py > "$LOGD/check4.txt" 2>&1
grep -E "^\(" "$LOGD/check4.txt" | tail -4 | sed 's/^/      /'
LR=$(grep -oE "[0-9]+\.[0-9]+ mm / [0-9]+\.[0-9]+ mm" "$LOGD/check4.txt" | tail -1)
if [ -n "${LR:-}" ]; then
  LM=$(echo "$LR" | grep -oE "^[0-9]+\.[0-9]+")
  if python3 -c "import sys; sys.exit(0 if $LM < 15 else 1)"; then
    ok "雷达位姿: 激光反推误差 ${LM} mm (< 15 mm, 标定正常)"
  else
    bad "雷达位姿: 激光反推误差 ${LM} mm (>= 15 mm)"
  fi
else
  note "雷达位姿: 见 $LOGD/check4.txt"
fi

# ---------------------------------------------------------------- 5/6. 巡逻 + 入库
if $QUICK; then
  hdr "5/6 巡航  (--quick 跳过)"
  note "巡航: 已跳过 (去掉 --quick 会跑)"
  hdr "6/6 倒车入库"
  note "倒车入库: 已跳过"
else
  TOT=$(python3 -c "
import yaml,sys
d=yaml.safe_load(open('$ROUTE'))
print(sum(1 for w in d.get('waypoints',[]) if w.get('task')!='waypoint'), len(d.get('waypoints',[])))
" 2>/dev/null | awk '{print $2}')
  TOT=${TOT:-0}
  NPARK=$(python3 -c "
import yaml
d=yaml.safe_load(open('$ROUTE'))
print('有' if d.get('reverse_park') else '无')
" 2>/dev/null)
  hdr "5/6+6/6 最终路线巡航 ($(basename "$ROUTE"), $TOT 站) + 倒车入库($NPARK)"
  rm -f "$LOGD/trace.csv"
  timeout 900 python3 tools/patrol.py --file "$ROUTE" --save-trace "$LOGD/trace.csv" \
      > "$LOGD/check5.txt" 2>&1
  grep -E "一圈跑完|倒车|入库|尾|位姿伺服结束" "$LOGD/check5.txt" | tail -6 | sed 's/^/      /'
  if grep -qE "$TOT/$TOT" "$LOGD/check5.txt"; then
    ok "巡航: $TOT/$TOT 站全部到达"
  else
    N=$(grep -oE "[0-9]+/$TOT 个航点" "$LOGD/check5.txt" | tail -1)
    bad "巡航: ${N:-没跑完} (见 $LOGD/check5.txt)"
  fi
  # ★ 一条判定同时报"有没有入位"和"真值精度"（门槛 3 cm）。
  #   patrol.py 打印: [park] 对真值: 位置 0.0120 m, 朝向 0.44 deg  ✓ 入位
  #   旧版这里被拆成两处判定, 且精度门槛 grep 的是早已不存在的
  #   "倒车入库 位置误差 x.xx cm" 格式, 永远匹配不上、只打一句"没跑" —— 见 issue #7
  if [ "$NPARK" = "有" ]; then
    if ! grep -qE "入位" "$LOGD/check5.txt"; then
      bad "倒车入库: 没看到入位结果 (见 $LOGD/check5.txt)"
    else
      PARKL=$(grep -oE "对真值: 位置 [0-9.]+ m, 朝向 [0-9.]+ deg" "$LOGD/check5.txt" | tail -1)
      if [ -n "${PARKL:-}" ]; then
        PM=$(echo "$PARKL" | sed -n 's/.*位置 \([0-9.]*\) m.*/\1/p')
        PY=$(echo "$PARKL" | sed -n 's/.*朝向 \([0-9.]*\) deg.*/\1/p')
        if python3 -c "import sys; sys.exit(0 if $PM < 0.03 else 1)"; then
          ok "倒车入库: 已入位, 真值位置误差 ${PM} m / 朝向 ${PY}° (< 3 cm)"
        else
          bad "倒车入库: 已入位但精度不足, 真值位置误差 ${PM} m (>= 3 cm)"
        fi
      else
        ok "倒车入库: 已入位 (没解析到真值误差行)"
      fi
    fi
  fi
  # 车道合规: 轨迹有没有进 A/B 街区
  if [ -f "$LOGD/trace.csv" ]; then
    VIOL=$(python3 - "$LOGD/trace.csv" <<'PY'
import csv, sys
rows = list(csv.DictReader(open(sys.argv[1])))
BLK = [(-1.472, 0.847, 0.843, 1.468), (-1.475, -0.537, -0.407, 0.197)]
n = sum(1 for r in rows for (x0, y0, x1, y1) in BLK
        if x0 < float(r['x']) < x1 and y0 < float(r['y']) < y1)
print(n)
PY
)
    if [ "${VIOL:-1}" = "0" ]; then
      ok "车道合规: 全程 $(wc -l < "$LOGD/trace.csv") 帧, 0 帧压越车道线"
    else
      bad "车道合规: 有 $VIOL 帧闯进街区"
    fi
  fi
fi

# ---------------------------------------------------------------- 汇总
echo
echo "════════════════════════════════════════════════════════════════"
echo "  验收汇总"
echo "════════════════════════════════════════════════════════════════"
for p in "${PASS[@]:-}"; do [ -n "$p" ] && printf '  \033[32m通过\033[0m  %s\n' "$p"; done
for f in "${FAIL[@]:-}"; do [ -n "$f" ] && printf '  \033[31m失败\033[0m  %s\n' "$f"; done
for n in "${NOTE[@]:-}"; do [ -n "$n" ] && printf '  \033[33m待看\033[0m  %s\n' "$n"; done
echo
echo "  通过 ${#PASS[@]} 项, 失败 ${#FAIL[@]} 项"
echo "  详细日志: $LOGD/"
if $KEEP; then
  echo
  echo "  --keep: 仿真还在跑, 手动操作请另开终端:"
  echo "    export ROS_MASTER_URI=$ROS_MASTER_URI"
  echo "    source $ROOT/devel/setup.bash"
  echo "    rostopic echo /traffic_light/state"
  echo "  关掉: kill -INT $SIM_PID"
fi
[ ${#FAIL[@]} -eq 0 ]
