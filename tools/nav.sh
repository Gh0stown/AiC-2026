#!/usr/bin/env bash
# 一键启动导航 —— 自动处理那些"新开终端就会忘"的事。
#
#   tools/nav.sh                    # 仿真 + 静态地图 + AMCL + move_base + RViz (DWA)
#   tools/nav.sh planner:=teb       # 换局部规划器 (dwa / teb / traj)
#   tools/nav.sh gplan:=global      # 换全局规划器 (navfn / global)
#   tools/nav.sh sim:=false         # 仿真已经在跑了, 不要重复起
#   tools/nav.sh gui:=false rviz:=false   # 都不开界面 (无显卡/远程时)
#   tools/nav.sh --build            # 先 catkin_make 再启动
#
# 为什么需要这个脚本:
#   `roslaunch competition_robot navigation.launch` 要求当前 shell 里
#   **source 过本工作区的 devel/setup.bash**, 否则会报:
#       RLException: [navigation.launch] is neither a launch file in package
#       [competition_robot] nor is [competition_robot] a launch file name
#   而 ~/.bashrc 通常只 source 了 /opt/ros/noetic/setup.bash (fishros 一键装的就是这样),
#   不含工作区。新开一个终端就很容易忘。本脚本把这件事自动化了。

set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

# ------------------------------------------------------------------ 1. ROS
# 注意: ROS 的 setup.*.sh 会引用未定义变量 (如 $ROS_DISTRO), 在 `set -u` 下
# 会直接报 "unbound variable" 并中止, 所以 source 期间要临时关掉 -u。
if [ -f /opt/ros/noetic/setup.bash ]; then
    set +u
    # shellcheck disable=SC1091
    source /opt/ros/noetic/setup.bash
    set -u
elif ! command -v roslaunch >/dev/null 2>&1; then
    echo "错误: 找不到 ROS Noetic (/opt/ros/noetic/setup.bash)" >&2
    exit 1
fi

# ------------------------------------------------------- 2. 可选: 先编译
if [ "${1:-}" = "--build" ]; then
    shift
    echo "==> catkin_make"
    catkin_make
fi

if [ ! -f devel/setup.bash ]; then
    echo "==> 还没有编译过, 先跑 catkin_make ..."
    catkin_make
fi

# ------------------------------------------------------------ 3. 工作区
set +u
# shellcheck disable=SC1091
source devel/setup.bash
set -u

if ! rospack find competition_robot >/dev/null 2>&1; then
    echo "错误: 仍然找不到 competition_robot 包, 请检查 devel/setup.bash" >&2
    exit 1
fi

# --------------------------------------------------------- 4. 避开 venv
# 视觉环境的 .venv 里有 numpy 1.24 / opencv 4.10, 会遮蔽系统的 1.17 / 4.2,
# 而 ROS 的 Python 脚本节点用的是 `#!/usr/bin/env python3`,
# venv 一激活就会走 venv 的解释器, 可能让节点行为异常。
if [ -n "${VIRTUAL_ENV:-}" ]; then
    _VENV="$VIRTUAL_ENV"
    echo "⚠  检测到已激活的虚拟环境: $_VENV"
    echo "   本脚本会在自己的进程内忽略它 (ROS 用系统 Python 更稳)"
    unset VIRTUAL_ENV
    PATH="$(printf '%s' "$PATH" | tr ':' '\n' | grep -vx "$_VENV/bin" | paste -sd:)"
    export PATH
fi

# ------------------------------------------------------- 5. 残留进程提醒
_residual=0
for n in gzserver gzclient rosmaster; do
    c=$(pgrep -c "$n" 2>/dev/null || true)
    if [ "${c:-0}" -gt 0 ]; then
        echo "⚠  检测到 $c 个残留的 $n 进程"
        _residual=1
    fi
done
if [ "$_residual" = 1 ]; then
    echo "   上次可能没退干净, Gazebo 会起不来。清理: pkill -f gzserver; pkill -f rosmaster"
    echo "   (继续启动中, 若失败请先清理)"
fi

# ---------------------------------------------------------------- 6. 启动
echo "==> roslaunch competition_robot navigation.launch $*"
exec roslaunch competition_robot navigation.launch "$@"
