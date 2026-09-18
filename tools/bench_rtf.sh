#!/usr/bin/env bash
WS="$(cd "$(dirname "$0")/.." && pwd)"
source /opt/ros/noetic/setup.bash; source "$WS/devel/setup.bash"
[ -n "${DSH_SIM_HOME:-}" ] && { export HOME="$DSH_SIM_HOME"; mkdir -p "$HOME/.ros"; export LIBGL_ALWAYS_SOFTWARE=1; }
export ROS_LOG_DIR="${HOME}/.ros/log"; export DISPLAY=:0
pkill -9 -f "[g]zserver"; pkill -9 -f "[r]osmaster"; sleep 2
mkdir -p "$WS/.verify"
export ROS_MASTER_URI=http://localhost:12010
export GAZEBO_MASTER_URI=http://localhost:12020
roslaunch --port=12010 competition_robot robot_gazebo.launch gui:=false rviz:=false > "$WS/.verify/b.log" 2>&1 & SIM=$!
sleep 40

rtf() {  # 用 /clock 和墙钟算实时率
  python3 - <<'PY'
import subprocess, time, re
def clock():
    o = subprocess.run(['rostopic','echo','-n1','/clock'], capture_output=True, text=True, timeout=10).stdout
    m = re.search(r'secs:\s*(\d+)', o); n = re.search(r'nsecs:\s*(\d+)', o)
    return int(m.group(1)) + int(n.group(1))/1e9 if m and n else None
a = clock(); w0 = time.time(); time.sleep(6); b = clock(); w1 = time.time()
print('  RTF = %.3f   (仿真时间 %.2fs / 墙钟 %.2fs)' % ((b-a)/(w1-w0), b-a, w1-w0))
PY
}
echo "=== 当前配置 (3D雷达 16x360@10Hz + RGB相机 640x480@15Hz) ==="
rtf
echo "=== 各传感器的 CPU 占用 ==="
top -bn1 -p $(pgrep -d, gzserver) 2>/dev/null | tail -3 | sed 's/^/  /'
kill -INT $SIM 2>/dev/null; sleep 3; kill -9 $SIM 2>/dev/null
pkill -9 -f "[g]zserver"; pkill -9 -f "[r]osmaster"
