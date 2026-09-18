#!/usr/bin/env bash
WS="$(cd "$(dirname "$0")/.." && pwd)"
source /opt/ros/noetic/setup.bash; source "$WS/devel/setup.bash"
[ -n "${DSH_SIM_HOME:-}" ] && { export HOME="$DSH_SIM_HOME"; mkdir -p "$HOME/.ros";
  export LIBGL_ALWAYS_SOFTWARE=1; }   # 只在无 GPU 的沙箱里生效
export ROS_LOG_DIR="${HOME}/.ros/log"; export DISPLAY="${DISPLAY:-:1}"
pkill -9 -f "[g]zserver" 2>/dev/null; pkill -9 -f "[r]osmaster" 2>/dev/null; sleep 2
mkdir -p "$WS/.verify"
export ROS_MASTER_URI=http://localhost:11550
export GAZEBO_MASTER_URI=http://localhost:11560
roslaunch --port=11550 competition_robot robot_gazebo.launch gui:=false rviz:=false > "$WS/.verify/l3d.log" 2>&1 & LP=$!
sleep 40
kill -0 $LP 2>/dev/null || { echo "启动失败"; tail -20 "$WS/.verify/l3d.log"; exit 1; }

echo "=== 话题 ==="
for t in /points /scan /odom /joint_states /imu /camera/depth/image_raw; do
  rostopic list 2>/dev/null | grep -qx "$t" && echo "  [OK] $t" || echo "  [缺失] $t"
done

echo "=== /points 概况 (点数/每线点数/帧) ==="
timeout 10 rostopic echo -n1 /points 2>/dev/null | grep -E "^(height|width|point_step|is_dense|frame_id)" | sed 's/^/  /' | head -6
echo "=== /scan 概况 ==="
timeout 10 rostopic echo -n1 /scan 2>/dev/null | grep -E "^(angle_min|angle_max|angle_increment|range_min|range_max|frame_id)" | sed 's/^/  /' | head -6
echo "  距离分布:"
timeout 12 rostopic echo -n1 /scan/ranges 2>/dev/null | python3 -c "
import sys, re, collections
t = sys.stdin.read()
vals = re.findall(r'inf|[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?', t)
real = sorted(float(x) for x in vals if x != 'inf')
print('    总 bin %d, 有回波 %d, 无回波 %d' % (len(vals), len(real), len(vals)-len(real)))
if real:
    print('    最近 %.2f m  中位 %.2f m  最远 %.2f m' % (real[0], real[len(real)//2], real[-1]))
    h = collections.Counter(min(int(x), 9) for x in real)
    for k in sorted(h):
        print('      %d~%d m : %s' % (k, k+1, '#' * min(h[k]//4, 60)))
"
echo "=== 频率 ==="
for t in /points /scan; do
  r=$(timeout 8 rostopic hz --window=8 $t 2>/dev/null | grep -m1 "average rate" | grep -oE "[0-9.]+$")
  printf "  %-10s %s Hz\n" "$t" "${r:-无数据}"
done
kill -INT $LP 2>/dev/null; sleep 3; kill -9 $LP 2>/dev/null
pkill -9 -f "[g]zserver" 2>/dev/null; pkill -9 -f "[r]osmaster" 2>/dev/null
echo "=== done ==="
