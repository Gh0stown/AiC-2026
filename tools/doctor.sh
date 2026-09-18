#!/usr/bin/env bash
# 仿真体检: 起完仿真 (或 rviz) 之后单独跑这个, 看哪一环断了
#   usage:  bash tools/doctor.sh
cd "$(dirname "$0")/.."
source /opt/ros/noetic/setup.bash
source devel/setup.bash 2>/dev/null

echo "=========== 1. 有没有残留进程 ==========="
for n in gzserver gzclient rosmaster roslaunch spawn_model; do
  c=$(pgrep -c "$n" 2>/dev/null || echo 0)
  printf "  %-14s %s" "$n" "$c"
  [ "$c" -gt 1 ] && echo "   ⚠ 多于一个, 可能有残留" || echo ""
done
echo "  端口 11311: $(ss -tln 2>/dev/null | grep -q ':11311 ' && echo '占用' || echo '空闲')"
echo "  端口 11345: $(ss -tln 2>/dev/null | grep -q ':11345 ' && echo '占用' || echo '空闲')"

echo
echo "=========== 2. 话题在不在发 ==========="
printf "  %-28s %-28s %s\n" "话题" "类型" "频率"
for t in /scan /points /odom /odom_groundtruth /joint_states /imu /camera/rgb/image_raw /map; do
  if rostopic list 2>/dev/null | grep -qx "$t"; then
    ty=$(rostopic type "$t" 2>/dev/null | sed 's|.*/||')
    r=$(timeout 5 rostopic hz --window=4 "$t" 2>/dev/null | grep -m1 'average rate' | grep -oE '[0-9.]+$')
    printf "  %-28s %-28s %s\n" "$t" "$ty" "${r:-（本次没采到）}"
  else
    printf "  %-28s %s\n" "$t" "✗ 不存在"
  fi
done

echo
echo "=========== 3. 相机到底出不出图 ==========="
if timeout 6 rostopic echo -n1 /camera/rgb/image_raw 2>/dev/null | grep -q "width"; then
  timeout 6 rostopic echo -n1 /camera/rgb/image_raw 2>/dev/null \
    | grep -E "^\s+(width|height|encoding|frame_id)" | sed 's/^/    /'
  echo "    ✅ 相机有数据"
else
  echo "    ✗ /camera/rgb/image_raw 没有数据"
fi
echo "  注意: 深度图已按需求关掉, /camera/depth/* 不存在是正常的"

echo
echo "=========== 4. TF 树 ==========="
timeout 6 rostopic echo /tf 2>/dev/null | grep -E "frame_id:" | sort -u | sed 's/^/    /' | head -8
timeout 6 rostopic echo /tf_static 2>/dev/null | grep -E "frame_id:" | sort -u | sed 's/^/    (static) /' | head -8

echo
echo "=========== 5. 发指令车动不动 ==========="
get() { timeout 6 rosservice call /gazebo/get_model_state "{model_name: 'competition_robot'}" 2>/dev/null \
        | grep -A3 "^pose:" | grep -E "^\s+(x|y):" | tr -d ' ' | tr '\n' ' '; }
A=$(get); echo "    前: $A"
timeout 5 rostopic pub -r 20 /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null 2>&1
sleep 1
B=$(get); echo "    后: $B"
[ "$A" = "$B" ] && echo "    ⚠ 车没动!" || echo "    ✅ 车动了"
echo
echo "=========== 体检结束 ==========="
