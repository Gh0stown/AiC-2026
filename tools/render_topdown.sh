#!/usr/bin/env bash
# Render one top-down frame of a Gazebo world, offscreen (no GUI, no window).
#
#   usage: render_topdown.sh <world_file> <out_png> [camera_z] [fov_rad]
#
# Uses a downward camera + gazebo_ros_camera to publish one frame, saved as a PNG.
# Used to check that the floor texture lines up with the wall geometry.
set -e
WORLD="$1"; OUT="$2"; CZ="${3:-30.0}"; FOV="${4:-0.15}"
HERE="$(cd "$(dirname "$0")" && pwd)"
WS="$(cd "$HERE/.." && pwd)"
PKG="$WS/src/competition_arena"
TMP="$WS/.verify"
mkdir -p "$TMP"

python3 - "$WORLD" "$TMP/_cam.world" "$CZ" "$FOV" <<'PY'
import sys
src, dst, cz, fov = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4]
cam = '''
    <model name="topdown_cam">
      <static>true</static>
      <pose>0 0 %s 0 1.5707963 0</pose>
      <link name="link">
        <sensor name="cam" type="camera">
          <always_on>true</always_on>
          <update_rate>5</update_rate>
          <camera>
            <horizontal_fov>%s</horizontal_fov>
            <image><width>1000</width><height>1000</height><format>R8G8B8</format></image>
            <clip><near>0.05</near><far>100</far></clip>
          </camera>
          <plugin name="cam_plugin" filename="libgazebo_ros_camera.so">
            <cameraName>arena_topdown</cameraName>
            <imageTopicName>image_raw</imageTopicName>
            <cameraInfoTopicName>camera_info</cameraInfoTopicName>
            <frameName>map</frameName>
            <alwaysOn>true</alwaysOn>
            <updateRate>5.0</updateRate>
          </plugin>
        </sensor>
      </link>
    </model>
''' % (cz, fov)
open(dst, 'w').write(open(src).read().replace('</world>', cam + '</world>'))
PY

source /opt/ros/noetic/setup.bash
[ -f /usr/share/gazebo/setup.sh ] && source /usr/share/gazebo/setup.sh
export GAZEBO_RESOURCE_PATH="$PKG:${GAZEBO_RESOURCE_PATH:-/usr/share/gazebo-11}"
export DISPLAY="${DISPLAY:-:1}"

# Headless / VM setups with no usable $HOME or GLX: export DSH_SIM_HOME and/or
# LIBGL_ALWAYS_SOFTWARE before calling this script.
if [ -n "${DSH_SIM_HOME:-}" ]; then
  export HOME="$DSH_SIM_HOME"; mkdir -p "$HOME/.ros" "$HOME/.gazebo"
  export ROS_LOG_DIR="$HOME/.ros/log"
fi

PORT=$((11399 + RANDOM % 400))
export ROS_MASTER_URI="http://127.0.0.1:$PORT"
roscore -p "$PORT" > "$TMP/roscore.log" 2>&1 & RC=$!
sleep 5
gzserver -e ode -s libgazebo_ros_api_plugin.so "$TMP/_cam.world" > "$TMP/gzserver.log" 2>&1 & GZ=$!
sleep 11
python3 "$HERE/capture_topdown.py" "$OUT" > "$TMP/capture.log" 2>&1 || true
kill $GZ $RC 2>/dev/null || true
sleep 1; kill -9 $GZ $RC 2>/dev/null || true
if [ -f "$OUT" ]; then echo "OK -> $OUT"; else
  echo "RENDER FAILED"; tail -20 "$TMP/gzserver.log"; exit 1
fi
