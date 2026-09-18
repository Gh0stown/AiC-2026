#!/usr/bin/env bash
# 启动 4.2x4.2 场地 + 麦克纳姆机器人 + Gazebo 界面 + RViz
# 用法:  bash tools/run_gui.sh
cd "$(dirname "$0")/.."
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch competition_robot robot_gazebo.launch "$@"
