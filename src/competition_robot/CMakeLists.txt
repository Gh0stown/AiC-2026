cmake_minimum_required(VERSION 3.0.2)
project(competition_robot)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
  tf
  gazebo_ros
  gazebo_plugins
)

find_package(gazebo REQUIRED)

catkin_package(
  LIBRARIES competition_holonomic_drive
  CATKIN_DEPENDS roscpp nav_msgs geometry_msgs tf gazebo_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

# ---------------------------------------------------------------------------
#  全向(麦克纳姆)底盘驱动插件
#
#  替代 gazebo_ros_planar_move。后者实测实际自转只有指令的 0.637 倍,
#  原因见 src/holonomic_drive_plugin.cpp 开头的说明。
#  <velocityMode> 可以在 model / all_rigid / canonical 之间切换做对比。
# ---------------------------------------------------------------------------
add_library(competition_holonomic_drive src/holonomic_drive_plugin.cpp)
target_link_libraries(competition_holonomic_drive
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)
add_dependencies(competition_holonomic_drive ${catkin_EXPORTED_TARGETS})
set_target_properties(competition_holonomic_drive PROPERTIES
  OUTPUT_NAME competition_holonomic_drive)

# ---------------------------------------------------------------------------
#  Python 节点 (仿真/真机共用)
# ---------------------------------------------------------------------------
catkin_install_python(PROGRAMS
  scripts/gen_robot.py
  scripts/mecanum.py
  scripts/mecanum_odometry.py
  scripts/sim_wheel_encoders.py
  scripts/pointcloud_to_scan.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS competition_holonomic_drive
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY config urdf launch rviz docs meshes
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
