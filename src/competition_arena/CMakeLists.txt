cmake_minimum_required(VERSION 3.0.2)
project(competition_arena)

find_package(catkin REQUIRED)

catkin_package()

# 纯资源包 (worlds / launch / urdf / maps / media / scripts), 无编译目标。
# 场地几何与贴图由 scripts/build_arena.py 生成, 可重复执行。

install(DIRECTORY launch urdf worlds maps rviz media scripts tools
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
