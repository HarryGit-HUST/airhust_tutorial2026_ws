cmake_minimum_required(VERSION 3.0.2)
project(airhust_complete_mission)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS scripts/spawn_random.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch worlds models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
