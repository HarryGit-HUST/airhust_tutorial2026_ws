cmake_minimum_required(VERSION 3.0.2)
project(tutorial_pcl_basic)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  pcl_ros
  pcl_conversions
)

# 阶段三只用得到这几个模块：读写、通用、滤波、分割、表面、kdtree
find_package(PCL REQUIRED COMPONENTS common io filters segmentation surface kdtree features)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs pcl_ros pcl_conversions
  DEPENDS PCL
)

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# 数据在 ws 根目录的 data/，输出统一丢到 ws 根目录的 out/
get_filename_component(TUTORIAL_WS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
message(STATUS "tutorial ws dir: ${TUTORIAL_WS_DIR}")

function(tutorial_pcl_exe name src)
  add_executable(${name} ${src})
  target_link_libraries(${name} ${catkin_LIBRARIES} ${PCL_LIBRARIES})
  target_compile_definitions(${name} PRIVATE
    DATA_DIR="${TUTORIAL_WS_DIR}/data"
    OUT_DIR="${TUTORIAL_WS_DIR}/out")
  add_dependencies(${name} ${catkin_EXPORTED_TARGETS})
endfunction()

# 学生要改的就是下面 src/ 里这几个文件
foreach(s 01_io 02_publish 03_filter 05_cluster)
  tutorial_pcl_exe(s${s} src/${s}.cpp)
endforeach()

# 在线综合考核，处理函数由学生补全。
tutorial_pcl_exe(s07_pipeline src/07_pipeline.cpp)
