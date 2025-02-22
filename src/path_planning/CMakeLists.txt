cmake_minimum_required(VERSION 3.0.2)
set(CMAKE_BUILD_TYPE "Debug")
# set(CMAKE_C_COMPILER "/usr/bin/clang-14")
# set(CMAKE_CXX_COMPILER "/usr/bin/clang++-14")
project(path_planning)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  std_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES path_planning
#  CATKIN_DEPENDS nav_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${PROJECT_SOURCE_DIR}/include/path_planning
  ${catkin_INCLUDE_DIRS}
)
# 添加库
add_library(planer src/planer.cpp)
add_library(bfs src/bfs.cpp)


add_executable(path_pub src/path_pub.cpp)
target_link_libraries(path_pub
  ${catkin_LIBRARIES}
  planer
  bfs
)