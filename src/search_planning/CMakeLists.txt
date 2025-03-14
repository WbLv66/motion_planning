cmake_minimum_required(VERSION 3.0.2)
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_STANDARD 14)
set(CONAN_GEN_DIR "${CMAKE_BINARY_DIR}/Debug/generators")
message("Conan Generators Directory: ${CONAN_GEN_DIR}")

# 将 Conan 生成器目录插入到 CMAKE_PREFIX_PATH 开头
list(INSERT CMAKE_PREFIX_PATH 0 "${CONAN_GEN_DIR}")

# set(CMAKE_C_COMPILER "/usr/bin/clang-14")
# set(CMAKE_CXX_COMPILER "/usr/bin/clang++-14")
project(search_planning)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  std_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES search_planning
#  CATKIN_DEPENDS nav_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${PROJECT_SOURCE_DIR}/include/search_planning
  ${catkin_INCLUDE_DIRS}
)
# 添加库
add_library(planer src/planer.cpp)
add_library(bfs src/bfs.cpp)
add_library(dfs src/dfs.cpp)
add_library(dijkstra src/dijkstra.cpp)
add_library(a_star src/a_star.cpp)


add_executable(path_pub src/path_pub.cpp)
target_link_libraries(path_pub
  ${catkin_LIBRARIES}
  planer
  bfs
  dfs
  dijkstra
  a_star
)