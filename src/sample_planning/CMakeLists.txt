cmake_minimum_required(VERSION 3.0.2)
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_STANDARD 14)
set(CONAN_GEN_DIR "${CMAKE_BINARY_DIR}/Debug/generators")
message("Conan Generators Directory: ${CONAN_GEN_DIR}")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 将 Conan 生成器目录插入到 CMAKE_PREFIX_PATH 开头
list(INSERT CMAKE_PREFIX_PATH 0 "${CONAN_GEN_DIR}")

project(sample_planning)



## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sample_planning
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${PROJECT_SOURCE_DIR}/include/sample_planning
  ${catkin_INCLUDE_DIRS}
)
# 添加库
add_library(prm src/prm.cpp)



# add_executable(path_pub src/path_pub.cpp)
# target_link_libraries(path_pub
#   ${catkin_LIBRARIES}
  
# )