cmake_minimum_required(VERSION 2.8.3)
project(qnx2ros)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf
)
catkin_package(
  CATKIN_DEPENDS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  ${PCL_OPENMP_PACKAGES}
  DEPENDS
)
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(moving_average
   src/moving_average.cpp
)

## Declare a cpp executable
add_executable(qnx2ros
  src/receive_robot_status_node.cpp
)
add_dependencies(qnx2ros
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(qnx2ros
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  moving_average
)