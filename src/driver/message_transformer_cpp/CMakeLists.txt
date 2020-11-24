cmake_minimum_required(VERSION 2.8.3)
project(message_transformer_cpp)

# add_compile_options(-std=c++11)
SET(CMAKE_CXX_FLAGS "-std=c++11 -g -O3 ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf
  message_generation
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

add_library(input_socket
  src/input.cc
)

## Declare a cpp executable
add_executable(qnx2ros
  src/qnx2ros.cpp
)
target_link_libraries(qnx2ros
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  moving_average
)

add_executable(ros2qnx
  src/ros2qnx.cpp
)
target_link_libraries(ros2qnx
  ${catkin_LIBRARIES}
  input_socket
)
