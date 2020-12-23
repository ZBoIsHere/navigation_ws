cmake_minimum_required(VERSION 2.8.3)
project(ndt_localization)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  nav_msgs
  rosbag
  sensor_msgs
  std_msgs
  visualization_msgs
  tf
  tf_conversions
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES 
 CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs tf ${PCL_OPENMP_PACKAGES}
 DEPENDS system_lib PCL
)

string(REPLACE ":" ";" LIBRARY_DIRS $ENV{LD_LIBRARY_PATH})
link_directories(${LIBRARY_DIRS})

include_directories(include)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(ndt_localization_node
  src/ndt_localization_node.cpp
)

add_dependencies(ndt_localization_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(ndt_localization_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)