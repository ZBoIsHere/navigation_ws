cmake_minimum_required(VERSION 2.8.3)
project(message_transformer_cpp)

SET(CMAKE_CXX_FLAGS "-std=c++11 -g -O3 ${CMAKE_CXX_FLAGS}")
set(CMAKE_BUILD_TYPE Release)
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
  DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(receiver
  src/receiver.cpp
)
target_link_libraries(receiver
  ${catkin_LIBRARIES}
)

add_executable(sender
  src/sender.cpp
)
target_link_libraries(sender
  ${catkin_LIBRARIES}
)

install(TARGETS receiver sender
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)