cmake_minimum_required(VERSION 2.8.3)
project(csc_nav2d_navigator)
SET(CMAKE_CXX_FLAGS "-std=c++11 -g -O3 ${CMAKE_CXX_FLAGS}")
## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  tf
  pluginlib
  actionlib
  actionlib_msgs
  geometry_msgs
  csc_nav2d_operator
  csc_nav2d_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate actions in the 'action' folder
add_action_files(
  FILES
  MoveToPosition2D.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES RobotNavigator MapInflationTool
  CATKIN_DEPENDS
    roscpp
    message_runtime
    tf
    actionlib
    actionlib_msgs
    geometry_msgs
    csc_nav2d_msgs
    csc_nav2d_operator
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} )

## Declare a cpp library
add_library(MapInflationTool src/MapInflationTool.cpp)
add_library(RobotNavigator src/RobotNavigator.cpp)

## Declare a cpp executable
add_executable(navigator src/navigator.cpp src/RobotNavigator.cpp)
add_executable(set_goal_client src/set_goal_client.cpp)
add_executable(set_goal_track_ball src/set_goal_track_ball.cpp)

add_executable(ObstaclesDetect src/velodyne_ObstaclesDetect.cpp )
target_link_libraries(ObstaclesDetect ${catkin_LIBRARIES})

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(RobotNavigator ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(navigator ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(set_goal_client ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(set_goal_track_ball ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
## Specify libraries to link a library or executable target against
target_link_libraries(RobotNavigator MapInflationTool ${catkin_LIBRARIES})
target_link_libraries(navigator RobotNavigator ${catkin_LIBRARIES})
target_link_libraries(set_goal_client ${catkin_LIBRARIES})
target_link_libraries(set_goal_track_ball ${catkin_LIBRARIES})
