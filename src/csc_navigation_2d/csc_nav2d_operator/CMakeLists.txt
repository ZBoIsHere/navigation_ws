cmake_minimum_required(VERSION 2.8.3)
project(csc_nav2d_operator)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  tf
  costmap_2d
  sensor_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(FILES cmd.msg)

## Generate added messages and services with any dependencies listed here
generate_messages()

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES RobotOperator
  CATKIN_DEPENDS roscpp message_runtime tf costmap_2d sensor_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a cpp library
add_library(RobotOperator src/RobotOperator.cpp)
target_link_libraries(RobotOperator ${catkin_LIBRARIES})

## Declare a cpp executable
add_executable(operator src/node.cpp)
target_link_libraries(operator RobotOperator)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(RobotOperator ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

