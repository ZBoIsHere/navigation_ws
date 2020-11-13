cmake_minimum_required(VERSION 2.8.3)
project(csc_nav2d_msgs)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  sensor_msgs
  geometry_msgs
)

add_message_files(FILES
  LocalizedScan.msg
  RobotPose.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES 
  std_msgs
  sensor_msgs
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    message_runtime
	std_msgs
	sensor_msgs
	geometry_msgs
)
