cmake_minimum_required(VERSION 2.8.3)
project(ndt_localization)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(PCL REQUIRED)
#find_package(CUDA)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

# if (CUDA_FOUND)
#     add_definitions(-DCUDA_FOUND)
#     list(APPEND PCL_OPENMP_PACKAGES ndt_gpu)
# endif ()

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
  # ${PCL_OPENMP_PACKAGES}
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES 
 CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs tf ${PCL_OPENMP_PACKAGES}
 DEPENDS system_lib PCL
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

string(REPLACE ":" ";" LIBRARY_DIRS $ENV{LD_LIBRARY_PATH})
link_directories(${LIBRARY_DIRS})  # 可以将libgpu_ndt.so加到这个路径里，然后就可以链接到了

include_directories(include)
# link_directories(lib)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)



# if (NOT (PCL_VERSION VERSION_LESS "1.7.2"))
#     set_target_properties(ndt_localization_node PROPERTIES COMPILE_DEFINITIONS "USE_PCL_OPENMP")
# endif (NOT (PCL_VERSION VERSION_LESS "1.7.2"))

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

# add_executable(ndt_test
#   src/ndt_test.cpp
# )

# target_link_libraries(ndt_test
#   ${catkin_LIBRARIES}
#   ${PCL_LIBRARIES}
# )

# add_executable(tf_test
#   src/tf_test.cpp
# )

# add_dependencies(tf_test
#   ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
# )

# target_link_libraries(tf_test
#   ${catkin_LIBRARIES}
# )



