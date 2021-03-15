cmake_minimum_required(VERSION 3.0.2)
project(autonomy)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rviz
  tf_conversions
  tf2
  trajectory_msgs
  visualization_msgs
)
catkin_package()

find_package(Eigen3 REQUIRED)

set(CMAKE_AUTOMOC ON)

# Qt 4 or 5
if("${rviz_QT_VERSION}" VERSION_LESS "5")
  find_package(Qt4 ${rviz_QT_VERSION} REQUIRED QtCore QtGui)
  include(${QT_USE_FILE})
  macro(qt_wrap_ui)
    qt4_wrap_ui(${ARGN})
  endmacro()
else()
  find_package(Qt5 ${rviz_QT_VERSION} REQUIRED Core Widgets)
  set(QT_LIBRARIES Qt5::Widgets)
  macro(qt_wrap_ui)
    qt5_wrap_ui(${ARGN})
  endmacro()
endif()

add_definitions(-DQT_NO_KEYWORDS)

include_directories(include ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME} src/autonomy.cpp include/autonomy/autonomy.h)
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})