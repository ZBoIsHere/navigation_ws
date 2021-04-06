- [自主导航程序使用说明](#自主导航程序使用说明)
- [版本说明](#版本说明)
- [常用指令](#常用指令)
- [如何生成 ROS 安装包文件](#如何生成-ros-安装包文件)
  - [如何安装可执行文件、动态库或静态库等二进制文件](#如何安装可执行文件动态库或静态库等二进制文件)
  - [如何安装库的头文件](#如何安装库的头文件)
  - [安装 launch、YAML、RViz、地图等普通文本文件](#安装-launchyamlrviz地图等普通文本文件)
  - [安装 Python 程序](#安装-python-程序)
  - [打包 deb 文件](#打包-deb-文件)

## 自主导航程序使用说明

注：考虑到运行性能与可靠性，强烈建议用[**主从机方式**](https://blog.csdn.net/Spacegene/article/details/86499467)调试

1. 配置参数
   
   打开 `navigation_ws/src/navigation/nav/launch/robot.launch` 文件，参考配置：

   ```xml
   <launch>
     <!-- 绝影型号 MINI/JYL/LITE -->
     <arg name="robot_type" default="MINI" />
     <!-- 雷达型号 rslidar(默认)/velodyne -->
     <arg name="lidar_type" default="rslidar" />
     <!-- 开启定位 false(默认定位程序与导航程序单独开启)/true(采用导航自带的定位程序) -->
     <arg name="enable_lidar_localization" default="false" />
     <!-- 开启RViz true(默认)/false -->
     <arg name="enable_rviz" default="true" />
     <!-- 点云/占据栅格地图名称 map(默认) -->
     <arg name="map_id" default="map" />
     ...
   </launch>
   ```

2. 安装编译 (如果无源码请忽略此步骤)
   
   如果采用导航自带的定位程序且启用 velodyne 激光雷达，需要执行一次：

   ```bash
   $ cd ~/navigation_ws && mv src/driver/velodyne/CATKIN_IGNORE src/driver/ros_rslidar
   ```
   否则直接执行：

   ```bash
   $ sudo apt-get install ros-$ROS_DISTRO-navigation ros-$ROS_DISTRO-teb-local-planner ros-$ROS_DISTRO-spatio-temporal-voxel-layer libproj-dev libpcap-dev
   $ cd ~/navigation_ws && catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_WHITELIST_PACKAGES=""
   ```

3. 调节导航参数，参数文件位于 `navigation_ws/src/navigation/nav/config`，修改时务必**理解参数意义**
4. 启动导航程序
   
   如果采用 **TeamViewer 调试**，机器人端执行：
   
   ```bash
   $ cd ~/navigation_ws && source devel/setup.bash
   $ roslaunch nav robot.launch
   ```
   
   如果采用 **ROS 主从机调试**，机器人端执行：
   
   ```bash
   $ cd ~/navigation_ws && source devel/setup.bash
   $ roslaunch nav robot.launch
   ```
   
   PC 端执行：
   
   ```bash
   $ cd ~/navigation_ws && source devel/setup.bash
   $ roslaunch nav pc.launch
   ```
## 版本说明

- v0.2@2021/04/06
  - 支持 velodyne/rslidar 两种激光
  - 支持 MINI/JYL/LITE 三种机器人
  - 默认不启用 D435 相机

- v0.1@2020/12/23
  - 采用角速度控制
  - 最大线速度 0.5m/s
  - 融合 LIDAR 与 D435 避障
  - 导航点误差小于 0.1m

## 常用指令

```bash
$ rosrun rqt_tf_tree rqt_tf_tree
$ rosrun tf tf_echo from to
$ rqt_graph
$ rqt_logger_level
$ rqt_console
$ catkin config -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER_ARG1=-std=c++11
$ catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release
# 代码格式化
$ find . -iname '*.h' -o -iname '*.hh' -o -iname '*.hpp' -o -iname '*.c' -o -iname '*.cc' -o -iname '*.cpp' | xargs clang-format -i
$ black name-of-package
$ find . -name '*.DS_Store' -delete
# 固件替换
$ telnet 192.168.1.120
$ cd /home/bin
$ chmod 777 new_firmware
$ ln -sf new_firmware robot_control_system
$ shutdown
# 调试工具
$ sudo apt-get install ros-$ROS_DISTRO-plotjuggler python-catkin-tools vim htop
```

## [如何生成 ROS 安装包文件](http://wiki.ros.org/catkin/CMakeLists.txt#Optional_Step:_Specifying_Installable_Targets)

下面通过修改 `CMakeLists.txt` 指定 deb 安装包所含内容：
### 如何安装可执行文件、动态库或静态库等二进制文件

```cmake
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

### 如何安装库的头文件

```cmake
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
```

### 安装 launch、YAML、RViz、地图等普通文本文件

```cmake
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)
install(FILES nodelet_velodyne.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
```

### 安装 Python 程序

```cmake
install(TARGETS python_module_library
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/myscript
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
```

### 打包 deb 文件

```bash
$ sudo apt-get install python-bloom
$ cd path/to/your/catkin/package
$ bloom-generate rosdebian
$ fakeroot debian/rules binary
# $ bloom-generate rosdebian --os-name ubuntu --os-version xenial --ros-distro kinetic
# $ fakeroot debian/rules binary
```

* [How to make a debian from a ROS package](https://gist.github.com/awesomebytes/196eab972a94dd8fcdd69adfe3bd1152)