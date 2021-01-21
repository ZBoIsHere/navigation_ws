- [自主导航程序使用说明](#自主导航程序使用说明)
- [版本说明](#版本说明)
  - [TODO](#todo)
  - [~~建图参考 (TODO 优化)~~](#建图参考-todo-优化)
  - [~~常用指令~~](#常用指令)
  - [UDP](#udp)
  - [如何导出 ROS 二进制包](#如何导出-ros-二进制包)
    - [如何安装可执行文件、动态库或静态库等二进制文件](#如何安装可执行文件动态库或静态库等二进制文件)
    - [如何安装库的头文件](#如何安装库的头文件)
    - [安装 roslaunch 文件、YAML等普通文本文件](#安装-roslaunch-文件yaml等普通文本文件)
    - [安装 Python 程序](#安装-python-程序)
    - [生成 deb 文件](#生成-deb-文件)

## 自主导航程序使用说明

注：考虑到运行性能与可靠性，强烈建议用[**主从机方式**](https://blog.csdn.net/Spacegene/article/details/86499467)调试

1. 安装编译，先进入用户目录 ($ cd ~) 解压导航程序包，然后执行：
   ```bash
   $ sudo apt-get install ros-kinetic-navigation ros-kinetic-navigation-experimental ros-kinetic-teb-local-planner ros-kinetic-spatio-temporal-voxel-layer libproj-dev chrony
   $ cd ~/navigation_ws
   $ catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_WHITELIST_PACKAGES=""
   ```
2. 配置参数，先把新地图文件 `XXX.pcd;XXX.pgm;XXX.yaml` (`XXX` 表示地图名称) 存放在 `navigation_ws/src/localization/map/` 目录下；再修改配置文件 `navigation_ws/src/navigation/nav/launch/robot.launch` 的相关参数，
   
   如果采用 **TeamViewer 调试**，配置如下
   ```yaml
   <launch>
     <arg name="robot_type" default="MINI" />
     <arg name="lidar_type" default="rslidar" />
     <arg name="enable_driver" default="true" />
     <arg name="enable_rviz" default="true" />
     <arg name="map_id" default="XXX" />
     <arg name="mprim_id" default="mprim05" />

     <!-- 其他参数略 -->
   </launch>
   ```
   如果采用 **ROS 主从机调试**，配置如下
   ```yaml
   <launch>
     <arg name="robot_type" default="MINI" />
     <arg name="lidar_type" default="rslidar" />
     <arg name="enable_driver" default="true" />
     <arg name="enable_rviz" default="false" />
     <arg name="map_id" default="XXX" />
     <arg name="mprim_id" default="mprim05" />

     <!-- 其他参数略 -->
   </launch>
   ```
   具体参数分别对应：
   - 机器人型号: `MINI`/`JYL`
   - 激光雷达型号 (默认 `rslidar`，如果设置为 `velodyne`，需先删除文件夹 `navigation_ws/src/driver/ros_rslidar-master/` 后解压 `velodyne-master.zip` 文件，并重新编译工作空间或者单独编译激光雷达驱动 `$ catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_WHITELIST_PACKAGES="velodyne;velodyne_driver;velodyne_laserscan;velodyne_msgs;velodyne_pcl;velodyne_pointcloud"`): `rslidar`/`velodyne`
   - 是否开启激光与通信驱动: `true`/`false`
   - 是否开启 RViz: `true`/`false`
   - 导航地图的名称: `XXX`
   - 导航地图的分辨率: `mprim05`/`mprim10`，对应 5cm/10cm
3. 调优导航参数，参数文件位于 `navigation_ws/src/navigation/nav/config/`，请务必**在熟悉参数意义的前提下**修改。
4. 启动导航程序，
   
   如果采用 **TeamViewer 调试**，机器人端执行命令
   ```bash
   $ cd ~/navigation_ws
   $ source devel/setup.bash
   $ roslaunch nav robot.launch
   ```
   如果采用 **ROS 主从机调试**，机器人端执行命令
   ```bash
   $ cd ~/navigation_ws
   $ source devel/setup.bash
   $ roslaunch nav robot.launch
   ```
   调试端执行
   ```bash
   $ cd ~/navigation_ws
   $ source devel/setup.bash
   $ roslaunch nav pc.launch
   ```
## 版本说明

- v0.1@2020/12/23
  - 采用角速度控制
  - 最大线速度 0.5m/s
  - 融合 LIDAR 与 D435 避障
  - 导航点误差小于 0.1m

### TODO

- 检测到碰撞立即停止
  ```yaml
  <param name="controller_patience" value="0.0" />
  <param name="planner_frequency" value="0.0" />
  <param name="recovery_behavior_enabled" value="false" />
  <param name="clearing_rotation_allowed" value="false" />
  ```
- BT Design
- Elevation_mapping
- PCD convert to costmap
- Elevation_map convert to costmap
- TEB Bugs
- Use fixed global path
- Improve the accuracy of goal
- Improve the accuracy of localization using IMU
- Reduce pointcloud_to_laserscan's %CPU
- ADD `waitForTransform` before `lookupTransform`
- hdl_imu + STVL + sbpl + teb
- `tf1` to `tf2`
- fix IMU bug for hdl_localization
- Mini-Doc
- RSLiDAR 固件、SDK 更新和数据裁剪
- IMU 标定与初始化
- 注释掉 laserscan nodelet
- 修改雷达点云发布频率 20Hz
- 降低 `libndt_omp.so` CPU 占用率
- 时间同步设置 chrony
- waypoint_follower
- 相机-雷达标定

### ~~建图参考 (TODO 优化)~~

```bash
$ roslaunch ndt_mapping ndt_mapping.launch
$ rosbag record /globalmap/map_full
$ rosrun map_server map_saver map:=/projected_map
$ rosrun pcl_ros bag_to_pcd NAME_OF_BAG.bag /globalmap/map_full .
```

### ~~常用指令~~

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
$ sudo apt-get install ros-kinetic-plotjuggler python-catkin-tools vim htop
```

### UDP

- [Socket Programming](https://www.cs.dartmouth.edu/~campbell/cs60/socketprogramming.html)
- [sendto() + recvfrom() buffer confusion Datagram](https://stackoverflow.com/questions/30015205/sendto-recvfrom-buffer-confusion-datagram)
- [UDP sendto() and recvfrom() max buffer size](https://stackoverflow.com/questions/3292281/udp-sendto-and-recvfrom-max-buffer-size)

### [如何导出 ROS 二进制包](http://wiki.ros.org/catkin/CMakeLists.txt#Optional_Step:_Specifying_Installable_Targets)

文件类型：

* TARGETS **二进制文件**
* DIRECTORY **头文件夹/launch文件夹/参数文件夹**
* FILES **头文件/launch文件/参数文件**
* PROGRAMS **脚本文件**

安装路径：

* CATKIN_PACKAGE_BIN_DESTINATION
* CATKIN_PACKAGE_LIB_DESTINATION
* CATKIN_PACKAGE_INCLUDE_DESTINATION
  * CATKIN_GLOBAL_INCLUDE_DESTINATION
* CATKIN_PACKAGE_SHARE_DESTINATION
* CATKIN_PACKAGE_PYTHON_DESTINATION

下面通过修改 `CMakeLists.txt` 指定生成 deb 安装包需要的内容：
#### 如何安装可执行文件、动态库或静态库等二进制文件

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

#### 如何安装库的头文件

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

#### 安装 roslaunch 文件、YAML等普通文本文件

```cmake
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)
install(FILES nodelet_velodyne.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
```

#### 安装 Python 程序

```cmake
install(TARGETS python_module_library
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/myscript
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
```

#### 生成 deb 文件

```bash
$ sudo apt-get install python-bloom
$ cd path/to/your/catkin/package
$ bloom-generate rosdebian
$ fakeroot debian/rules binary
# $ bloom-generate rosdebian --os-name ubuntu --os-version xenial --ros-distro kinetic
# $ fakeroot debian/rules binary
```