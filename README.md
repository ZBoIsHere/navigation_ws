### Issues & TODO

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
- Using fixed global path
- Improve the accuracy of goal
- Improve the accuracy of localization

### WRC 小场景建图与定位@2020/12/02

```yaml
max_range: 7.0
ndt_resolution: 1.0
voxel_leaf_size: 0.5
```

### 地形图@2020/11/25

```bash
$ roslaunch nav sbpl.launch
$ roslaunch elevation_mapping_demos simple_demo.launch
$ roslaunch realsense2_camera rs_camera.launch
```

### 依赖项和编译工具

```bash
# 安装依赖包与调试工具
$ sudo apt-get install ros-kinetic-navigation ros-kinetic-navigation-experimental ros-kinetic-teb-local-planner
$ sudo apt-get install ros-kinetic-plotjuggler python-catkin-tools vim wireshark

$ catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_WHITELIST_PACKAGES=""
$ catkin config -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER_ARG1=-std=c++11
$ catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 新环境建图 (TODO 流程需要优化)

```bash
$ roslaunch ndt_mapping ndt_mapping.launch
$ rosbag record /globalmap/map_full
$ rosrun map_server map_saver map:=/projected_map
$ rosrun pcl_ros bag_to_pcd NAME_OF_BAG.bag /globalmap/map_full .
```

### 代码格式化

```bash
$ find . -iname '*.h' -o -iname '*.hh' -o -iname '*.hpp' -o -iname '*.c' -o -iname '*.cc' -o -iname '*.cpp' | xargs clang-format -i
$ black name-of-package
$ find . -name '*.DS_Store' -delete
```

### 固件修改

```bash
$ telnet 192.168.1.120
$ cd /home/bin
$ chmod 777 new_firmware
$ ln -sf new_firmware robot_control_system
$ shutdown
```

### 常用指令

```bash
$ rosrun rqt_tf_tree rqt_tf_tree
$ rosrun tf tf_echo from to
$ rqt_graph
$ rqt_logger_level
$ rqt_console
```