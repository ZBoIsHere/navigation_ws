### 地形图@2020/11/25

```bash
$ roslaunch nav sbpl.launch
$ roslaunch elevation_mapping_demos simple_demo.launch
$ roslaunch realsense2_camera rs_camera.launch
```
### 依赖项和编译工具
```bash
# 安装依赖包与调试工具
$ sudo apt-get install ros-kinetic-navigation
$ sudo apt-get install ros-kinetic-navigation-experimental
$ sudo apt-get install ros-kinetic-teb-local-planner
$ sudo apt-get install python-catkin-tools
$ sudo apt-get install ros-kinetic-plotjuggler
$ sudo apt-get install vim
$ sudo apt-get install wireshark
# Release 编译
$ catkin build
$ catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release
$ catkin build --mem-limit 4G
$ catkin clean
$ catkin config -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER_ARG1=-std=c++11
```

### 新场地建图
```bash
$ roslaunch ndt_mapping ndt_mapping.launch
$ rosrun map_server map_saver map:=/projected_map
$ rosbag record /globalmap/map_full
$ rosrun pcl_ros bag_to_pcd 2020-XXX.bag /globalmap/map_full .
$ pcl_viewer 0.000000000.pcd
```

### 代码格式化
```bash
$ find . -iname '*.h' -o -iname '*.hh' -o -iname '*.hpp' -o -iname '*.c' -o -iname '*.cc' -o -iname '*.cpp' | xargs clang-format -i
$ black name-of-package
$ find . -name '*.DS_Store' -delete
```

### 固件修改
1. FTP (Transmit) 修改文件: 用户名 ntuser，密码 ntuser
2. telnet root登陆 192.168.1.120/60，`chmod 777` 添加可执行权限
   ```bash
   $ ln -sf robot_control_system_ip120 robot_control_system
   ```
3. `shutdown` 重启

### 常用指令
```bash
$ rosrun rqt_tf_tree rqt_tf_tree
$ rosrun tf tf_echo from to
$ rqt_graph
# 文件传输 Robot -> MacBook
$ scp -r ysc@192.168.1.102:~/DeepNavi/src .
# 文件传输 MacBook -> Robot
$ scp -r src/ ysc@192.168.1.102:~/DeepNavi/
```

### Issues & TODO
- 检测到碰撞立即停止
  ```yaml
  <param name="controller_patience" value="0.0" />
  <param name="planner_frequency" value="0.0" />

  <param name="recovery_behavior_enabled" value="false" />
  <param name="clearing_rotation_allowed" value="false" />
  ```
- BT Design
- Elevation_mapping/Octomap/STVL using LIDAR or D435
- PCD convert to costmap
- Elevation_map convert to costmap