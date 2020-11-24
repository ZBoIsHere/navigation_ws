### 依赖项和编译工具
```bash
# 依赖包
$ sudo apt-get install ros-kinetic-navigation
$ sudo apt-get install ros-kinetic-teb-local-planner
$ sudo apt-get install python-catkin-tools
$ sudo apt-get install ros-kinetic-plotjuggler
$ sudo pip install pathlib
# 编译 Release
$ catkin build
$ catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release
$ catkin build --mem-limit 4G
$ catkin clean
$ catkin config -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER_ARG1=-std=c++11
```

### 配置参数
- Footprint: MINI、JYL
- 地图替换
- 速度/加速度限制
- 膨胀系数
- IP地址: 192.168.1.XXX
- 雷达替换: 话题 & 坐标名称
- 固件替换
- 速度偏置: /home/deeprcs.json
- Wireshark   网络通信分析
- PlotJuggler 实时数据曲线

### 新场地建图 (参数调节: `resolution`: 0.1,`ndt_resolution`: 2.0,`ndt_step_size`: 0.2)
```bash
$ roslaunch ndt_mapping ndt_mapping.launch
# 保存导航 2D 地图，修改 `map.yaml` 的 origin: [XXX, XXX, 0.000000] 和 image: XXX.pgm
$ rosrun map_server map_saver map:=/projected_map
# 保存定位 3D 地图，修改文件名与 2D 地图一致
$ rosbag record /globalmap/map_full
$ rosrun pcl_ros bag_to_pcd 2020-XXX.bag /globalmap/map_full .
# 3D 地图可视化
$ pcl_viewer 0.000000000.pcd
```

### 代码格式化
```bash
# C++:
$ find . -iname '*.h' -o -iname '*.hh' -o -iname '*.hpp' -o -iname '*.c' -o -iname '*.cc' -o -iname '*.cpp' | xargs clang-format -i
# Python:
$ sudo black auto_nav2d_pipeline
# 删除 .DS_Store 文件
$ find . -name '*.DS_Store' -delete
```

### 固件修改
1. FTP (Transmit) 修改文件: 用户名 ntuser，密码 ntuser
2. telnet root登陆 192.168.1.120/60，`chmod 777` 添加可执行权限
   ```
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
- message_transformer `timeout!`
  ```txt
  Inbound TCP/IP connection failed: connection from sender terminated before handshake header received. 0 bytes were received. Please check sender for additional details.
  ```
- BT Design
- Elevation_mapping/Octomap/STVL using LIDAR or D435
- How to disable local costmap?
- PCD convert to costmap
- Elevation_map convert to costmap