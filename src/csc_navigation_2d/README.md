# navigation

#### 开发环境

ubuntu16.04 

[ros Kinetic](http://wiki.ros.org/kinetic/Installation)

#### 需要具备的知识（重要！）
1. [tf坐标系转化关系](http://wiki.ros.org/navigation/Tutorials/RobotSetup/TF)
2. [eigen矩阵变换](http://eigen.tuxfamily.org/index.php?title=Main_Page)
3. [最短路径Dijkstra算法](https://blog.csdn.net/qq_35644234/article/details/60870719)
4. [地图服务器 map_server](http://wiki.ros.org/map_server)
5. [costmap_2d](https://blog.csdn.net/jinking01/article/details/79455962)
6. [actionlib](http://wiki.ros.org/actionlib)

#### 项目介绍
导航规划

1. 基于已有的定位、静态栅格占用图地图，指定的目标点，实现全局路径规划。
2. 基于已有的定位、静态栅格占用图地图、传感器获得的动态障碍物地图，全局路径，实现10Hz局部路径规划。

 **csc_nav2d_msgs** 定义了本功能包特有的数据格式

 **csc_nav2d_navigator** 包含了地图操作工具与整个全局路径规划、局部路径规划的内容。

 **csc_nav2d_operator** 将自定义的导航输出数据转化为ros通用的格式，并限制幅度。

 **csc_nav2d_working** 配置文件、launch文件、和底层通讯的node。


#### 使用说明



确保传感器已经开启

编译后执行
```
roslaunch csc_nav2d_working auto_start.launch 
```
会开启导航程序。

之后在rviz中按上方的2D Nav Goal ，在地图中点击目标的位置和方向。

#### 地图说明
每个地图由3个文件组成分别是用于定位的***.pcd和用于导航的配置文件***.yaml  静态地图***.pgm。

其中配置文件格式如下。想使用不同的静态地图时，只需要修改里面的名称（image）即可。

```
image: tengxun.pgm
resolution: 0.100000
origin: [-83.999997, -25.400000, 0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

静态地图由黑白像素组成，黑色是障碍物，白色是可通行区域。

可通过ps修改静态地图来实现虚拟障碍物的功能，也可以让机器人远离一些人为不想让它过去的地方。

注意，pgm格式比较特殊，ubuntu下使用gimp，windows下使用photoshop。

```
sudo apt-get install gimp
```

地图文件在 location.launch 中调用，修改好地图后需要同步修改launch文件中的yaml文件。
注意：地图修改不在导航的功能包里，在定位功能包里有打开。

```
 <node name="MapServer" pkg="map_server" type="map_server" output="screen" args="$(find location)/map/tengxun.yaml"/>
```

