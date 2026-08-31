# 2-dnavigation-package

### 介绍
局部路径规划及跟踪包(DWA/TEB)

### 软件架构
slamware_ros_sdk：思岚Aurora ROS1驱动;
2dnavigation: 基于move_base的局部路径规划（DWA/TEB）避障，差速模型

### 编译
#### 依赖
```bash
bash 3rdparty/g2omake.sh
```
#### slamware_ros_sdk
```bash
catkin_make -DCATKIN_WHITELIST_PACKAGES="slamware_ros_sdk"
```
#### 2dnavigation
```bash
catkin_make -DCATKIN_WHITELIST_PACKAGES="move_base" && catkin_make -DCATKIN_WHITELIST_PACKAGES="teb_local_planner" && catkin_make -DCATKIN_WHITELIST_PACKAGES="teb_local_planner_tutorials" && catkin_make -DCATKIN_WHITELIST_PACKAGES="dwa_local_planner" && catkin_make -DCATKIN_WHITELIST_PACKAGES="map_server" && catkin_make -DCATKIN_WHITELIST_PACKAGES="global_planner" && catkin_make -DCATKIN_WHITELIST_PACKAGES="base_global_planner"
```
· Note: 编译2dnavigation时，需要先编译依赖包，出现缺啥直接按照sudo apt install ros-noetic-xxx安装依赖

### 运行
```bash
roslaunch teb_local_planner_tutorials robot_diff_drive.launch
```

默认使用 TEB。使用 RPP（Regulated Pure Pursuit）跟踪器：

```bash
roslaunch teb_local_planner_tutorials robot_diff_drive_rpp.launch
```

使用 Stage 离线仿真 RPP：

```bash
roslaunch teb_local_planner_tutorials robot_diff_drive_rpp_in_stage.launch
```

从整机启动文件选择 RPP：

```bash
roslaunch grinder_scheduler grinder_system.launch navigation_local_planner:=rpp
```

RPP 会根据局部代价地图执行碰撞预测、减速和停车，但不会自行生成绕障路径。
需要绕行时，必须由全局规划器或调度器重新生成可通行路径。


