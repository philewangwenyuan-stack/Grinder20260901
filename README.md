# Grinder 研磨机项目说明

本文档用于说明研磨机项目的工程结构、核心链路、启动方式与联调要点。

## 项目目标

本项目用于实现研磨机的：

- 地图管理（建图、保存、删除、切图）
- 区域编辑（工作区/障碍区/擦除区/裁剪区）
- 路径规划与任务执行（含区域重复执行）
- 底盘与磨盘控制（手动/自动）
- 上位机与安卓通过 SL-LinkA 协议通信

## 目录结构

```text
Grinder/
├── catkin_ws/
│   ├── src/
│   │   ├── grinder_scheduler/         # 调度核心：协议处理、任务状态机、地图与路径流程
│   │   ├── grinder_chassis_driver/    # 底盘驱动：/cmd_vel->轮速、Modbus485读写、状态上报
│   │   ├── 2-dnavigation-package/     # move_base / teb / 全局路径相关
│   │   ├── slamware_ros_sdk/          # 雷达SDK：map/odom/图像与模式控制
│   │   └── ...
│   ├── start_grinder_stack.sh         # 一键启动脚本
│   └── build_grinder_platform.sh      # 构建脚本
├── third_party/
│   ├── sl_linka/                      # 协议、SDK、PyQt上位机
│   └── path_planner/                  # 自适应路径规划器与可视化
├── tools/
│   └── mediamtx/                      # 本地RTSP服务工具
└── README.md
```

## 核心链路

### 1. 协议链路（APP/上位机 <-> 调度）

- 调度监听 `8002`（SL-LinkA）
- 地图类：`MapPreview/MapEdit/MapCatalog/MapSave/MapDelete/MapMetrics`
- 任务类：`TaskConfig/TaskCommand/TaskStatus/TaskResult`
- 新增：`LiveMapCacheClearRequest/Response (0x051E/0x051F)`

协议说明见：
[sl-link.md](/home/chersxir/work/Grinder/third_party/sl_linka/sl-link.md)

### 2. 导航与任务链路

- 调度生成/维护全局路径，执行时按段发布目标点
- 导航输出 `cmd_vel`，底盘驱动按任务使能决定是否接收
- `/chassis/task_enable`：
  - `true`：底盘处理 `/cmd_vel`
  - `false`：底盘忽略 `/cmd_vel`

### 3. 底盘控制链路

- 底盘驱动订阅 `/cmd_vel`（默认）
- 转换为左右轮速度，做限幅/方向处理后通过 Modbus 写入
- 配置文件：
[chassis_driver.yaml](/home/chersxir/work/Grinder/catkin_ws/src/grinder_chassis_driver/config/chassis_driver.yaml)

## 关键配置文件

- 调度配置：
[scheduler.yaml](/home/chersxir/work/Grinder/catkin_ws/src/grinder_scheduler/config/scheduler.yaml)
- 底盘配置：
[chassis_driver.yaml](/home/chersxir/work/Grinder/catkin_ws/src/grinder_chassis_driver/config/chassis_driver.yaml)
- 协议定义：
[sl_link.proto](/home/chersxir/work/Grinder/third_party/sl_linka/proto/sl_link.proto)

常用调度参数（`scheduler.yaml`）：

- `path_goal_reach_dist`：到点判定距离
- `path_segment_timeout`：当前段超时重发时间
- `draw_region_label_on_preview`：区域名称是否绘制
- `draw_region_id_on_preview`：是否拼接显示区域ID
- `live_map_cache_clear_mode`：`all` / `memory_only`

## 编译与启动

MID-360S + Super-LIO 的固定内存实时二维建图见
`catkin_ws/src/2-dnavigation-package/cloud_to_occupancy_grid/README.md`。该流程直接使用 LIO 位姿和
当前配准点云生成 `/map`，不再叠加 GMapping。

### 1. 编译

```bash
cd ~/work/Grinder/catkin_ws
catkin_make
source devel/setup.bash
```

### 2. 每个终端的基础环境

板端每开一个新终端都先执行：

```bash
source /opt/ros/noetic/setup.bash
source /home/neardi/work/Grinder/catkin_ws/devel/setup.bash
```

以下命令假定工作区位于 `/home/neardi/work/Grinder/catkin_ws`。建图与重定位
是互斥模式，不要同时运行 `super_lio_node` 和 `relocation_node`。

### 3. 启动 MID-360S 驱动

```bash
roslaunch livox_ros_driver2 msg_MID360s.launch
```

验证：

```bash
rostopic hz /livox/lidar
rostopic hz /livox/imu
```

### 4. 建图模式

启动 Super-LIO、回环后端和固定尺寸二维栅格地图：

```bash
roslaunch cloud_to_occupancy_grid mid360_mapping.launch \
  rviz:=true \
  save_3d_map:=true \
  loop_closure:=true \
  loop_backend:=industrial
```
```bash
roslaunch cloud_to_occupancy_grid mid360_mapping.launch \
  rviz:=true \
  save_3d_map:=true \
  loop_closure:=false
```

Super-LIO 只在位移、旋转、时间间隔和点数满足关键帧准入条件时向三维地图
追加点云；设备静止时不会逐帧重复累计。关键帧参数位于
`Super-LIO/src/super_lio/config/livox_360.yaml`。

验证建图输出：

```bash
rostopic hz /lio/odom
rostopic hz /lio/map_cloud
rostopic hz /map
rosrun tf tf_echo map base_link
```

保存三维定位地图：

```bash
mkdir -p /home/neardi/maps
rosservice call /lio/get_3dmap \
  "cmd_id: 1
enable_filter: true
map_dir: '/home/neardi/work/Grinder/maps'
map_name: 'grinder_map'
```

正常生成：

```text
/home/neardi/maps/loc_grinder_map.pcd
/home/neardi/maps/plan_grinder_map.pcd
```

保存二维导航地图：

```bash
rosservice call /cloud_to_occupancy_grid/save_map
```

二维地图的实际输出目录和文件名由
`cloud_to_occupancy_grid/config/mid360.yaml` 配置。确认 PCD、PGM 和 YAML 均已
成功生成后再停止建图节点：

```bash
rosnode kill /super_lio_node
rosnode kill /super_lio_loop
rosnode kill /cloud_to_occupancy_grid
```

Livox 驱动可以继续运行，供重定位使用。

### 5. 旧地图重定位模式

先启动二维静态地图。路径必须替换为本次保存的 YAML：

```bash
rosrun map_server map_server /home/neardi/work/Grinder/maps/grinder_map.yaml
```

再启动固定三维地图重定位：

```bash
roslaunch super_lio relocation.launch \
  rviz:=true \
  map_dir:=/home/neardi/work/Grinder/maps \
  map_name:=loc_grinder_map.pcd \
  base_to_laser_x:=0.0 \
  base_to_laser_y:=0.0 \
  base_to_laser_z:=0.0 \
  base_to_laser_roll:=0.0 \
  base_to_laser_pitch:=0.0 \
  base_to_laser_yaw:=0.0
```

`base_to_laser_*` 必须按实际安装外参填写。重定位模式由 launch 强制设置
`/lio/map/save_map=false`，不累计保存点云；固定地图定位还应保持
`/lio/relocation/update_map=false`。

首次启动后在 RViz 使用 `2D Pose Estimate` 发布 `/initialpose`。验证：

```bash
rostopic hz /lio/odom
rostopic hz /scan
rostopic echo -n 1 /map/info
rosrun tf tf_echo map base_link
```

### 6. 启动 move_base

旧地图和重定位均正常后启动导航。RPP 使用底盘反馈 `/odom_wheel`：

```bash
roslaunch teb_local_planner_tutorials robot_diff_drive.launch \
  local_planner:=rpp
```

如需 TEB：

```bash
roslaunch teb_local_planner_tutorials robot_diff_drive.launch \
  local_planner:=teb
```

验证代价地图和速度输出：

```bash
rostopic hz /move_base/global_costmap/costmap
rostopic hz /move_base/local_costmap/costmap
rostopic hz /cmd_vel
```

### 7. 启动底盘驱动

启动前检查 `grinder_chassis_driver/config/chassis_driver.yaml` 中的串口、波特率、
轮距、轮径和方向参数。真实底盘启动会访问 Modbus 硬件：

```bash
ls -l /dev/ttyS0
roslaunch grinder_chassis_driver chassis_driver.launch
```

默认 `task_enable_default=false`，节点启动时先下发零速。验证：

```bash
rosnode info /chassis_driver
rostopic hz /odom_wheel
rostopic echo /chassis/task_enable
```

### 8. 启动调度器

使用 Super-LIO 时，显式把调度定位输入设为 `/lio/odom`。move_base 已经单独
启动时，只启动 `scheduler.launch`，不要再启动一个包含导航的系统 launch：

```bash
roslaunch grinder_scheduler scheduler.launch \
  odom_topic:=/lio/odom \
  map_topic:=/map \
  navigation_map_yaml_path:=/home/neardi/maps/grinder_map.yaml
```

验证：

```bash
rosnode info /grinder_scheduler
rostopic info /grinder/navigation/active_segment_plan
rostopic echo /chassis/task_enable
ss -lntp | grep 8002
```

完整导航链路为：

```text
grinder_scheduler
  -> /grinder/navigation/active_segment_plan
  -> move_base
  -> /cmd_vel
  -> grinder_chassis_driver
  -> 底盘
```

### 9. 一键启动（Aurora/既有系统模式）

```bash
cd ~/work/Grinder/catkin_ws
./start_grinder_stack.sh
```

一键脚本主要面向既有 Aurora 系统链路。已经手动启动 Super-LIO、map_server、
move_base 时不要再运行默认一键脚本，避免 `/map`、TF 和 `/move_base` 重复发布。

### 一键启动super-lio模式
cd /home/neardi/work/Grinder/catkin_ws
chmod +x start_grinder_super_lio_base.sh
./start_grinder_super_lio_base.sh

### 10. 单独启动调度（Aurora 模式）

```bash
cd ~/work/Grinder/catkin_ws
source devel/setup.bash
roslaunch grinder_scheduler scheduler.launch aurora_ip_address:=<雷达IP>
```

### 11. 上位机启动（PyQt）

```bash
cd ~/work/Grinder
python3 third_party/sl_linka/sdk/python/tools/sl_linka_pyqt_debugger.py
```

连接参数：

- Host：调度程序IP（本机可 `127.0.0.1`）
- Port：`8002`

## 协议库生成

当 `sl_link.proto` 变更后，需重新生成 SDK：

```bash
cd ~/work/Grinder
bash third_party/sl_linka/scripts/generate_proto.sh all
```

若仅需 Python/Android：

```bash
bash third_party/sl_linka/scripts/generate_proto.sh python
bash third_party/sl_linka/scripts/generate_proto.sh kotlin
```

## 常用联调命令

查看调度订阅发布：

```bash
rosnode info /grinder_scheduler
```

查看导航目标发布：

```bash
rostopic echo /move_base_simple/goal
```

查看底盘任务使能：

```bash
rostopic echo /chassis/task_enable
```

查看底盘速度输入：

```bash
rostopic hz /cmd_vel
```

## 说明

- 地图、任务、区域均建议基于 `map_id` 进行绑定与操作。
- 清缓存指令 `0x051E` 为无参请求，具体清理范围由调度配置决定。
- 本项目是在线联调工程，修改后请重启对应节点验证参数是否已生效。
