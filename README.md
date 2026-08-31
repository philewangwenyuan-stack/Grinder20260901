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

### 1. 编译

```bash
cd ~/work/Grinder/catkin_ws
catkin_make
source devel/setup.bash
```

### 2. 一键启动（推荐）

```bash
cd ~/work/Grinder/catkin_ws
./start_grinder_stack.sh
```

### 3. 单独启动调度

```bash
cd ~/work/Grinder/catkin_ws
source devel/setup.bash
roslaunch grinder_scheduler scheduler.launch aurora_ip_address:=<雷达IP>
```

### 4. 上位机启动（PyQt）

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
