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
│   │   ├── localization/             # Super-LIO 三维里程计、定位及回环
│   │   ├── mapping/                  # 三维点云到二维占据栅格
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

### 4. MID-360 / Super-LIO 当前完整链路

本节描述当前源码的标准模式：`lio/ros/legacy_tf: false`，导航参考点为
`base_link`，对应底盘运动中心。MID-360 的 IMU 内置在雷达设备中；两者具有
固定的内部外参，整台设备相对底盘的安装外参由实际标定提供。

```text
MID-360：三维点云 + 内置 IMU
  -> Livox 驱动：/livox/lidar、/livox/imu
  -> Super-LIO：时间同步、静止重力初始化、IMU 预测、点云去畸变、点云匹配
       ├─ 内部：估计 IMU 的三维位姿和速度，用于点云及地图计算
       └─ 输出：利用安装外参换算运动中心的位姿、速度和协方差
                  -> /lio/odom、/lio/path、odom -> base_link

建图模式：三维世界点云 -> 可选回环 -> 二维栅格 /map -> 保存地图资产包
定位模式：加载三维 PCD + 二维地图 -> 底盘初始位姿 -> 三维配准 -> 定位质量检查

APP / 上位机 -> SL-LinkA -> grinder_scheduler
  -> 地图模式管理、区域编辑、作业路径规划、任务状态机
  -> move_base 目标点 / 路径段
  -> RPP（常驻启动默认）或 TEB -> /cmd_vel
  -> grinder_chassis_driver：检查 /chassis/task_enable、速度限幅、左右轮换算
  -> Modbus RTU / RS485 -> 底盘轮速与磨盘控制
```

`/lio/odom` 是连续的底盘里程计；`/lio/map_cloud` 是世界坐标中的三维点云。
两者的参考不同：点云仍由 IMU 状态正确变换，输出给导航的位姿则已换算为底盘中心，
不能再给 `/lio/odom` 额外减一次雷达偏移。历史调试话题 `/lio/robo/odom`
仍是单独的 IMU 频率接口，正式导航使用 `/lio/odom`。

可选内部轮速融合在建图、定位模式下均默认关闭。底盘从同一次实际转速读取
额外发布有效观测 `/chassis/wheel_twist`，RPP 的 `/odom_wheel` 保持原用途。
`grinder_scheduler/config/super_lio.yaml` 分别配置两模式总开关及线速度、角速度
观测开关；支持倒车，缺少轮速时继续纯 LIO，不改变定位异常暂停条件或 TF 发布权。
现场标定前不要开启；参数、回放和诊断说明见
[`Super-LIO/UPSTREAM.md`](catkin_ws/src/localization/Super-LIO/UPSTREAM.md#optional-internal-wheel-fusion)。

### 5. 建图、保存和定位的两条分支

**建图分支**由 `super_lio_mode_manager` 启动 `super_lio_node` 和
`cloud_to_occupancy_grid`，按配置决定是否启动 `super_lio_loop`：

- 关闭回环时，栅格节点接收 `/lio/map_cloud`，`map -> odom` 是单位变换。
- 开启回环时，后端接收 `/lio/odom` 和 `/lio/map_cloud`，发布校正点云
  `/lio/loop_cloud`、校正里程计 `/lio/loop_odom`，并负责 `map -> odom`。
  栅格节点改为接收 `/lio/loop_cloud`；工业后端接受回环后，支持历史关键帧重投影。
- 回环后端将世界点云变换到与关键帧位姿一致的局部坐标，重投影时再变回世界坐标。
  工业后端使用完整三维位姿；轻量后端两次变换都使用其保存的二维位姿，保留点云的
  世界高度和倾角。这里不再次应用雷达安装外参。

当前 `super_lio.yaml` 的 `super_lio_loop_closure` 默认是 `false`；手工建图示例
显式指定 `loop_closure:=true loop_backend:=industrial`，两者要区分。

保存时，模式管理器分别调用 `/lio/get_3dmap` 和
`/cloud_to_occupancy_grid/save_map`，校验文件并写入资产版本，形成独立目录：

```text
maps/<map_id>/
  loc_map.pcd       三维定位地图
  plan_map.pcd      三维规划点云
  map.yaml         二维地图元信息
  map.pgm          二维占据栅格
  map_info.json    地图身份、版本及文件清单
```

保存成功后停止建图，不自动开始定位。当前三维 PCD 的保存入口仍是 Super-LIO 的
`/lio/get_3dmap`；开启回环时，不能仅凭二维栅格已经校正就认为该入口的三维 PCD
也已自动完成同样的校正，应核对保存后二维与三维资产的坐标一致性。

**定位分支**加载同一资产包：`map_server` 从 `map.yaml` 发布 `/map`，
`relocation_node` 从 `loc_map.pcd` 加载三维环境。底盘初始位姿与 IMU 重力方向
共同形成配准初值，经过 NDT、ICP 和后续 LIO 更新，输出底盘位姿。
当前固定地图定位 launch 发布单位 `map -> odom`，定位结果直接位于加载地图的坐标中。

模式管理器检查传感器和地图新鲜度、TF、配准质量、位姿协方差及连续稳定帧。
运行中同一定位帧只计一次；普通质量阈值连续 5 帧失败才撤销就绪，合格帧清零失败计数。
超过 0.75 m 或约 30° 的位姿突跳立即撤销就绪；传感器、配准诊断和 TF 断流按新鲜度立即处理。
管理器以 10 Hz 发布 `/super_lio/localization_status`，调度超过 0.5 秒收不到心跳即暂停任务。
定位恢复需连续 5 帧合格，任务保持暂停，必须由操作者手动继续。
APP 提交的初始位姿必须匹配当前 `map_id/map_revision`；达到 `READY`
后才允许开始任务。建图和定位互斥，不同时运行两个 LIO 主节点。

### 6. IMU、雷达、运动中心与 TF 的关系

标准 TF 链为：

```text
map
  └─ odom                   回环后端发布校正；无回环建图 / 固定地图定位发布单位变换
       └─ base_link         Super-LIO 发布运动中心的动态位姿
            └─ base_laser_link   根据已有标定发布固定安装变换
```

每条 TF 边只由一个发布者负责。IMU 坐标是 LIO 内部计算参考，并不要求在上述链中
另加一个公开的 `imu_link`，也不需要额外启动第二个雷达固定 TF 发布者。

定义 `T_A_B` 为将 B 坐标转换到 A 坐标的变换。已有标定给出 `T_base_laser`，
`lio/extrinsic/lidar_imu` 给出设备内部的 `T_imu_laser`，输出层计算：

```text
T_odom_base = T_odom_imu × T_imu_laser × inverse(T_base_laser)
```

这一步同时处理位置和安装方向。例如 MID-360 位于运动中心前方约 10 cm，
换算后的 `base_link` 就代表真实运动中心；俯仰和航向变化时，偏移随当前姿态旋转。
转弯时还补偿安装偏移产生的线速度差，并将线速度和角速度表达在 `base_link` 中，
同步传播位姿和速度协方差。

move_base 的全局、局部 costmap 都使用 `robot_base_frame: base_link`。
它读取上述 TF 链确定运动中心位置，并用固定安装外参转换雷达观测；
运动中心换算由 Super-LIO 输出层完成，move_base 不重新修改该动态 TF。
`base_link` 保留车体三维姿态，不因为重力对齐而强制变成一个水平车身坐标系。

### 7. 安装外参从哪里来、何时生效

托管模式沿用调度器保存的安装标定，参数对应关系为：

| 调度标定参数 | 模式启动参数及 LIO 参数末尾名称 | 单位 |
| --- | --- | --- |
| `base_laser_x/y/z` | `base_to_laser_x/y/z` | 米 |
| `base_laser_roll_deg` | `base_to_laser_roll` | 调度为度，启动 / LIO 为弧度 |
| `base_laser_pitch_deg` | `base_to_laser_pitch` | 调度为度，启动 / LIO 为弧度 |
| `base_laser_yaw_deg` | `base_to_laser_yaw` | 调度为度，启动 / LIO 为弧度 |

参数传递链为：

```text
调度器已有标定 / 设备配置
  -> 调度器将角度转换为弧度，写入模式管理器参数
  -> 模式管理器在建图或定位启动时传入同一组 base_to_laser_* 参数
  -> launch 同时配置：
       ① base_link -> base_laser_link 固定 TF
       ② /lio/extrinsic/base_to_laser_*，供 LIO 输出及初始定位换算
```

因此，修改已有标定后应通过正常流程停止并重新启动地图模式，确保固定 TF 和
LIO 同时使用新参数。更换代码后还需先在 Linux / ROS Noetic 环境重新构建。
不要仅改其中一处，也不要把“约 10 cm”硬编码进算法。

手工 roslaunch 不会自动读取调度器已保存的标定；建图和定位命令都必须显式传入
实际 `base_to_laser_*`。本 README 命令中的 `0.0` 为示例占位值，不代表设备实测值。
全零底盘—雷达标定表示两者坐标重合，不能消除 MID-360 内部雷达与 IMU 的固定差异。
旧 `legacy_tf: true` 模式仍保持历史里程计 / TF 行为，并提示未启用本节的底盘补偿。

### 8. 30° 行走模式建图与 0° 工作模式定位

车身整体倾斜会同时改变 MID-360 点云和内置 IMU 的姿态。
建图、定位初始化代码都用 IMU 平均加速度估计重力方向，再计算对齐旋转；
建图不会直接将车身起始的 30° 倾角当作整张地图的倾角。
定位则按当前 0° 姿态重新估计重力方向，再与三维地图配准。

这部分处理写在 `super_lio.cpp` 和 `relocation.cpp` 的初始化代码中。
虽然配置中存在 `kf_align_gravity`，当前实现读取该参数，但上述初始化对齐代码
没有用它作为开关判断。因此无需把 30° 写进配置来开启补偿，也不能依赖将该参数
设为 `false` 来关闭当前初始化对齐。

在设备与车身刚性关系不变、实际运动中心定义不变的前提下，30° 建图与 0° 定位
可以共用环境地图及安装外参。拆装导向轮后应先放稳车辆，再开始初始化；
高度和视野变化会影响配准初值与可见点云重叠，需要通过回放和实车定位验收确认效果。

`/initialpose` 在标准模式中表示底盘中心在地图中的位置及航向。
初始化时结合重力和安装外参换算为 IMU 配准初值，已移除写死的传感器高度 `0.2`。
输入 Z 是地图坐标中的底盘高度，不一定等于离地高度；当前 APP 的二维重定位请求
提供 X/Y/航向，调度器发布时 Z 为 `0.0`，RViz 的二维初始位姿也应按地图参考理解。
有明确高度初值时可通过模式管理器的带三维 Pose 服务提交；最终位姿仍由三维配准优化。

### 9. 任务执行与安全链路

调度器根据当前地图、作业区域、机器人尺寸和任务参数调用路径规划适配层，
管理路径段、作业轮次、目标点及任务进度。move_base 使用 `/map`、里程计、TF
和雷达观测生成 `/cmd_vel`，底盘驱动再将线速度 / 角速度换算为左右轮指令。
任务使能与速度指令是两条独立输入，收到速度并不等于允许底盘执行。

暂停、停止或异常处理继续关闭 `/chassis/task_enable`，清零轮速并执行磨盘安全控制；
串口超时与急停保护仍由现有安全链路处理。此次位姿修复不改变这些控制条件。
磨盘中心若不在底盘运动中心，作业工具的固定偏移仍需单独描述；雷达安装标定
只负责雷达与底盘的关系，不自动定义磨盘中心。

本次坐标修复已通过独立 C++ 数学回归、14 项调度 / 启动链测试及轻量回环节点
Catkin 编译。完整 Super-LIO 构建在本地 WSL 因缺少 `livox_ros_driver2` 被阻断；
完整构建、录包回放、旧地图一致性与 30°/0° 实车定位仍需在目标环境验收。
实现和验证入口见 [Super-LIO 集成说明](catkin_ws/src/localization/Super-LIO/UPSTREAM.md#base-pose-and-mounting-calibration)。

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

Super-LIO 模式、地图根目录和启停超时位于
`catkin_ws/src/grinder_scheduler/config/super_lio.yaml`；该文件不包含设备凭据，可随代码部署。

### 默认配置与设备本地覆盖

`scheduler.yaml` 是受 Git 管理的公共默认配置，不含设备凭据；平台上传、
MQTT 和远端视频推流默认关闭。设备端复制
`catkin_ws/src/grinder_scheduler/config/scheduler.local.example.yaml` 为同目录的
`scheduler.local.yaml`，只填写设备差异、服务地址和凭据，并显式打开所需服务。
`scheduler.local.yaml` 被 Git 忽略，Catkin 安装也不携带该文件。

`start_grinder_stack.sh` 和 `start_grinder_super_lio_base.sh` 自动发现该本地文件，
启动前检查 YAML。可通过 `GRINDER_SCHEDULER_LOCAL_CONFIG=/absolute/path/device.yaml`
选择其他覆盖文件；显式设置为空字符串可使用纯默认配置。
`start_grinder_sim.sh` 不自动发现设备文件，仿真配置强制关闭平台上传、MQTT 和远端推流。

直接使用 roslaunch 时，按需传入覆盖文件：

```bash
roslaunch grinder_scheduler scheduler.launch \
  scheduler_local_config:=/absolute/path/scheduler.local.yaml
```

不传覆盖参数且没有 `GRINDER_SCHEDULER_LOCAL_CONFIG` 环境变量时，只加载公共配置。
顺序为公共配置、Super-LIO 配置（真实启动）、本地覆盖、显式 launch 参数；
仿真在本地覆盖之后加载仿真配置。显式传入的文件缺失或 YAML 无效会阻止启动。
密码也可由 `GRINDER_PLATFORM_PASSWORD`、`MQTT_USERNAME`/`MQTT_PASSWORD` 等环境变量提供；
只有本地配置显式启用相应服务后才会连接。

### Super-LIO 源码恢复

`catkin_ws/src/localization/Super-LIO` 已按普通文件纳入根仓库，保留了本地集成改动。
新克隆无需初始化子模块；上游地址、基准版本和导入说明见该目录的 `UPSTREAM.md`，
原有 GPL-3.0 许可证和第三方声明随源码保留。

## ROS Noetic 安装、工业级编译与正式启动

以下流程面向 Ubuntu 20.04（focal）+ ROS Noetic，支持 x86_64 和 RK3588/aarch64，
传感器为 Livox MID-360S。工业回环使用 GTSAM 4.2 iSAM2，并优先使用 fast_gicp
（FastVGICP）；二维地图由 cloud_to_occupancy_grid 增量维护，不使用 GMapping。

新板部署按本节安装、编译，再配置开机自启。2026-10-22 的实际版本、热点与雷达网口配置、
systemd 设备覆盖、验收结果和当天踩坑见 [RK3588 部署实录](#deployment-20261022)。
其中 STVL 与 jemalloc 是导航运行依赖，不能只以 Catkin 编译通过作为部署完成的依据。

工作区路径示例：

~~~bash
export GRINDER_WS=/home/neardi/work/Grinder/catkin_ws
cd "$GRINDER_WS"
~~~

建图和定位互斥：

~~~text
建图：super_lio_node + super_lio_loop + cloud_to_occupancy_grid
定位：map_server + relocation_node
~~~

不要同时运行 super_lio_node 和 relocation_node。

### 1. 安装 ROS Noetic

ROS Noetic 的目标系统是 Ubuntu 20.04。在板端执行：

~~~bash
sudo apt update
sudo apt install -y curl gnupg2 lsb-release ca-certificates

curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  | gpg --dearmor \
  | sudo tee /usr/share/keyrings/ros-archive-keyring.gpg >/dev/null

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros/ubuntu focal main" \
  | sudo tee /etc/apt/sources.list.d/ros1.list >/dev/null

sudo apt update
sudo apt install -y ros-noetic-ros-base
~~~

板端无桌面运行使用 `ros-base`。需要 RViz 与本仓库 Super-LIO 可视化配置使用的 JSK 插件时，
可单独补装以下包，无需安装完整桌面套件：

~~~bash
sudo apt install -y --no-install-recommends \
  ros-noetic-rviz ros-noetic-jsk-rviz-plugins ros-noetic-jsk-visualization
source /opt/ros/noetic/setup.bash
command -v rviz
rospack find jsk_rviz_plugins
~~~

`ros-base` 不包含 `rviz`，只 source ROS 环境不会安装缺失命令。
仓库的 `lio.rviz`、`relocation.rviz` 等使用 `jsk_rviz_plugin/OverlayText`、`Plotter2D`
等插件，单装 RViz 仍会缺少这些显示类型。启动 RViz 需要可用的图形会话；
纯 SSH 没有 DISPLAY 时应在板端桌面的终端启动，或使用已配置好的远程图形会话。
如果确实需要完整 ROS 桌面工具集，再选择 `ros-noetic-desktop-full`。

**RK3588 + MobaXterm 的 X11/OpenGL 兼容配置（2026-10-22 实测）**：
若 RViz 在 Mali 初始化后报 `Unsupported extension used`、`X connection to localhost:10.0 broken`，
先核对 `echo "$DISPLAY"`，并保留 MobaXterm 自动设置的转发显示地址，不要手工改成 `:0`。
当天板端本地 X Server 没有 GLX，直接改到 `:0` 的 RViz 测试在 `glXCreateContext` 崩溃；
MobaXterm 转发显示有 GLX，选择 Mesa/llvmpipe 后已能创建 OpenGL 上下文并运行 RViz 界面。
还需覆盖登录会话中的 `QT_XCB_GL_INTEGRATION=xcb_egl`：该设置会让 Qt 继续加载 Mali EGL，
即使已经设置 Mesa 变量仍会报同样的 X11 错误。RViz 转发启动时显式指定 `xcb_glx`。
在 MobaXterm 的板端 SSH 终端执行：

~~~bash
source /opt/ros/noetic/setup.bash
unset LIBGL_ALWAYS_INDIRECT
QT_XCB_GL_INTEGRATION=xcb_glx QT_X11_NO_MITSHM=1 LIBGL_ALWAYS_SOFTWARE=1 \
  __GLX_VENDOR_LIBRARY_NAME=mesa GALLIUM_DRIVER=llvmpipe LP_NUM_THREADS=2 rviz
~~~

这些设置只作用于该次 RViz。llvmpipe 使用 CPU 渲染，密集点云显示会增加 CPU 开销，
可降低显示频率/点数；无需把这些变量写入导航服务或替换系统 Mali 库。
变量含义见 [Mesa 环境变量说明](https://docs.mesa3d.org/envvars.html)。
当天另在板端 `/usr/local/bin/rviz-x11` 保存了同样设置并加载 Grinder 工作区，
之后可直接运行 `rviz-x11` 或 `rviz-x11 -d /absolute/path/config.rviz`；该设备脚本不随 clone 自动安装。
诊断可安装 `mesa-utils` 并在相同环境下运行 `glxinfo -B`，当天 renderer 为 Mesa llvmpipe。
如果官方源不可访问，将 /etc/apt/sources.list.d/ros1.list 中的
packages.ros.org/ros/ubuntu 替换为可用镜像后再执行 sudo apt update。

安装 rosdep、Catkin 和基础工具：

~~~bash
sudo apt install -y \
  python3-rosdep python3-rosinstall python3-rosinstall-generator \
  python3-wstool python3-catkin-tools build-essential cmake git pkg-config

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update
~~~

每个新终端先加载：

~~~bash
source /opt/ros/noetic/setup.bash
source "$GRINDER_WS/devel/setup.bash" 2>/dev/null || true
~~~

工作区第一次成功编译后，再把这两行加入 ~/.bashrc。

### 2. 安装工作区依赖

~~~bash
cd "$GRINDER_WS"
source /opt/ros/noetic/setup.bash

rosdep install \
  --from-paths src \
  --ignore-src \
  --rosdistro noetic \
  --skip-keys="gtsam fast_gicp" \
  -r -y

sudo apt install -y \
  libboost-all-dev libeigen3-dev libmetis-dev libpcl-dev \
  libgoogle-glog-dev libgflags-dev libyaml-cpp-dev libceres-dev libusb-1.0-0-dev \
  python3-pip python3-yaml python3-serial python3-protobuf python3-paho-mqtt ffmpeg \
  ros-noetic-navigation ros-noetic-pcl-ros ros-noetic-cv-bridge \
  ros-noetic-tf2-geometry-msgs ros-noetic-laser-geometry ros-noetic-robot-state-publisher \
  ros-noetic-xacro ros-noetic-tf-conversions ros-noetic-eigen-conversions \
  ros-noetic-tf2-sensor-msgs ros-noetic-mbf-costmap-core ros-noetic-mbf-msgs \
  ros-noetic-spatio-temporal-voxel-layer libjemalloc2

# 当前协议生成代码使用 protobuf 4.25.1；Ubuntu 20.04 自带的 3.6.1 不够。
sudo python3 -m pip install 'protobuf==4.25.1'
python3 -c 'import google.protobuf; from google.protobuf.internal import builder; print(google.protobuf.__version__)'
~~~

GTSAM 和 fast_gicp 是外部依赖，按下一节安装；不要用 ROS 2 包解决 ROS 1 依赖。

### 2.1 MST27 路径规划依赖

调度器默认使用：

~~~yaml
planner_script_path: third_party/path_planner/mst27/mst27.py
~~~

MST27 的 Python 规划器需要 NumPy、SciPy、Matplotlib 和 OpenCV；生产环境还应编译
同目录下的 C++/pybind11 加速内核。安装依赖：

~~~bash
sudo apt install -y \
  python3-numpy \
  python3-scipy \
  python3-matplotlib \
  python3-opencv \
  python3-dev \
  pybind11-dev
~~~

设置仓库根目录，确保调度器能解析相对的 planner_script_path：

~~~bash
export GRINDER_BASE_DIR=/home/neardi/work/Grinder
~~~

编译 MST27 C++ 内核（Release）：

MST27 扩展当前使用独立 CMake 工程；不要假设仅执行 catkin_make 就一定会生成该
Python 扩展，正式部署应显式执行下面的 CMake 命令。

~~~bash
cd "$GRINDER_BASE_DIR/third_party/path_planner/mst27"
cmake \
  -S . \
  -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DPython3_EXECUTABLE="$(which python3)"
cmake --build build --parallel 2
~~~

扩展模块应生成在 mst27.py 同目录。验证 Python 依赖和 C++ 内核：

~~~bash
python3 -c "import numpy, scipy, matplotlib, cv2; print('MST27 Python dependencies: OK')"
python3 -c \
  "import sys; sys.path.insert(0, '$GRINDER_BASE_DIR/third_party/path_planner/mst27'); import _mst27_cpp; print('MST27 C++ backend: OK')"
~~~

调度器启动日志出现 [C++后端] 使用C++规划内核 表示使用加速内核；如果出现
Python fallback，路径规划仍可运行，但速度会明显下降。若工作区从 x86_64 复制到
RK3588，先检查扩展架构：

~~~bash
file "$GRINDER_BASE_DIR/third_party/path_planner/mst27/"_mst27_cpp*.so
~~~

架构不匹配时，只删除该目录下的旧 _mst27_cpp*.so，再在目标板重新编译，不能复用
x86_64 的 .so。

<a id="deployment-stvl"></a>

### 2.2 导航运行依赖：STVL 与 jemalloc

`spatio_temporal_voxel_layer`（STVL）是导航代价地图的三维障碍物图层。
本仓库 `local_costmap_params.yaml` 已引用
`spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer`，所以即使只走 MID-360S + RPP，
也必须安装 `ros-noetic-spatio-temporal-voxel-layer`。它与 Super-LIO、MST27 分别负责
障碍物表示、定位建图和作业路径规划，不能相互替代。包管理器会带入 OpenVDB、Blosc 等依赖。

~~~bash
sudo apt install -y ros-noetic-spatio-temporal-voxel-layer libjemalloc2
source /opt/ros/noetic/setup.bash
rospack find spatio_temporal_voxel_layer
dpkg-query -W ros-noetic-spatio-temporal-voxel-layer libjemalloc2
~~~

jemalloc 是程序的内存分配库。[STVL 上游说明](https://github.com/SteveMacenski/spatio_temporal_voxel_layer#running)
指出 Ubuntu 20.04 的 OpenVDB 二进制存在编译选项兼容问题，需要预加载 jemalloc。
当天 ARM64 插件加载/销毁探针在未预加载时退出码为 139，预加载后为 0；
因此板端仅给 `move_base` 加载该库，不把 `LD_PRELOAD` 全局写入所有 ROS 进程。

**当前 main 的 launch 尚未包含下述环境透传；这是本次板端额外修改，新板需补齐。**
在 `catkin_ws/src/2-dnavigation-package/2dnavigation/teb_local_planner_tutorials/launch/robot_diff_drive.launch`
的 `move_base` 节点内部加入一行（保留原有参数和节点内容）：

~~~xml
<env name="LD_PRELOAD" value="$(optenv GRINDER_NAV_LD_PRELOAD)" />
~~~

手工启动前设置以下变量；systemd 则写入后文的 `/etc/grinder/hardware.env`：

~~~bash
# RK3588 / aarch64
export GRINDER_NAV_LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libjemalloc.so.2
test -r "$GRINDER_NAV_LD_PRELOAD"
# x86_64 对应 /usr/lib/x86_64-linux-gnu/libjemalloc.so.2，不能直接照抄 ARM64 路径。
~~~

仅安装库或仅设置 `GRINDER_NAV_LD_PRELOAD` 都不能替代 launch 中的透传。
修改 launch 无需 Catkin 重编译；须在任务停止后重启导航，并重新加载地图、完成定位，
再检查 RPP 参数服务。缺失插件可能直到 `map` 到车体 TF 就绪、代价地图开始初始化时才报错。
完整排查见 [当天踩坑](#deployment-pitfalls-20261022)。

### 3. 安装工业回环依赖

| 依赖 | 作用 | 要求 |
| --- | --- | --- |
| GTSAM 4.2.2 | Pose3 图优化、iSAM2、协方差 | 必需 |
| fast_gicp | FastVGICP 点云配准 | 推荐 |
| ROS Noetic PCL 1.10 | 无 fast_gicp 时的 PCL-GICP 降级路径 | 已由 ROS/apt 提供 |

在线安装（推荐）：

~~~bash
cd "$GRINDER_WS"
JOBS=2 bash ./install_super_lio_loop_deps.sh
~~~

RK3588 建议 JOBS=2。脚本固定 GTSAM 4.2.2 和已验证的 fast_gicp 提交，并安装到
/usr/local，不替换 ROS 自带的 PCL。

GitHub 网络受限时，在电脑上下载以下源码后复制到板端：

- GTSAM 4.2.2：https://github.com/borglab/gtsam/tree/4.2.2
- fast_gicp：https://github.com/koide3/fast_gicp

~~~bash
scp gtsam-4.2.2.tar.gz neardi@<板端IP>:/home/neardi/
~~~

板端编译 GTSAM：

~~~bash
mkdir -p /home/neardi/deps
tar -xzf /home/neardi/gtsam-4.2.2.tar.gz -C /home/neardi/deps

cmake \
  -S /home/neardi/deps/gtsam-4.2.2 \
  -B /home/neardi/deps/gtsam-build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/usr/local \
  -DGTSAM_BUILD_TESTS=OFF \
  -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
  -DGTSAM_BUILD_UNSTABLE=OFF \
  -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
  -DGTSAM_USE_SYSTEM_EIGEN=ON

cmake --build /home/neardi/deps/gtsam-build --parallel 2
sudo cmake --install /home/neardi/deps/gtsam-build
sudo ldconfig
find /usr/local -name GTSAMConfig.cmake
~~~

只安装 GTSAM 也会生成工业图优化节点；缺少 fast_gicp 时会明确降级到较慢的
PCL-GICP，不会悄悄切换成轻量回环。

### 4. 正式 Release 编译

先在独立工作区构建 Livox ROS 1 驱动；本仓库不自带该驱动或 Livox SDK2。
2026-10-22 的 MID-360S 已验证组合是 SDK2 `v1.3.1` + driver2
`13eb05e4e6dd7a765b934d0c5fd6236676a57b49`，不要直接换成最新 master。
以下在普通运行用户下执行；仓库根目录和用户路径按实际部署修改。

~~~bash
export GRINDER_WS=/home/ztl/work/Grinder/catkin_ws
export LIVOX_WS="$HOME/Livox/ws_livox"
mkdir -p "$HOME/Livox" "$LIVOX_WS/src"
git clone --branch v1.3.1 --depth 1 https://github.com/Livox-SDK/Livox-SDK2.git "$HOME/Livox/Livox-SDK2"
cmake -S "$HOME/Livox/Livox-SDK2" -B "$HOME/Livox/Livox-SDK2/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$HOME/Livox/Livox-SDK2/build" --parallel 2
sudo cmake --install "$HOME/Livox/Livox-SDK2/build"
sudo ldconfig

git clone https://github.com/Livox-SDK/livox_ros_driver2.git "$LIVOX_WS/src/livox_ros_driver2"
git -C "$LIVOX_WS/src/livox_ros_driver2" checkout 13eb05e4e6dd7a765b934d0c5fd6236676a57b49
cp "$LIVOX_WS/src/livox_ros_driver2/package_ROS1.xml" "$LIVOX_WS/src/livox_ros_driver2/package.xml"
source /opt/ros/noetic/setup.bash
cd "$LIVOX_WS"
catkin_make -DROS_EDITION=ROS1 -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCMAKE_BUILD_TYPE=Release -j2 -l3
source "$LIVOX_WS/devel/setup.bash"
~~~

已有目录时检查版本后复用，不要覆盖设备的 `MID360s_config.json`。
`ROS_EDITION=ROS1` 和 ROS 1 package.xml 必须同时正确；不要加载 ROS 2 环境。

**纯 MID-360S、未安装 Aurora Linux SDK 链接库的板端，使用下面的选择性构建。**
调度器仍依赖 `slamware_ros_sdk` 的 ROS 消息，须生成消息，但不用构建 Aurora 服务节点。
第一步只完成 CMake 配置并列出目标，第二步构建实际运行目标，避免默认 `all` 拉入缺库的 Aurora。

~~~bash
cd "$GRINDER_WS"
source /opt/ros/noetic/setup.bash
source "$LIVOX_WS/devel/setup.bash"
catkin_make --only-pkg-with-deps \
  grinder_scheduler super_lio_loop cloud_to_occupancy_grid \
  regulated_pure_pursuit_controller carrot_planner move_base map_server \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  -DSUPER_LIO_REQUIRE_INDUSTRIAL_BACKEND=ON \
  -DCMAKE_EXE_LINKER_FLAGS=-fuse-ld=gold -DCMAKE_SHARED_LINKER_FLAGS=-fuse-ld=gold \
  --make-args help
cmake --build build --parallel 2 --target \
  grinder_chassis_driver_generate_messages grinder_scheduler_generate_messages \
  slamware_ros_sdk_generate_messages super_lio_node relocation_node \
  super_lio_loop_node super_lio_graph_backend_node cloud_to_occupancy_grid_node \
  regulated_pure_pursuit_controller carrot_planner move_base_node map_server \
  map_server-map_saver clear_costmap_recovery rotate_recovery navfn
source "$GRINDER_WS/devel/setup.bash"
rospack find livox_ros_driver2
ls -lh "$GRINDER_WS/devel/lib/super_lio_loop/"
~~~

Ubuntu 20.04 的 `binutils` 提供 `ld.gold`，先用 `command -v ld.gold` 确认。
当天 PCL/VTK 的 BFD 链接长时间占满单核，改用 gold 后完成；这不改变算法源码。
`SUPER_LIO_REQUIRE_INDUSTRIAL_BACKEND=ON` 会在 GTSAM 缺失时失败；产物应同时包含
`super_lio_loop_node`、`super_lio_graph_backend_node`。**编译工业后端不等于开启运行时回环**，
后者由 `super_lio_loop_closure` 等配置决定。MST27 的原生扩展仍按 2.1 节单独构建。

安装了所需 Aurora 链接库的完整系统也可使用平台脚本：

~~~bash
cd "$GRINDER_WS"
CATKIN_JOBS=2 CATKIN_LOAD=2 PROFILE=mapping ./build_grinder_platform.sh
PROFILE=runtime ./build_grinder_platform.sh   # 底盘、Aurora、调度器，包含 Aurora 服务编译
PROFILE=nav     ./build_grinder_platform.sh   # move_base、RPP/TEB
PROFILE=full    ./build_grinder_platform.sh   # 完整平台，包含 Aurora
~~~

`PROFILE=mapping` 自动传入工业后端强制选项。切换构建方式前检查
`build/CMakeCache.txt` 的 `CATKIN_WHITELIST_PACKAGES`；纯 MID-360S 板不要用
“清空白名单后全量构建”解决缺包问题，否则会重新触发 Aurora 链接失败。
跨架构/跨路径拷贝只同步源码，在目标机重新生成 `build/devel`，不要复用 Windows 或 x86_64 产物。

### 4.1 编译 Super-LIO

本小节用于已有依赖和工作区的增量编译；首次部署纯 MID-360S 板端先完成上一节的
独立 Livox 工作区与选择性目标构建，避免遗漏底盘/调度消息或触发 Aurora 链接。

本次修改包含 Super-LIO 的 C++ 位姿输出、初始定位、速度及协方差换算，以及
回环代码、launch 和调度模式管理器。**必须在目标 Linux / ROS Noetic 环境重新编译，
再重新启动相关进程；仅更新源码或重新启动 move_base 不会更新正在运行的 LIO。**
Windows 工作区用于编辑，不能把 Windows 编译产物直接用于 RK3588。

源码与编译产物的位置：

| 内容 | 仓库内位置 / 工作区产物位置 |
| --- | --- |
| Super-LIO 源码包 | `catkin_ws/src/localization/Super-LIO/src/super_lio/` |
| 配套基础库源码 | `catkin_ws/src/localization/Super-LIO/src/basic/` |
| 回环源码包 | `catkin_ws/src/localization/super_lio_loop/` |
| 建图可执行文件 | `catkin_ws/devel/lib/super_lio/super_lio_node` |
| 定位可执行文件 | `catkin_ws/devel/lib/super_lio/relocation_node` |
| 回环可执行文件 | `catkin_ws/devel/lib/super_lio_loop/` 下的两个后端节点 |

编译命令在仓库的 **`catkin_ws` 工作区根目录**执行，而不是进入 Super-LIO 源码目录
单独运行 make。`PROFILE=mapping` 会带依赖构建 `super_lio`、`super_lio_loop` 和
`cloud_to_occupancy_grid`。**当前 `PROFILE=full` 只构建平台主包，不包含 mapping；
单独运行默认构建脚本不能替代下面的 mapping 构建。**

更新代码前先通过 APP / 现有任务接口停止任务。使用 systemd 管理整套程序的板端，
再执行 `sudo grinder-service stop`；手工启动的程序应先正常退出原启动脚本和地图模式。
下面是已有依赖的板端更新构建方法，路径按实际部署位置调整：

~~~bash
export GRINDER_WS=/home/neardi/work/Grinder/catkin_ws
cd "$GRINDER_WS"
source /opt/ros/noetic/setup.bash

# 先更新 LIO、定位、回环及二维建图包
CATKIN_JOBS=2 CATKIN_LOAD=2 PROFILE=mapping bash ./build_grinder_platform.sh

# 再更新本次一起修改的调度模式管理器及相关启动资源
CATKIN_JOBS=2 CATKIN_LOAD=2 PROFILE=scheduler bash ./build_grinder_platform.sh

# 当前终端使用本工作区的包与新产物
source "$GRINDER_WS/devel/setup.bash"
~~~

已有构建缓存与当前路径、架构一致时可以增量编译，不需要先删除 `build` 或 `devel`。
平台脚本会检查路径和架构不匹配的缓存并按其配置备份处理。
无需重新生成 SL-LinkA SDK，也无需为此次更新单独重编译 move_base；协议和导航源码
没有因本次运动中心换算而改变。

首次部署或依赖不完整时，先完成本章前面的 ROS、Livox 和工业回环依赖安装。
`PROFILE=mapping` 强制要求工业回环依赖，即使运行配置暂时关闭回环也一样。
缺少 `livox_ros_driver2` 时，先确认板端 ROS 1 驱动源码存在并按第 4 节构建驱动；
不要改成 ROS 2 工程或删除这个依赖来绕过错误。驱动若在独立工作区，构建前还需
source 该工作区的 `devel/setup.bash`，使其进入当前依赖搜索路径。

构建成功后，在同一终端检查实际使用的包和产物：

~~~bash
rospack find super_lio
rospack find livox_ros_driver2
ls -lh "$GRINDER_WS/devel/lib/super_lio/super_lio_node"
ls -lh "$GRINDER_WS/devel/lib/super_lio/relocation_node"
ls -lh "$GRINDER_WS/devel/lib/super_lio_loop/super_lio_loop_node"
ls -lh "$GRINDER_WS/devel/lib/super_lio_loop/super_lio_graph_backend_node"
~~~

确认 `rospack find super_lio` 指向当前部署仓库，而不是旧工作区。
systemd 部署在构建成功后执行 `sudo grinder-service start`；手工部署则重新运行
`./start_grinder_super_lio_base.sh`。常驻服务启动后地图模式为 `IDLE`，还需通过 APP
重新选择地图并启动定位、提交底盘中心初始位姿；达到 `READY` 后再恢复任务。
既有标定沿原参数链加载，已有地图保留，先做定位一致性验证再决定是否需要重建。

底盘静止并完成定位初始化后，可只读检查：

~~~bash
rosparam get /lio/ros/legacy_tf
rosparam get /lio/extrinsic/base_to_laser_x
rosparam get /lio/extrinsic/base_to_laser_y
rosparam get /lio/extrinsic/base_to_laser_z
rosparam get /lio/extrinsic/base_to_laser_roll
rosparam get /lio/extrinsic/base_to_laser_pitch
rosparam get /lio/extrinsic/base_to_laser_yaw
rostopic echo -n 1 /lio/odom
rosrun tf tf_echo odom base_link
rosrun tf tf_echo base_link base_laser_link
~~~

标准模式下 `legacy_tf` 应为 `false`，`/lio/odom` 的 `header.frame_id` 为 `odom`、
`child_frame_id` 为 `base_link`；LIO 的六个安装参数应与固定 TF 对应的实测外参一致。
这些检查用于确认配置与发布坐标系，不能单靠帧名证明底盘中心数值准确，仍需核对
实际安装偏移、点云与地图重合程度及转动时的运动中心位置。
独立数学回归和启动链测试方法见
[Super-LIO 集成验证说明](catkin_ws/src/localization/Super-LIO/UPSTREAM.md#base-pose-and-mounting-calibration)。

### 5. MID-360S 网络和驱动验证

优先让雷达直连网口与管理网络使用不同网段。同网段时必须配置精确路由，避免数据走错网卡。
当天热点为 `192.168.1.3/24`、雷达网口为 `192.168.1.50/32`、雷达为 `192.168.1.199`，
具体 NetworkManager 配置见 [部署网络配置](#deployment-network-20261022)。先检查：

~~~bash
ip -br addr
ip route
ip route get <雷达IP>
~~~

最后一条必须显示连接雷达的网卡（例如 eth0），不能显示 lo 或路由器网卡。
host_ip、雷达 ip 和端口必须与 livox_ros_driver2/config/MID360s_config.json 一致。

~~~bash
source /opt/ros/noetic/setup.bash
source "$GRINDER_WS/devel/setup.bash"
roslaunch livox_ros_driver2 msg_MID360s.launch
rostopic hz /livox/lidar
rostopic hz /livox/imu
~~~

不需要录制 LVX 数据时：

~~~bash
roslaunch livox_ros_driver2 msg_MID360s.launch \
  rosbag_enable:=false lidar_bag:=false imu_bag:=false
~~~

Release 编译不会关闭 ROS 日志；上面的参数只关闭雷达/IMU bag 录制。

### 6. 工业级手工建图和定位

驱动正常后，另一个终端启动工业建图：

~~~bash
source /opt/ros/noetic/setup.bash
source "$GRINDER_WS/devel/setup.bash"
roslaunch cloud_to_occupancy_grid mid360_mapping.launch \
  rviz:=true save_3d_map:=true loop_closure:=true loop_backend:=industrial \
  base_to_laser_x:=0.0 base_to_laser_y:=0.0 base_to_laser_z:=0.0 \
  base_to_laser_roll:=0.0 base_to_laser_pitch:=0.0 base_to_laser_yaw:=0.0
~~~

检查：

~~~bash
rosnode list | grep -E 'super_lio_node|super_lio_loop|cloud_to_occupancy_grid'
rostopic hz /lio/odom
rostopic hz /lio/loop_cloud
rostopic hz /map
rostopic echo -n 1 /super_lio_loop/status
rosrun tf tf_echo map base_link
~~~

应看到 /super_lio_node、/super_lio_loop 和 /cloud_to_occupancy_grid。
保存 PCD、PGM 和 YAML：

~~~bash
mkdir -p /home/neardi/work/Grinder/maps
rosservice call /lio/get_3dmap "cmd_id: 1
enable_filter: true
map_dir: '/home/neardi/work/Grinder/maps'
map_name: 'grinder_map'"
rosservice call /cloud_to_occupancy_grid/save_map
~~~

定位模式不启动回环，而是加载二维地图和三维 PCD：

~~~bash
rosrun map_server map_server /home/neardi/work/Grinder/maps/grinder_map.yaml
roslaunch super_lio relocation.launch \
  rviz:=true map_dir:=/home/neardi/work/Grinder/maps \
  map_name:=loc_grinder_map.pcd \
  base_to_laser_x:=0.0 base_to_laser_y:=0.0 base_to_laser_z:=0.0 \
  base_to_laser_roll:=0.0 base_to_laser_pitch:=0.0 base_to_laser_yaw:=0.0
~~~

### 7. 正式环境一键启动

start_grinder_super_lio_base.sh 启动 Livox、底盘（可选）、RPP move_base（可选）、
调度器和 super_lio_mode_manager，但不会自动进入建图或定位：

~~~bash
cd "$GRINDER_WS"
GRINDER_BASE_DIR=/home/neardi/work/Grinder \
LIVOX_SETUP="$GRINDER_WS/devel/setup.bash" \
START_LIVOX=1 START_CHASSIS=1 START_NAV=1 \
./start_grinder_super_lio_base.sh
~~~

如果 Livox 在独立工作区：

~~~bash
LIVOX_SETUP=/home/neardi/Livox/ws_livox/devel/setup.bash \
./start_grinder_super_lio_base.sh
~~~

工业配置应保持以下值（文件：catkin_ws/src/grinder_scheduler/config/super_lio.yaml
和 scheduler.yaml）：

~~~yaml
super_lio_loop_closure: true
super_lio_autosave: false
~~~

修改 YAML 不需要重新编译，但必须重启调度器。启动后切换地图模式：

~~~bash
rosservice call /super_lio_mode/get_status
rosservice call /super_lio_mode/start_mapping "{}"
rosparam get /super_lio_mode_manager/super_lio_loop_closure
rosnode list | grep -E 'super_lio_loop|super_lio_node|relocation_node|map_server'
~~~

### 7.1 控制盒状态灯带：板端安装与启用（可选）

硬件采用微雪 ESP32-S3-Zero + 10 cm、6 颗灯珠的 5V WS2812B 灯带，
通过 RK3588 的 USB 同时通信和供电。接线、ESP32 固件编译与烧录详见
[状态灯带部署说明](firmware/status_strip/README.md)。ESP32 固件可在开发电脑上烧录，
工控机只需要安装 ROS 节点依赖，不需要安装 PlatformIO。

同步本次源码后，先停止运行中的任务，再在板端安装依赖并重新构建。
虽然灯带节点主要是 Python，新增的 Catkin 安装入口和依赖仍需更新；
本机应已具备上文的完整项目依赖，包括 slamware_ros_sdk 和 super_lio：

~~~bash
sudo apt install -y python3-serial
cd "$GRINDER_WS"
source /opt/ros/noetic/setup.bash
PROFILE=runtime ./build_grinder_platform.sh
source "$GRINDER_WS/devel/setup.bash"
~~~

上述 runtime 构建要求 Aurora 链接库齐全。纯 MID-360S 板端改用
[正式 Release 编译](#4-正式-release-编译) 中的选择性目标，不要为更新灯带而拉起 Aurora 服务编译。

构建成功后，重启底盘和调度节点，使新增的状态话题生效。
先在独立终端运行模拟后端，不会打开 USB 或发送运动控制命令：

~~~bash
source /opt/ros/noetic/setup.bash
source "$GRINDER_WS/devel/setup.bash"
roslaunch grinder_scheduler status_indicator.launch backend:=mock
~~~

在另一个同样加载工作区环境的终端查看 `rostopic echo /indicator/state`。
接入已烧录的 ESP32 后，查询 USB 身份；运行用户需有串口访问权限：

~~~bash
python3 -m serial.tools.list_ports -v
# 若当前用户没有 dialout 权限，执行后重新登录：
sudo usermod -aG dialout "$USER"
~~~

停止模拟灯带节点，再填写实际 USB 序列号启动真实后端，勿同时运行两个灯带节点：

~~~bash
roslaunch grinder_scheduler status_indicator.launch \
  backend:=serial usb_serial_number:=实际序列号
# 也可用稳定设备路径替代序列号：
# roslaunch grinder_scheduler status_indicator.launch \
#   backend:=serial device:=/dev/serial/by-id/实际设备
~~~

灯带配置位于 `catkin_ws/src/grinder_scheduler/config/status_indicator.yaml`。
默认不启用真实硬件；若直接使用 `scheduler.launch`，可传入
`enable_status_indicator:=true indicator_backend:=serial`
及 `indicator_usb_serial_number:=实际序列号`。
MID-360 + Super-LIO 的整套开机启动使用下文统一服务；手动运行启动脚本默认不启动灯带，
需要时设置 `START_INDICATOR=1` 并提供灯带 USB 身份。不要同时运行独立灯带 launch 和统一服务。

不接底盘测试灯效时，可在 `catkin_ws/src/grinder_scheduler/config/status_indicator.yaml`
设置 `ignore_chassis_status: true` 并重启灯带节点（整套服务可用
`sudo grinder-service restart`）。此时跳过底盘断连、状态缺失和过期判断，
已收到的急停提示仍保留；调度状态和 USB 通信仍需正常。
接回底盘后恢复 `false`。该配置由 ROS 灯带节点读取，无需重新烧录 ESP32；
ESP32 仅插在 Windows 电脑上时，修改此配置不会直接改变灯效。

状态指示灯对照表（6 颗灯珠同步显示，默认亮度 65%）：

| 运行状态 | 灯带颜色与效果 | 含义 |
| --- | --- | --- |
| ESP32 已供电，ROS 必需状态尚不完整 | 黄色常亮 | 启动或等待节点就绪 |
| 必需节点就绪、待机、准备、正常停止 | 绿色常亮 | 软件状态正常 |
| 路径规划 | 绿色常亮 | 不设置独立规划灯效 |
| 自动任务执行 | 蓝色呼吸，周期 2 秒 | 自动任务运行中 |
| 自动任务人为暂停 | 蓝色慢闪，1 Hz | 等待明确恢复或停止任务 |
| RPP 碰撞风险触发的暂停 | 黄色慢闪，1 Hz | 检测到前方碰撞风险，任务已进入暂停状态；不表示已经撞击 |
| 手动控制活动或非自动任务磨盘使能 | 青色常亮 | 手动控制活动或磨盘使能，不表示实测转速 |
| 自动任务完成 | 绿色闪三次后常亮 | 当前任务完成 |
| 调度错误、底盘断连、就绪后关键状态超时 | 红色慢闪，1 Hz | 需要检查状态或通信 |
| 急停提示未清除 | 红色快闪，4 Hz | 急停提示已锁存 |
| 控制器与灯带失去供电 | 熄灭 | 断电时无法显示故障颜色 |

急停和故障提示优先于运行状态。RPP 障碍信号解除、重复点击暂停，或恢复请求被拒绝时，
黄色慢闪仍保持；任务成功恢复、新任务成功启动或明确停止后才清除此暂停原因。
区分人为与 RPP 暂停需要部署新版调度代码并重新烧录 ESP32 固件（新增状态码 8）。
USB 断开导致控制器掉电时灯会熄灭；控制器仍有供电但 3 秒未收到有效帧时显示异常。
`/indicator/state` 中 `acknowledged=true` 只表示控制器已响应，不证明灯珠发光或机器安全停车。
急停灯光提示复位、诊断和实机验收步骤见部署说明。

### 7.2 软件急停与显式解除

上位机 `EmergencyStopControl.enabled=true` 会锁存软件急停、取消旧执行和导航目标，
底盘强制写入零轮速、零磨盘转速和磨盘关闭，不经缓存或速度斜坡。
节点启动及通信恢复后，先确认安全输出写入，再允许新指令；不重放旧命令。
软件运动锁存与红灯提示独立，分别持久化到运行用户的
`~/.ros/grinder_safety/` 和 `~/.ros/grinder_indicator/`。重启不会解除软件急停。
首次升级且运动记录不存在时，会把旧版已保存的急停提示迁移为运动锁存，要求显式解除。
运行账户或 HOME 改变会改变记录位置，部署时必须保持一致，不要通过删记录来解除。

确认现场安全并解除设备物理急停后，点击 Python 调试上位机的“解除急停”，
发送 `emergency_stop.enabled=false`，或在板端执行：

```bash
rosservice call /scheduler/reset_emergency_stop "{}"
rostopic echo /scheduler/motion_safety
rostopic echo /chassis/motion_safety
```

正式解除需底盘在线、安全输出成功确认，并在3秒内取得连续两次复位请求后的停止读数：
两轮实际转速零、磨盘既有转速读值零；磨盘开关读回值不参与解除判定。
急停及解除过程仍强制下发零轮速、零磨盘转速和磨盘关闭，并要求写入确认。失败保留锁存，并返回原因。
成功后保持 `STOPPED`、轮速零、磨盘关闭，另发新启动／手动指令才允许运动。
同时自动排队清除急停灯光；USB未恢复会继续重试，其他故障仍可能红色慢闪。
磨盘读回和Modbus写入确认不能替代现场机械停车验证；没有定义硬件急停复位寄存器。

无底盘测试时开启 `ignore_chassis_status: true`：解除操作可解除调度运动锁存，
返回 `success=true`／协议成功，提示测试模式、底盘未确认；保持停止并排队清灯，可继续规划测试。
此流程不解除底盘驱动锁存；已连接底盘的停车验证失败仍拒绝解除。
测试解除成功后重启，无底盘时不会因驱动重播的旧锁存再次进入调度急停；
一旦底盘连接或关闭测试开关，会重新执行驱动锁存同步，要求正式解除。
启动时若读取到已持久化的解除记录，会重新排队清除两端灯光提示并发送 ESP32 显式清除帧，
防止底盘旧锁存启动时重建红灯提示；服务／USB 未就绪会重试，新急停会取消清除。
没有解除记录或仍处于运动锁存时不会自动清灯。灯光恢复以 `/indicator/state` 和实际灯效为准。
若底盘节点存在，其显示记录必须成功清除；不直接忽略其急停TRUE。
接回底盘后关闭测试开关，再执行正式解除。

`/indicator/clear_emergency` 始终只清灯，不解除运动锁存。
`/chassis/reset_emergency_stop` 是协调流程使用的底盘验证服务，用户优先使用调度解除入口。
部署需同步两个自研包的 Python 模块／YAML、灯带节点及调试上位机；板端停止任务和服务后更新，
重新加载工作区环境并启动服务。急停逻辑本身无需重烧ESP32；亮度配置升级见下文。

灯带亮度配置在 `catkin_ws/src/grinder_scheduler/config/status_indicator.yaml`：
`brightness_percent: 65` 表示65%，范围1～100。修改后 `sudo grinder-service restart` 重新加载。
从原40%上限固件升级时需烧录一次新版S3固件，并同步灯带节点／协议模块／YAML／launch；
此后调整运行亮度无需重烧。独立上电、未通信前默认65%。完整步骤见 `firmware/status_strip/README.md`。

### 7.3 遥控响应与松手停车

平板摇杆持续指令周期为100ms；松手零速不等待任务停止回执，等待期间保存的移动目标同步作废。
板端手动执行线程保留一个最新待处理指令，松手零速替换旧移动指令，不因线程忙直接丢弃。
排队指令绑定接收时的执行代号，停止、暂停、急停或执行替换后不能恢复旧移动。
轮速ROS发布／订阅队列均为1并开启TCP_NODELAY，避免慢串口前积压多个旧速度；
双轮零速不经过轮速斜坡。调度 `manual_command_timeout_sec` 和底盘 `command_timeout` 默认均为0.5秒。
正常松手走显式零速，超时只作失联兜底；实际停止时间还包含通信、串口事务和电机响应。
`manual_reverse_guard_sec` 的换向停顿仍有效，本次保留现有1秒配置。

差速转向的单轮反向由 `manual_wheel_transition.py` 处理：仅该轮有界减速到零，等待两次独立、有效且新鲜的该轮零速测量，或完整保护时间后，再有界反向加速。
另一轮在当前目标轮速包络内协同保持前后分量，必要时允许低于目标，避免普通弧线转向人为整车停顿；输出不超过所请求的前后速度。
满杆目标前后速度为零，必要时允许短暂双轮归零，优先避免残余前冲。两轮同时反向仍等待完整保护时间。
重复、陈旧、无效、仍转动或目标非零的测量不能作为该轮停车确认；停止、急停及失联立即清零并取消过渡，灯态保持真实输出语义。
`WheelSpeedState.header.stamp` 使用实际测量时刻，避免缓存轮速重发被误当成新测量；需同步更新调度和底盘驱动。
离线回归：`test_manual_wheel_transition.py`、`test_manual_steering_guard_activity.py`、`test_wheel_feedback.py`。实机连续性和电机响应仍须现场验收。

MST27在完成Python输入解析后，仅对纯C++规划／加密阶段释放GIL，返回Python结果前重新取得锁，
避免较长规划占用同进程的Python控制回调。

部署需同步调度／底盘两个包的相关Python模块和YAML，并更新平板APK；本地覆盖配置也需核对两个超时参数。
修改C++规划内核后，在板端停止服务、更新源码，再从仓库根目录编译：

```bash
cmake -S third_party/path_planner/mst27 -B /tmp/grinder-mst27-build \
  -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3
cmake --build /tmp/grinder-mst27-build -j2
sudo grinder-service restart
```

ESP32协议未变化，无需重烧。离线回归见 `test_manual_control_delivery.py`、`test_motion_safety.py`；
实车验收需记录松手零速发出、板端接收／应用、485写帧及实际轮速归零，不能以灯效作为停车反馈。

### 8. 常见故障

| 现象 | 原因 | 处理 |
| --- | --- | --- |
| GTSAM 4.2 not found | GTSAM 未安装或 CMake 找不到 | 检查 GTSAMConfig.cmake，重跑工业编译并传入 -DGTSAM_DIR=/usr/local/lib/cmake/GTSAM |
| 只有 super_lio_loop_node | 只生成了轻量回环 | 安装 GTSAM 后用 SUPER_LIO_REQUIRE_INDUSTRIAL_BACKEND=ON 重编译 |
| unknown node /super_lio_loop | 未进入建图或工业节点启动失败 | 检查 rosnode list、回环日志和工业可执行文件 |
| ament_cmake_auto 找不到 | 混用了 ROS 2 Livox 工程 | 使用 ROS 1 Noetic 的 livox_ros_driver2，不要 source ROS 2 |
| /livox/lidar 没有数据 | 网卡、雷达 IP 或 host_ip 不匹配 | 检查 ip route get <雷达IP> 必须走雷达网卡 |
| `Cannot apply task process speed to navigation` | 导航控制器参数服务未就绪，可能是 TF 等待、STVL 缺失或插件崩溃 | 见 [STVL/jemalloc](#deployment-stvl) 和 [当天排障记录](#deployment-pitfalls-20261022)，先查 move_base 日志与 RPP 服务 |
| 建图和定位冲突 | 同时运行两种模式 | 先调用 /super_lio_mode/stop，再切换模式 |

## 手工操作参考（完成上面的安装和编译后）

MID-360S + Super-LIO 的固定内存实时二维建图见
`catkin_ws/src/2-dnavigation-package/cloud_to_occupancy_grid/README.md`。该流程直接使用 LIO 位姿和
当前配准点云生成 `/map`，不再叠加 GMapping。

### 1. 已完成编译后的手工操作

```bash
cd ~/work/Grinder/catkin_ws
# 工业正式编译请按上文第 4 节执行；这里仅加载已生成的工作区
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

### 4. 建图模式（手工调试）

启动 Super-LIO、回环后端和固定尺寸二维栅格地图：

```bash
roslaunch cloud_to_occupancy_grid mid360_mapping.launch \
  rviz:=true \
  save_3d_map:=true \
  loop_closure:=true \
  loop_backend:=industrial \
  base_to_laser_x:=0.0 base_to_laser_y:=0.0 base_to_laser_z:=0.0 \
  base_to_laser_roll:=0.0 base_to_laser_pitch:=0.0 base_to_laser_yaw:=0.0
```
```bash
roslaunch cloud_to_occupancy_grid mid360_mapping.launch \
  rviz:=true \
  save_3d_map:=true \
  loop_closure:=false \
  base_to_laser_x:=0.0 base_to_laser_y:=0.0 base_to_laser_z:=0.0 \
  base_to_laser_roll:=0.0 base_to_laser_pitch:=0.0 base_to_laser_yaw:=0.0
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

### 5. 旧地图重定位模式（手工调试）

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

首次启动后可在 RViz 使用 `2D Pose Estimate` 发布 `/initialpose` 做人工联调；正式 APP
链路通过 `RadarRelocalizationRequest (0x052A)` 一并传入当前 `map_id/map_revision` 和
`x/y/heading_deg`。调度器只在身份匹配后调用模式管理器发布 `/initialpose`。验证：

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

### 8. 启动调度器和 Super-LIO 模式管理器

使用 Super-LIO 时，显式把调度定位输入设为 `/lio/odom`。move_base 已经单独
启动时，只启动 `scheduler.launch`，不要再启动一个包含导航的系统 launch：

```bash
roslaunch grinder_scheduler scheduler.launch \
  odom_topic:=/lio/odom \
  map_topic:=/map \
  super_lio_map_root:=/home/neardi/work/Grinder/maps
```

验证：

```bash
rosnode info /grinder_scheduler
rosnode info /super_lio_mode_manager
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

### 9. 一键启动 Super-LIO 常驻服务

```bash
cd /home/neardi/work/Grinder/catkin_ws
chmod +x start_grinder_super_lio_base.sh
./start_grinder_super_lio_base.sh
```

该脚本启动 Livox、底盘、RPP move_base、调度器和
`super_lio_mode_manager`。启动后地图模式为 `IDLE`，不会自动启动建图或重定位。

正式运行时由 APP 的 `MapModeRequest` 和 `MapSaveRequest` 控制地图生命周期：

```text
MAP_MODE_MAPPING enabled=true
  -> 启动 Super-LIO + cloud_to_occupancy_grid

MapSaveRequest map_id="" 或 map_id="LIVE_MAP"
  -> 保存 PCD 与 PGM/YAML
  -> LOWER 生成唯一正式 map_id，并在响应中返回
  -> 停止并复查建图节点
  -> 返回 map_revision、mapping_stopped、localization_started=false

MAP_MODE_LOCALIZATION enabled=true
  -> 返回 STARTING，再进入 LOCALIZING，等待带 map_id/map_revision 的初始位姿
  -> APP 轮询 0x052D；只有 lifecycle_state=READY 才允许开始任务
```

APP 必须在 `MAPPING` 仍处于活动状态时发送 `MapSaveRequest`，不要先发送
`MAP_MODE_LOCALIZATION`。`LIVE_MAP` 只是实时建图源的保留 ID，不会作为资产目录名；
保存成功后响应中的新 `map_id/map_revision` 才是后续地图选择和重定位使用的身份。地图
保存成功不表示定位已经启动或已达到 `READY`。

每张地图保存为独立资产包：

```text
/home/neardi/work/Grinder/maps/<map_id>/
├── loc_map.pcd
├── plan_map.pcd
├── map.yaml
├── map.pgm
└── map_info.json
```

可直接检查模式管理器：

```bash
rosservice call /super_lio_mode/get_status
rosservice call /super_lio_mode/start_mapping
rosservice call /super_lio_mode/stop
```

手工调用保存服务仅用于联调，`map_id` 不得与已有资产包重复：

```bash
rosservice call /super_lio_mode/save_map "map_id: 'test_001'
map_name: '测试地图'"
```

保存任一环节失败时建图保持运行，可以再次保存。地图已保存但重定位启动失败时，
资产包仍保留，底盘任务使能保持关闭。

### 10. 单独启动调度

```bash
cd ~/work/Grinder/catkin_ws
source devel/setup.bash
roslaunch grinder_scheduler scheduler.launch \
  odom_topic:=/lio/odom \
  map_topic:=/map \
  super_lio_map_root:=/home/neardi/work/Grinder/maps
```

### 11. 上位机启动（PyQt）

```bash
cd ~/work/Grinder
python3 third_party/sl_linka/sdk/python/tools/sl_linka_pyqt_debugger.py
```

连接参数：

- Host：调度程序IP（本机可 `127.0.0.1`）
- Port：`8002`

### 12. 板端开机自启：主系统与灯带统一开关

MID-360 + Super-LIO 链路使用 `grinder.service` 管理 ROS master、Livox、底盘、RPP、
调度和灯带。开机只启动节点，任务仍需明确下发启动指令。
服务以安装时指定的普通用户运行，并使用该用户的 HOME 保存急停提示记录。

2026-10-14 已在 LPA3588（用户 `neardi`）完成重启后的主系统自启验证：
服务显示 `enabled`、`active (running)`，Livox、底盘、RPP/move_base、调度器、
Super-LIO 模式管理器及 mediamtx 进程已启动。本次未接 ESP32，使用 `START_INDICATOR=0`；
此结果验证开机启动流程，不代表灯带、底盘运动、定位或视频链路的实机验收。

完成构建、灯带固件烧录和手动联调后，先停止任务，再退出已运行的启动脚本及独立灯带节点。
在板端安装（将用户名替换为实际运行用户，工作区从脚本位置自动确定）：

~~~bash
cd "$GRINDER_WS"
sudo bash ./install_grinder_service.sh neardi
sudo nano /etc/grinder/startup.conf
~~~

首次安装默认 `AUTOSTART=0`，不会启动节点。重新安装保留已有配置和自启设置。
若 nano 提示 `/etc/grinder` 目录不存在，应先完成安装；若安装提示缺少
`scripts/grinder-service`，请将该文件同步到板端同名路径（文件无 `.py` 后缀）后重试。
配置为 `KEY=VALUE`，包含空格的值需用引号包裹；不展开 `$HOME`、`~` 或 shell 命令。
示例中的路径、运行用户、USB 序列号必须按板端实际情况填写：

~~~ini
AUTOSTART=0
START_INDICATOR=1
RUN_USER=neardi
WORKSPACE=/home/neardi/work/Grinder/catkin_ws
LIVOX_SETUP=/home/neardi/Livox/ws_livox/devel/setup.bash
NAV_MAP_YAML=/home/neardi/work/Grinder/maps/grinder_map.yaml
INDICATOR_USB_SERIAL_NUMBER=实际USB序列号
INDICATOR_DEVICE=''
~~~

`LIVOX_SETUP` 可留空，由启动脚本从项目环境和常见 Livox 工作区查找。
还没有 ESP32 时设置 `START_INDICATOR=0`，两个 USB 身份字段留空，先验证主系统开机自启。
拿到并烧录 ESP32 后，设置 `START_INDICATOR=1` 并填写 USB 身份，再应用配置、在任务停止后重启服务。
旧配置没有此字段时默认 `1`，保持原有灯带启动行为；旧版管理命令需先更新并重新运行安装脚本。
仅复制 `catkin_ws/scripts/grinder-service` 不会更新系统已安装的管理命令，必须重新执行安装脚本。
灯带 USB 序列号与 `INDICATOR_DEVICE=/dev/serial/by-id/实际设备` 二选一。
服务默认使用真实串口后端，自动获得 `dialout` 补充组权限；USB 未插入时灯带节点持续重连。
导航地图必须存在，工作区必须已构建。

~~~bash
# 修改配置后应用；不会启动、停止或重启当前节点
sudo grinder-service apply

# 保存 AUTOSTART 并切换下次开机行为；不改变当前运行状态
sudo grinder-service enable
sudo grinder-service disable

# 立即操作整套节点，不改变自启配置
sudo grinder-service start
sudo grinder-service stop
sudo grinder-service restart

grinder-service status
sudo grinder-service logs
~~~

常用开关与重启命令（逐条执行）：

| 操作 | 命令 | 生效范围 |
| --- | --- | --- |
| 开启开机自启 | `sudo grinder-service enable` | 保存 `AUTOSTART=1`，下次开机自动启动；当前节点不变 |
| 关闭开机自启 | `sudo grinder-service disable` | 保存 `AUTOSTART=0`，下次开机不自动启动；当前节点不变 |
| 立即启动节点 | `sudo grinder-service start` | 启动当前整套节点，不改变自启开关 |
| 立即停止节点 | `sudo grinder-service stop` | 停止当前整套节点，不改变自启开关 |
| 重启整套节点 | `sudo grinder-service restart` | 停止后重新启动当前整套节点，不重启板端 |
| 重启板端 | `sudo reboot` | 重启 RK3588，节点是否自动启动取决于自启开关 |
| 查看服务与自启状态 | `grinder-service status` | `enabled` / `disabled` 为自启状态，`active (running)` 为运行状态 |
| 编辑启动配置 | `sudo nano /etc/grinder/startup.conf` | 可修改 `AUTOSTART`、`START_INDICATOR`、路径和 USB 身份 |
| 应用手动修改的配置 | `sudo grinder-service apply` | 同步自启开关及服务定义，不重启当前节点 |

也可直接在配置文件中将 `AUTOSTART` 改为 `1`（自启）或 `0`（不自启），保存后执行
`sudo grinder-service apply`。修改灯带开关、路径或 USB 身份后，还需在任务停止后执行
`sudo grinder-service restart`，使运行中的节点使用新配置。
`START_INDICATOR=0` 只关闭灯带节点；`AUTOSTART=0` 关闭整套服务的开机自启。
执行节点停止、节点重启或板端重启前，先通过现有任务接口停止任务。

维护时先通过现有任务接口停止任务，再执行 `sudo grinder-service stop`。
若下次开机也不希望启动，再执行 `sudo grinder-service disable`。
更改运行用户、路径或 USB 身份后先 `apply`，再在任务停止后 `restart`。
服务启动失败或受监督进程退出时清理整套子进程，默认不自动重启整套服务。
板端已有启用的 `pm2-<运行用户>.service` 时，Grinder 排在它之后启动，复用它启动的 ROS master，
避免两套开机流程同时启动 master；此排序不会启用或强制依赖 PM2。
如果 PM2 在运行期间主动重启 master，ROS 链路仍可能中断，需要单独协调 PM2 的管理策略。
LPA3588 已有 PM2 管理的 `roscore`、`rosbridge` 等进程，应保留其服务；安装后用
`systemctl show grinder.service -p After` 确认包含 `pm2-neardi.service`。
ROS/Catkin 环境加载期间临时关闭未定义变量检查，避免全新 systemd 环境报
`ROS_DISTRO：未绑定的变量`；项目脚本自身仍保持严格检查。
灯带 launch 保留自身 `respawn`；同一工作区的重复启动由启动脚本锁拒绝。
服务启动成功只代表进程已启动，ROS 状态是否完整应查看灯带与 `/indicator/state`。

管理日志见 journal；各节点详细日志位于 `$GRINDER_WS/logs/super_lio_base_*`。
自启关闭且从未通信时，ESP32 通电显示启动黄灯；已经通信后停止节点且 USB 保持供电，
固件通信超时显示红灯，急停提示仍优先。USB 断电则熄灭。
板端验收包括开启／关闭后分别重启、完整状态变绿、USB 断连重连，以及停止后无残留节点。

没有 ESP32 时，可先保持两个 USB 字段为空并设置 `START_INDICATOR=0`，在任务停止、
旧的手动启动脚本退出后逐条执行：

~~~bash
sudo grinder-service apply
sudo grinder-service start
grinder-service status
# 确认持续运行后开启自启；确认任务停止后再重启板端
sudo grinder-service enable
sudo reboot
# 重新登录后等待约 20 秒，再检查
grinder-service status
~~~

重启后应同时看到 `enabled` 和 `active (running)`。若显示 `failed`，先读取
`sudo journalctl -u grinder.service -b -n 80 --no-pager`，并检查最新
`logs/super_lio_base_*/*.log`，保留失败日志后再重试。
本次板端日志仍有 MPP VP8 编码初始化失败；服务保持运行，但视频链路需另行验证。

离线验证（Linux，不连接硬件）：

~~~bash
python3 catkin_ws/scripts/test_grinder_service.py -v
~~~

<a id="deployment-20261022"></a>

## 2026-10-22 RK3588 板端部署实录与复现

本节记录当天实际部署，基线为 main 的 `e623f76e1e072e4e0faa122cafd1be054925297e`。
通用安装步骤已更新在上文；这里补齐设备配置、执行顺序和问题处理，便于新板复现。
板端独有的 launch 修改、NetworkManager 配置、systemd drop-in **不随 git clone 自动安装**。
密码由设备持有人设置并留在板端受限配置中，仓库不保存 SSH 或热点密码。

### 1. 最终部署清单与安装顺序

| 项目 | 当天最终配置 |
| --- | --- |
| 板端系统 | RK3588 / aarch64，Ubuntu 20.04.6，ROS 1 Noetic，Python 3.8，约 8 GB 内存 |
| 源码与运行用户 | `/home/ztl/work/Grinder`，服务以普通用户 `ztl` 运行；SSH 管理用户不等于服务用户 |
| 链路 | MID-360S → Livox → Super-LIO → 二维地图/定位 → 调度 → RPP/move_base；未启用 Aurora |
| Livox | SDK2 v1.3.1；driver2 `13eb05e4e6dd7a765b934d0c5fd6236676a57b49`；独立工作区 `/home/ztl/Livox/ws_livox` |
| 算法依赖 | GTSAM 4.2.2；fast_gicp `0e7ec1441c99f7be453db2ea216d5de029387417`，安装到 `/usr/local` 后执行 `ldconfig` |
| 导航运行依赖 | STVL `1.4.5-1focal.20250521.012635`、OpenVDB 6.2、Blosc、libjemalloc2；版本号为当天记录，安装时以 focal/arm64 仓库可用包为准 |
| 规划器/协议 | MST27 Python 依赖和 CPython 3.8 ARM64 原生扩展；protobuf 4.25.1 |
| 底盘 | `/dev/ttyS3`，19200，Modbus 从站 1；当天未接控制器，未做运动验收 |
| 灯带 | ESP32-S3 USB CDC，USB 序列号 `AC:27:6E:D2:C4:F0`，亮度 65%；按 USB 身份匹配而非固定 ttyACM 编号 |
| 视频基础环境 | FFmpeg；MediaMTX v1.21.2 linux_arm64，安装为 `tools/mediamtx/mediamtx_aarch64`，可执行权限；选择 libx264，未验收实际视频 |
| 开机服务 | `grinder.service` enabled；配置文件和额外覆盖见下文 |

按顺序完成：系统/ROS apt 依赖及 protobuf → Livox SDK/驱动 → GTSAM/fast_gicp →
选择性 Catkin 构建 → MST27 原生扩展 → 网络与雷达 JSON → 设备 YAML → STVL/jemalloc
launch 透传 → 手工启动检查 → 安装 systemd 并启用自启 → 重启后检查数据和导航服务。
前文已给出各项安装命令，不要用“包能编译”替代启动后的检查。

板端网络离线时，先在能联网的环境准备适配 Ubuntu focal **arm64** 的完整依赖，
核对 `.deb` 的架构、依赖与 SHA256 后再传输；不能拿开发电脑的 amd64 包安装到 RK3588。
源码可以用 `git bundle` 携带 main 历史传到板端，检出后用 `git rev-parse HEAD` 核对提交，
不要把本机未提交修改或旧的 `devel` 当成 main 部署。

MediaMTX 从 [官方 v1.21.2 发布页](https://github.com/bluenviron/mediamtx/releases/tag/v1.21.2)
选择 `linux_arm64` 压缩包，解压其中的 `mediamtx` 为上述 `mediamtx_aarch64`，
执行 `chmod +x` 并用 `file` 确认 AArch64；x86_64 板应选择相应架构。
只有 README 或 Windows 可执行文件的目录不能作为 Linux RTSP 服务使用。

当天还配置了 8 GB swap、`vm.swappiness=10`，启用 SSH、NetworkManager 和时间同步；
journald 持久日志限制 SystemMaxUse=300M、RuntimeMaxUse=100M，项目日志按每日、
最多 20 MB 轮转并保留 7 份。相关文件在 `/etc/fstab`、`/etc/sysctl.d/`、
`/etc/systemd/journald.conf.d/`、`/etc/logrotate.d/`；新板先检查已有配置，避免重复建 swap 或叠加轮转规则。
离线运行前校准时间；启用了时间同步服务不代表断网后仍能联系 NTP。

<a id="deployment-network-20261022"></a>

### 2. 热点、雷达网口与同网段路由

| 用途 | 网卡/配置名 | 地址与说明 |
| --- | --- | --- |
| 管理热点 | `wlP2p33s0` / `grinder-hotspot` | SSID `Grinder-MID360S`，板端 `192.168.1.3/24`，WPA2，自动连接 |
| 管理电脑 | WLAN | 当天固定 DHCP 租约 `192.168.1.10` |
| 雷达直连口 | `enP3p49s0` / `grinder-mid360s` | 板端 `192.168.1.50/32`，仅添加到雷达的主机路由 |
| MID-360S | 直连上述网口 | `192.168.1.199`，序列号 `ARMCP7K0034999`；不能把普通 MID-360 配置当成 MID-360S |
| 原有有线管理口 | `eth0` | `192.168.1.2/24`，后续已拔线；当前从热点 `.3` 登录 |

先用 `ip -br link`、`nmcli device status` 和 `nmcli connection show` 确认实际网卡、已有配置名。
以下 `connection add` 仅用于尚无对应 profile 的新板；已有 profile 用 `connection modify` 修改。
网络切换会影响 SSH，应从仍可用的管理链路操作。

~~~bash
# 新建热点；密码在板端交互输入，避免写进仓库。
read -r -s -p 'Hotspot password (8+ characters): ' AP_PASSWORD
printf '\n'
sudo nmcli connection add type wifi ifname wlP2p33s0 con-name grinder-hotspot ssid Grinder-MID360S
sudo nmcli connection modify grinder-hotspot \
  802-11-wireless.mode ap 802-11-wireless.band bg 802-11-wireless.channel 6 \
  802-11-wireless-security.key-mgmt wpa-psk 802-11-wireless-security.psk "$AP_PASSWORD" \
  ipv4.method shared ipv4.addresses 192.168.1.3/24 ipv4.route-metric 600 \
  ipv6.method disabled connection.autoconnect yes connection.autoconnect-priority 100
unset AP_PASSWORD

# 雷达口用 /32，防止其 /24 路由抢走热点客户端流量。
sudo nmcli connection add type ethernet ifname enP3p49s0 con-name grinder-mid360s \
  ipv4.method manual ipv4.addresses 192.168.1.50/32 ipv4.routes '192.168.1.199/32' \
  ipv4.never-default yes ipv6.method disabled connection.autoconnect yes
~~~

当天该内核不支持 `ip rule` 策略路由，返回 `Operation not supported`；最终采用主路由表中的
雷达 `/32` 路由，加热点客户端 `/32` 租约路由。没有使用仍然失败的 NM routing-rules 配置。

创建 `/etc/NetworkManager/dnsmasq-shared.d/grinder-hotspot.conf`。
下面 MAC 对应当天的电脑、雷达口和雷达，新设备须替换；后两条用于把 `.50`、`.199`
从热点动态地址分配中保留，防止客户端占用设备静态地址：

~~~ini
dhcp-host=4c:b7:e0:a4:cc:13,192.168.1.10
dhcp-host=1a:d1:c1:81:88:d7,192.168.1.50
dhcp-host=ec:72:f7:89:06:cb,192.168.1.199
dhcp-script=/usr/local/lib/grinder/hotspot-lease-route
~~~

创建 `/usr/local/lib/grinder/hotspot-lease-route` 并设为 root 所有、权限 0755：

~~~sh
#!/bin/sh
case "${DNSMASQ_INTERFACE:-wlP2p33s0}" in wlP2p33s0) ;; *) exit 0 ;; esac
case "$3" in 192.168.1.*) ;; *) exit 0 ;; esac
case "$1" in
  add|old) /sbin/ip route replace "$3/32" dev wlP2p33s0 src 192.168.1.3 metric 50 ;;
  del) /sbin/ip route del "$3/32" dev wlP2p33s0 metric 50 2>/dev/null || true ;;
esac
~~~

同网段网卡还需配置 ARP 与反向路径检查。写入 `/etc/sysctl.d/91-grinder-hotspot.conf`：

~~~ini
net.ipv4.conf.all.rp_filter=2
net.ipv4.conf.default.rp_filter=2
net.ipv4.conf.all.arp_ignore=1
net.ipv4.conf.default.arp_ignore=1
net.ipv4.conf.all.arp_announce=2
net.ipv4.conf.default.arp_announce=2
~~~

文件保存后执行 `sudo sysctl --system`，再启用雷达 profile 与热点 profile。
dnsmasq 配置在热点重新激活后读取；已有客户端需重新取得租约，让租约脚本补上路由。
网卡名、地址和 hook 必须一起修改，静态 IP 客户端也需显式添加对应 `/32` 路由。

~~~bash
sudo nmcli connection up grinder-mid360s
sudo nmcli connection up grinder-hotspot
ip route get 192.168.1.199  # 应为 enP3p49s0，src 192.168.1.50
ip route get 192.168.1.10   # 应为 wlP2p33s0，src 192.168.1.3
~~~

Livox 工作区的 `src/livox_ros_driver2/config/MID360s_config.json` 设置：
`lidar_summary_info.lidar_type=8`；使用 `Mid360s` 段，`host_net_info` 为列表，
其中 `host_ip=192.168.1.50`，主机端口为 56101、56201、56301、56401、56501；
`lidar_configs[0].ip=192.168.1.199`。保留驱动同版本的 JSON 结构，不从普通 MID-360 示例拼字段。
启动脚本应找到 `msg_MID360s.launch`；启动前按脚本的 Livox UDP 端口预检确认无占用，避免重复起驱动。

热点实测工作在 2.4 GHz、信道 6、20 MHz，发射功率 12 dBm；当天未上调带宽或功率。
RTL8852BE 的 Wi-Fi 6 能力不等于当前热点已使用 Wi-Fi 6；当天电脑协商为 802.11n，
没有做吞吐量测试，不能以标称速率作为实际传输带宽。

### 3. 设备配置、启动环境与自启覆盖

底盘 `config/chassis_driver.yaml` 的串口设置为 `/dev/ttyS3`，波特率 19200、从站地址 1。
配置调度的 `scheduler.local.yaml`，通过启动预检确认选用 Super-LIO 链路。
当天未开启内部轮速融合，`super_lio_loop_closure=false`；不要因安装了工业依赖就宣称回环已启用。
灯带设置 `START_INDICATOR=1`、USB 身份和 65% 亮度。
当前 `status_indicator.yaml` 的 `ignore_chassis_status=true` 是无底盘台架配置，接真实底盘后需改为 false，
恢复底盘状态参与就绪判断；不要通过忽略底盘状态或清除急停来掩盖硬件故障。

所有 ROS 节点都在板端，使用 `ROS_IP=127.0.0.1`，避免管理网线拔掉后 ROS 仍向 `.2` 宣告。
APP 使用板端热点地址 `.3` 的 SL-LinkA TCP 8002，视频入口为 RTSP 8554；
如果未来要跨机器直接运行 ROS 节点，应重新设计可达的 ROS 地址，不能照用 loopback。
交互终端可在 `/etc/profile.d/grinder-ros.sh` 加载已构建的工作区并设置仓库路径与 ROS_IP；
systemd 不读取交互 shell 配置，因此还需下面的显式环境。

先按 [统一自启安装](#12-板端开机自启主系统与灯带统一开关) 运行
`sudo bash ./install_grinder_service.sh ztl`。当天 `/etc/grinder/startup.conf`：

~~~ini
AUTOSTART=1
RUN_USER=ztl
WORKSPACE=/home/ztl/work/Grinder/catkin_ws
LIVOX_SETUP=/home/ztl/Livox/ws_livox/devel/setup.bash
NAV_MAP_YAML=/home/ztl/work/Grinder/maps/navigation.yaml
INDICATOR_USB_SERIAL_NUMBER=AC:27:6E:D2:C4:F0
INDICATOR_DEVICE=''
START_INDICATOR=1
~~~

`NAV_MAP_YAML` 必须指向存在的文件。当天 `maps/navigation.yaml` 是未知栅格的启动占位图，
不代表已测绘地图，也不在开机时自动进入定位。新板应指向已有地图；尚无地图时可准备独立占位图，
不得覆盖已有地图资产，后续仍需在 APP 中建图、保存、加载和定位。

仅在新板没有任何地图时，可生成独立的 1×1 未知栅格占位文件。以下命令遇到同名文件即退出，
不会覆盖地图；先确认 `GRINDER_WS` 指向目标板工作区：

~~~bash
python3 - <<'PY'
import os
from pathlib import Path
maps = Path(os.environ['GRINDER_WS']).parent / 'maps'
maps.mkdir(parents=True, exist_ok=True)
pgm, yaml = maps / 'navigation.pgm', maps / 'navigation.yaml'
if pgm.exists() or yaml.exists():
    raise SystemExit('Map file already exists; select an existing map instead.')
with pgm.open('xb') as f:
    f.write(b'P5\n1 1\n255\n\xcd')
with yaml.open('x') as f:
    f.write('image: navigation.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n'
            'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
PY
~~~

stock `grinder-service` 对配置键有白名单，并在 `run` 中固定开启 Livox/底盘/导航。
不要往 `startup.conf` 添加 `START_LIVOX` 或 jemalloc 等未知字段。
为支持先无雷达安装、后接雷达，以及本次 jemalloc 修复，板端额外创建
`/etc/grinder/hardware.env`（此文件由下面的 drop-in 读取）：

~~~ini
START_LIVOX=1
START_CHASSIS=1
START_NAV=1
GRINDER_NAV_LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libjemalloc.so.2
~~~

无雷达阶段可临时设置 `START_LIVOX=0`；接雷达后改回 1。
同时完成前文 [move_base 的 LD_PRELOAD 透传](#deployment-stvl)。
创建 `/etc/systemd/system/grinder.service.d/environment.conf`：

~~~ini
[Service]
Environment=GRINDER_BASE_DIR=/home/ztl/work/Grinder
Environment=OPENBLAS_NUM_THREADS=1
Environment=OMP_NUM_THREADS=2
Environment=ROS_IP=127.0.0.1
EnvironmentFile=/etc/grinder/startup.conf
EnvironmentFile=/etc/grinder/hardware.env
ExecStart=
ExecStart=/usr/bin/bash /home/ztl/work/Grinder/catkin_ws/start_grinder_super_lio_base.sh
WorkingDirectory=/home/ztl/work/Grinder/catkin_ws
Restart=on-failure
RestartSec=5
~~~

此覆盖沿用安装器设置的 `User`、`Group`、串口权限和启动顺序，但将 ExecStart 直接指向主启动脚本。
因此 `grinder-service run` 的启动前配置校验不再经过；维护时需自行核对路径、地图、USB 身份和环境文件，
并先通过项目配置预检。`apply` 不负责生成这些额外文件，也不会清除旧 drop-in。
默认安装器为 `Restart=no`，当天覆盖改为 on-failure；排障时可先停止服务，避免失败反复启动。

在 APP 中先终止任务，确认任务使能已关闭后，再执行：

~~~bash
sudo grinder-service apply
sudo systemctl daemon-reload
sudo grinder-service enable
sudo grinder-service restart
systemctl cat grinder.service
systemctl show grinder.service -p User -p ExecStart -p Restart -p NRestarts
grinder-service status
~~~

开机启动 ROS master、Livox、底盘、RPP/move_base、调度、Super-LIO 模式管理器、MediaMTX 和灯带。
建图/定位由 APP 操作模式管理器启动；开机不自动开始作业。
服务重启后要重新加载所需地图并定位，再检查导航就绪；`active (running)` 不等于定位和参数服务已就绪。
不要同时运行独立灯带进程与主系统灯带节点，避免串口被重复占用。

### 4. 部署后验收与当天结果

先 source ROS 和 Livox/Grinder 环境，再执行诊断。`ldd` 也需在该环境下使用，
否则 `/opt/ros/noetic/lib` 下的传递依赖可能被误报为缺失。

~~~bash
source /opt/ros/noetic/setup.bash
source /home/ztl/Livox/ws_livox/devel/setup.bash
source /home/ztl/work/Grinder/catkin_ws/devel/setup.bash
export ROS_IP=127.0.0.1
systemctl is-enabled grinder.service
systemctl is-active grinder.service
sudo journalctl -u grinder.service -b -n 100 --no-pager
rosnode list
rostopic hz /livox/lidar
rostopic hz /livox/imu
rostopic echo -n 1 /indicator/state
rostopic echo -n 1 /chassis/task_enable
ss -lnt | grep -E ':8002|:8554'
~~~

`rostopic hz` 持续采样，观察后 Ctrl+C。
APP 加载地图并完成定位、保持任务停止后，再检查 TF 和 RPP 服务：

~~~bash
rosrun tf tf_echo map base_link
rosservice list | grep /move_base/RegulatedPurePursuitController
python3 - <<'PY'
import rospy
from dynamic_reconfigure.client import Client
rospy.init_node('check_rpp_configuration', anonymous=True)
client = Client('/move_base/RegulatedPurePursuitController', timeout=5.0)
# 空更新触发一次服务往返，不改变速度或下发运动指令。
result = client.update_configuration({})
print('desired_linear_vel:', result['desired_linear_vel'])
PY
~~~

`tf_echo` 持续输出，观察后 Ctrl+C。实际车体 frame 若有覆盖，以板端配置为准。
应看到 move_base 进程持续稳定、日志已创建 RPP local_planner，参数服务成功应答，
而非仅有 rosnode 名称或 rosparam 静态参数。未加载地图时等待 TF 的现象不能直接判定插件修复失败。

| 验收项 | 当天已验证的范围 |
| --- | --- |
| 重启与网络 | 板端新 boot 后热点、雷达 profile 自动恢复；grinder enabled、active；两条主机路由指向正确网卡 |
| 雷达 | 真实 MID-360S 点云约 10.39 Hz，IMU 约 199.77 Hz；重启后重新确认有数据 |
| 灯带协议 | READY 状态、串口 ACK、65% 亮度下发，无串口错误；ACK 不代替实际颜色和急停提示的现场检查 |
| 导航修复 | STVL 已安装；move_base 环境确认 jemalloc 预加载；重定位后 RPP 空更新应答，速度约 0.03 m/s，进程持续稳定 |
| 离线回归 | 服务管理 15 项、调度配置 5 项、MST27 预处理 10 项，共 30 项通过 |
| 未完成的硬件项 | 底盘未连接，未验收行走、轮速、磨盘与实体急停；未验收真实视频播放；未验收运行时回环效果 |

板端检查记录位于 `/etc/grinder/DEPLOYMENT.md`、`deployment-verification.json`、
`navigation-repair-verification.json`；它们是设备运行记录，不能替代当前仓库源码和配置。

<a id="deployment-pitfalls-20261022"></a>

### 5. 当天部署遇到的问题与处理

| 问题/现象 | 定位到的原因 | 处理与复查 |
| --- | --- | --- |
| Livox 编译提示 `kLivoxLidarDoubleEchoData` / `LivoxLidarDoubleEchoRawPoint` 不存在 | 新 driver master 包含更新的 MID-360L 接口，与 SDK2 v1.3.1 不匹配 | 固定本文 MID-360S 已验证的 driver commit；使用 ROS 1 package.xml 和 `ROS_EDITION=ROS1` 重建 |
| 协议导入失败，找不到 `google.protobuf.internal.builder` | 系统 protobuf 3.6.1 早于当前生成代码所需版本 | Python 3.8 环境安装 protobuf 4.25.1；用实际服务用户/解释器验证导入 |
| 找不到 Aurora 的 Linux `.so`，完整构建失败 | runtime/full profile 包含 Aurora；纯 MID-360S 不运行 Aurora，但调度仍需其消息 | 用第 4 节选择性目标构建并生成 `slamware_ros_sdk` 消息；不通过全量构建或开启 Aurora 绕过 |
| Super-LIO 链接看似卡死，单核 CPU 很高 | 大量 PCL/VTK 依赖使用 BFD 链接较慢 | 改用 `-fuse-ld=gold`，控制并行 2、准备 swap；当天未观察到 OOM，不能把慢链接一概诊断成内存不足 |
| MST27 已有 Python 文件，但缺少原生内核 | Catkin 构建不会自动替代 MST27 独立 CMake 构建 | 安装 pybind11/科学 Python 依赖，单独生成 CPython 3.8 aarch64 `.so` 并跑预处理回归；不能复制 x86 扩展 |
| 原管理网口地址前缀错误 | 旧 NM profile 保存为 `192.168.1.2/0` | 修正并持久化为 `/24`；当前管理已切到热点 `.3`，不能继续用拔线的 `.2` 诊断在线状态 |
| 热点和雷达同网段时 SSH 或雷达走错网口；`ip rule` 不支持 | 两个 `/24` 直连路由竞争，板端内核缺少策略路由支持 | 雷达口 `/32` 加雷达主机路由，热点租约 hook 加客户端 `/32`；保留静态设备地址并核对两个 `ip route get` |
| 灯带一直黄灯 | 无硬件阶段禁用了灯带节点，ESP32 保持上电待通信状态 | 接入后启用 `START_INDICATOR`、设置 USB 身份，检查 `/indicator/state` 的 ACK 和错误；当天未重新烧录固件 |
| source ROS 后运行 `rviz` 提示未找到命令 | 板端最初安装的是无桌面 `ros-base`，不是 RViz 安装或环境变量损坏 | 单独补装 RViz、JSK RViz 插件及 JSK visualization 套件；离线板按 ARM64 完整依赖下载、校验后安装，图形启动仍需桌面会话 |
| MobaXterm 启动 RViz 报 X11 `Unsupported extension used` 并断开 | 转发显示与板端原生图形路径不兼容；登录会话的 `QT_XCB_GL_INTEGRATION=xcb_egl` 仍会强制加载 Mali；板端本地显示也缺 GLX | 保留 MobaXterm 的 DISPLAY，显式指定 `QT_XCB_GL_INTEGRATION=xcb_glx`，配合 Mesa/llvmpipe 和禁用 Qt MIT-SHM；修正后的板端 `rviz-x11` 已在带有原始 EGL 环境变量的条件下验证 |
| APP 报 `Cannot apply task process speed to navigation` | 当天直接原因是 STVL 未安装，costmap 初始化抛 `LibraryLoadException`，move_base 退出/重生，RPP 参数服务不可用 | 安装 `ros-noetic-spatio-temporal-voxel-layer` 及依赖；这是导航服务失败，不应修改工艺速度或绕过任务启动检查 |
| STVL 装好后仍存在插件兼容问题 | Ubuntu 20.04 OpenVDB/jemalloc 加载问题；探针不预加载退出 139，预加载成功 | 安装 libjemalloc2，并在 move_base launch 内透传 LD_PRELOAD；systemd 环境变量与 XML 两步都要配置，检查运行进程环境和稳定性 |
| 重启后 move_base 可见，但 RPP 服务仍没有 | 未加载地图/未完成定位，缺少 `map` 到车体 TF，导航尚未完成初始化 | APP 重新加载地图和定位后再验证参数服务；不要把等待 TF 与插件崩溃混为一谈，结合日志、进程和服务判断 |
| 离线包存在却不能安装 | 断电重连后发现此前传输的 6 个 `.deb` 均为 0 字节 | 重传、核对源与板端 SHA256 和包大小，落盘后再安装，不能只检查文件名存在 |
| `apt-get --no-download install /其他目录/*.deb` 仍报无法取得包 | 当天离线 apt 安装方式未正确满足缓存/依赖解析 | 将验证后的完整依赖放入 `/var/cache/apt/archives/`，再按包名安装；见下面命令 |
| 底盘串口持续读取超时 | 当天用户确认未连接底盘，串口名正确并不代表设备在线 | 保留超时与安全保护，接好供电/RS485 后另做硬件验收，不把台架启动成功记为底盘已可作业 |

离线修补示例（源目录按实际修改，必须提前准备完整依赖并核对 SHA256）：

~~~bash
dpkg-deb -f /path/to/validated/package.deb Package Version Architecture
sha256sum /path/to/validated/*.deb
sudo cp /path/to/validated/*.deb /var/cache/apt/archives/
sudo apt-get --yes --no-download --no-install-recommends install \
  ros-noetic-spatio-temporal-voxel-layer libjemalloc2
~~~

速度错误的通用排查顺序：查看 scheduler 的具体失败日志 → 检查地图/定位 TF → 检查
move_base 日志中的插件加载和崩溃 → 检查真实 RPP 命名空间和 `set_parameters` 服务 →
验证空更新应答。本文记录的是当天原因，其他设备还需核对速度值、控制器选择与配置命名空间。
涉及重启必须先停止任务；修复验证通过后，再由操作者按正常流程启动任务。

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

- `docs/` 为本地设计与归档资料目录，整体不纳入版本管理、不随仓库分发；运行与维护说明以根 README 和包内 README 为准。
- `.planning/`、`artifacts/` 和根目录临时计划文件属于被 Git 忽略的本地工作记录，不参与板端部署。
- 地图、任务、区域均建议基于 `map_id` 进行绑定与操作。
- 清缓存指令 `0x051E` 为无参请求，具体清理范围由调度配置决定。
- 本项目是在线联调工程，修改后请重启对应节点验证参数是否已生效。
