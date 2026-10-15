# Grinder 研磨机器人

Grinder 是基于 ROS 1 的研磨机器人控制工程，包含 MID-360 三维建图与定位、二维地图管理、
区域作业路径规划、导航、底盘与磨盘控制，以及 APP / 上位机通信。

本指南按首次部署、日常运行和调试的顺序编写。示例使用普通用户 `neardi`，仓库位于
`/home/neardi/work/Grinder`；部署到其他位置时，统一替换用户和路径。

## 目录

- [1. 系统组成](#1-系统组成)
- [2. 环境与依赖](#2-环境与依赖)
- [3. 设备与应用配置](#3-设备与应用配置)
- [4. 编译与产物检查](#4-编译与产物检查)
- [5. 手工运行与地图操作](#5-手工运行与地图操作)
- [6. systemd 部署与开机自启](#6-systemd-部署与开机自启)
- [7. ESP32 状态灯控制器](#7-esp32-状态灯控制器)
- [8. 调试与验收](#8-调试与验收)
- [9. 日常维护与开发验证](#9-日常维护与开发验证)
- [10. 专项文档](#10-专项文档)

## 1. 系统组成

### 1.1 源码目录

```text
Grinder/
├── catkin_ws/
│   ├── src/grinder_scheduler/       调度、地图、协议、模式管理与状态灯 ROS 节点
│   ├── src/grinder_chassis_driver/  底盘串口、轮速换算、磨盘控制与状态反馈
│   ├── src/localization/           Super-LIO、基础库与回环后端
│   ├── src/mapping/                点云到二维占据栅格
│   ├── src/2-dnavigation-package/  move_base、RPP、TEB 及 Slamware 适配
│   ├── src/grinder_gazebo/         Gazebo 仿真
│   ├── build_grinder_platform.sh   分模块构建入口
│   ├── start_grinder_super_lio_base.sh  常驻服务启动入口
│   └── install_grinder_service.sh systemd 安装入口
├── firmware/status_strip/         ESP32 状态灯固件
├── third_party/path_planner/      MST27 作业路径规划
├── third_party/sl_linka/          协议、生成脚本和上位机工具
├── maps/                          地图资产
└── AGENTS.md                      开发文件定位索引
```

仓库根目录是唯一 Git 仓库。Super-LIO 源码已纳入根仓库，无需初始化子模块。
`build/`、`devel/`、`logs/`、`temp/` 和二进制地图等属于构建或运行数据。

### 1.2 数据与控制链路

```text
MID-360 点云和内置 IMU
  -> Livox 驱动：/livox/lidar、/livox/imu
  -> Super-LIO：同步、重力初始化、预测、去畸变、点云匹配
       ├─ 三维世界点云 -> 可选回环 -> 二维栅格 /map
       └─ IMU 状态按安装外参换算 -> 底盘 /lio/odom、/lio/path、TF

APP / 上位机 -> SL-LinkA TCP / Protobuf -> grinder_scheduler
  -> 地图模式管理、区域编辑、MST27 规划、任务状态机
  -> move_base 目标与路径段 -> RPP 或 TEB -> /cmd_vel
  -> grinder_chassis_driver：任务使能、限幅、左右轮换算
  -> Modbus RTU / RS485 -> 底盘与磨盘

底盘反馈 -> /chassis/status、/odom_wheel -> 调度、导航和诊断
调度与底盘状态 -> 状态灯 ROS 节点 -> USB -> ESP32 -> 外接灯带
```

常驻启动脚本默认选用 RPP；导航控制器支持通过 launch 切换为 TEB。
底盘运动还受 `/chassis/task_enable` 控制；存在 `/cmd_vel` 不代表已经允许执行。
暂停、停止和异常处理应保持关闭任务使能、轮速清零及磨盘安全控制。

### 1.3 坐标系与安装外参

标准模式采用 `lio/ros/legacy_tf: false`，TF 链为：

```text
map -> odom -> base_link -> base_laser_link
```

| 变换 | 发布来源与含义 |
| --- | --- |
| `map -> odom` | 回环后端发布校正；无回环建图和固定地图定位使用单位变换 |
| `odom -> base_link` | Super-LIO 发布底盘运动中心的动态位姿 |
| `base_link -> base_laser_link` | 根据实测安装外参发布固定变换 |

LIO 内部估计 IMU 的三维状态，用于点云和地图计算。定义 `T_A_B` 为将 B 坐标转换到
A 坐标的变换，发布底盘状态时计算：

```text
T_odom_base = T_odom_imu × T_imu_laser × inverse(T_base_laser)
```

`T_imu_laser` 来自 MID-360 内部雷达与 IMU 外参，`T_base_laser` 来自安装标定。
`/lio/odom`、动态 TF 和路径对应运动中心，速度包含转动产生的安装偏移补偿，协方差
同步传播。move_base 使用该底盘位姿，并通过固定外参转换雷达观测，不再次修正底盘中心。
每条 TF 边只由一个发布者负责。历史 IMU 频率调试接口 `/lio/robo/odom` 不作为正式导航输入。

重力对齐由建图和定位初始化代码根据 IMU 平均加速度执行，初始化时车辆应静止。
当前 `kf_align_gravity` 虽被读取，但初始化对齐代码没有以它作为开关判断。
车身整体俯仰改变不等于安装外参改变；刚性安装关系和运动中心定义不变时，倾斜建图与
水平定位原则上可共用地图，仍需验证高度、视野和点云重叠变化后的定位效果。
`base_link` 保留车体三维姿态，不强制变为水平参考系。

## 2. 环境与依赖

### 2.1 平台要求与工作区变量

| 项目 | 要求 |
| --- | --- |
| 目标系统 | Ubuntu 20.04、ROS 1 Noetic |
| 架构 | x86_64 或 RK3588 / aarch64，构建脚本自动识别 |
| Python | 保持 Python 3.8 兼容 |
| 雷达 | MID-360，使用 ROS 1 版 `livox_ros_driver2` |
| 底盘 | 配套 Modbus RTU 控制器与正确的 RS485 接口 |
| 可选指示灯 | ESP32-S3-Zero、GPIO4 外接 WS2812B 灯带 |

Windows 工作区可用于编辑源码和烧录 ESP32。ROS 构建及运行在目标 Linux 环境完成，
不能直接复用另一架构的 `.so`、可执行文件或 Catkin 缓存。

在 Linux 终端设置部署路径，后续命令统一使用这两个变量：

```bash
export GRINDER_BASE_DIR=/home/neardi/work/Grinder
export GRINDER_WS="$GRINDER_BASE_DIR/catkin_ws"
```

这些变量可加入运行用户的 `~/.bashrc`。systemd 的配置文件使用独立的绝对路径，不展开变量。

### 2.2 安装 ROS Noetic

已安装 ROS Noetic 的设备可跳过安装步骤。未安装时，在 Ubuntu 20.04 执行：

```bash
sudo apt update
sudo apt install -y curl gnupg2 lsb-release ca-certificates
curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  | gpg --dearmor \
  | sudo tee /usr/share/keyrings/ros-archive-keyring.gpg >/dev/null
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu focal main" \
  | sudo tee /etc/apt/sources.list.d/ros1.list >/dev/null
sudo apt update
sudo apt install -y ros-noetic-desktop-full
sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator \
  python3-wstool python3-catkin-tools build-essential cmake git pkg-config
```

初始化 rosdep 并加载 ROS 环境：

```bash
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update
source /opt/ros/noetic/setup.bash
```

### 2.3 安装工程依赖

将完整源码放到工作区后执行。GTSAM 和 fast_gicp 由独立安装脚本处理：

```bash
cd "$GRINDER_WS"
source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro noetic \
  --skip-keys="gtsam fast_gicp" -r -y
sudo apt install -y libboost-all-dev libeigen3-dev libmetis-dev libpcl-dev \
  libgoogle-glog-dev libgflags-dev libyaml-cpp-dev \
  python3-yaml python3-serial python3-protobuf python3-numpy python3-scipy \
  python3-matplotlib python3-opencv python3-dev pybind11-dev
```

`grinder_scheduler` 的构建声明仍包含 `slamware_ros_sdk`；即使传感器使用 Super-LIO，
也应保留这项编译依赖，不要删除依赖或改用 ROS 2 包绕过配置错误。

### 2.4 Livox 驱动与工业回环依赖

准备 ROS 1 版 `livox_ros_driver2`。板端可将其放在
`catkin_ws/src/drivers/ws_livox/src/livox_ros_driver2/`，也可使用独立驱动工作区。
确认部署驱动提供 `msg_MID360s.launch` 和相应设备 JSON 配置。
独立工作区构建完成后，使用其实际 `devel/setup.bash` 加载环境。

工业回环依赖通过工程脚本安装：

```bash
cd "$GRINDER_WS"
JOBS=2 bash ./install_super_lio_loop_deps.sh
```

脚本固定 GTSAM 4.2.2 与 fast_gicp 提交，默认安装到 `/usr/local`，不替换 ROS 的 PCL。
工业图优化要求 GTSAM；缺少 fast_gicp 时使用较慢的 PCL-GICP，并在构建时提示。
`mapping` 构建 profile 强制要求工业后端依赖，与运行时是否开启回环无关。

## 3. 设备与应用配置

### 3.1 配置文件职责

| 文件 | 用途 |
| --- | --- |
| [scheduler.yaml](catkin_ws/src/grinder_scheduler/config/scheduler.yaml) | 调度公共默认配置、规划、机器人尺寸与标定默认值 |
| [scheduler.local.example.yaml](catkin_ws/src/grinder_scheduler/config/scheduler.local.example.yaml) | 设备本地配置示例 |
| [super_lio.yaml](catkin_ws/src/grinder_scheduler/config/super_lio.yaml) | 模式生命周期、地图根目录、定位质量门限与回环开关 |
| [chassis_driver.yaml](catkin_ws/src/grinder_chassis_driver/config/chassis_driver.yaml) | 串口、波特率、轮径轮距、方向、超时和磨盘能力 |
| [livox_360.yaml](catkin_ws/src/localization/Super-LIO/src/super_lio/config/livox_360.yaml) | LIO 建图、雷达与 IMU 参数 |
| [relocation.yaml](catkin_ws/src/localization/Super-LIO/src/super_lio/config/relocation.yaml) | 三维地图定位参数 |
| [status_indicator.yaml](catkin_ws/src/grinder_scheduler/config/status_indicator.yaml) | 灯带通信和状态超时配置 |

公共配置不保存设备凭据。首次部署时复制本地配置示例，已有文件则直接编辑：

```bash
cd "$GRINDER_BASE_DIR"
if [ ! -f catkin_ws/src/grinder_scheduler/config/scheduler.local.yaml ]; then
  cp catkin_ws/src/grinder_scheduler/config/scheduler.local.example.yaml \
    catkin_ws/src/grinder_scheduler/config/scheduler.local.yaml
fi
```

`scheduler.local.yaml` 被 Git 忽略，不随 Catkin 安装分发。平台上传、MQTT 和外部视频推流
按需在本地配置中显式开启；凭据也可使用对应环境变量提供。
真实设备启动脚本会发现该文件，配置顺序为公共默认、Super-LIO 配置、本地覆盖、
显式 launch 参数。可用 `GRINDER_SCHEDULER_LOCAL_CONFIG` 指向另一份绝对路径配置；
设置为空字符串表示不加载本地覆盖。直接 roslaunch 时使用
`scheduler_local_config:=/absolute/path/device.yaml`。

### 3.2 雷达网络和底盘串口

雷达直连网口与上联网口应使用可区分的网段。检查网卡、路由与驱动配置中的
`host_ip`、雷达 IP、UDP 端口是否一致：

```bash
ip -br addr
ip route
# 将下面变量替换为实际雷达 IP，再检查对应出口网卡
LIVOX_IP=192.168.1.100
ip route get "$LIVOX_IP"
```

底盘默认配置为 `/dev/ttyS0`、19200 波特率、Modbus 地址 1；部署前必须核对真实设备。
ESP32 的 USB 串口与底盘 RS485 串口是两个独立设备。串口访问权限可通过 `dialout` 组提供：

```bash
sudo usermod -aG dialout "$USER"
```

更改用户组后重新登录。单独启动底盘节点会访问真实控制器，节点默认先输出零速，
任务使能默认关闭；调试时保留超时和急停保护。

### 3.3 安装标定、地图与导航参数

APP 保存的底盘—雷达标定由调度器传给模式管理器，建图和定位共用同一组参数：

| 调度参数 | 模式启动 / LIO 参数末尾名称 | 单位 |
| --- | --- | --- |
| `base_laser_x/y/z` | `base_to_laser_x/y/z` | 米 |
| `base_laser_roll_deg` | `base_to_laser_roll` | 调度为度，启动 / LIO 为弧度 |
| `base_laser_pitch_deg` | `base_to_laser_pitch` | 调度为度，启动 / LIO 为弧度 |
| `base_laser_yaw_deg` | `base_to_laser_yaw` | 调度为度，启动 / LIO 为弧度 |

launch 同时用这组参数发布固定 TF 并配置 `/lio/extrinsic/base_to_laser_*`。
例如雷达在运动中心前方 0.2 m，X 应为 `0.2`；Y、Z 和安装角仍按实测填写。
车身对地面的倾角不是安装外参。修改标定后通过正常流程停止并重启地图模式，使
固定 TF 和 LIO 同时加载新值。手工启动 LIO 时必须自己传入同一组标定参数。

`super_lio_map_root` 指定托管地图目录。`super_lio_loop_closure` 默认 `false`，
需要工业回环时在设备配置中显式设为 `true`；手工建图 launch 默认开启工业回环。
`super_lio_autosave` 默认 `false`，由 APP 明确保存地图。

导航全局与局部 costmap 使用 `base_link`。RPP 控制器使用底盘反馈 `/odom_wheel`；
LIO 的 `/lio/odom` 提供底盘定位输入。参数位于导航包的 `cfg/diff_drive/`，
修改前先确认启动的是 RPP 还是 TEB。

## 4. 编译与产物检查

### 4.1 首次构建

在 `catkin_ws` 根目录运行构建，不进入 Super-LIO 源码目录单独 make。
驱动源码在同一工作区时，先构建 ROS 1 Livox 包：

```bash
cd "$GRINDER_WS"
source /opt/ros/noetic/setup.bash
catkin_make -DCATKIN_WHITELIST_PACKAGES=livox_ros_driver2 \
  -DCMAKE_BUILD_TYPE=Release -j2 -l2
source devel/setup.bash
```

若使用独立 Livox 工作区，先完成该工作区构建并 source 它的 `devel/setup.bash`，
跳过上面的同工作区驱动构建。随后构建定位建图链和平台主包：

```bash
cd "$GRINDER_WS"
CATKIN_JOBS=2 CATKIN_LOAD=2 PROFILE=mapping bash ./build_grinder_platform.sh
CATKIN_JOBS=2 CATKIN_LOAD=2 PROFILE=full bash ./build_grinder_platform.sh
source devel/setup.bash
```

**`full` 不包含 `mapping`，完整 MID-360 系统需要分别构建这两个 profile。**
RK3588 建议从两任务并行开始，避免内存不足。脚本会检查工作区路径和架构缓存，
不匹配时按配置备份处理；正常增量构建不需要删除 `build`、`devel`。

### 4.2 构建范围选择

| `PROFILE` | 范围 |
| --- | --- |
| `mapping` | Super-LIO、回环、二维栅格及其依赖，强制工业后端依赖检查 |
| `full` | 底盘、Slamware 适配、调度和导航主包，不包含 mapping |
| `runtime` | 底盘、Slamware 适配、调度 |
| `scheduler` | 调度包，要求依赖包已可用 |
| `chassis` | 底盘驱动包 |
| `nav` | move_base、RPP / TEB 等导航包 |
| `aurora` | Slamware 适配包 |
| `sim` | Gazebo 仿真、调度及对应导航依赖 |

例如仅修改 LIO 和调度时，分别执行 `PROFILE=mapping` 与 `PROFILE=scheduler`。
修改 C++、消息、服务或 CMake 后重新构建；普通参数文件修改后重新启动对应节点。
更新 systemd 管理脚本后还需重新执行服务安装脚本，不能只更新工作区里的副本。

### 4.3 MST27 C++ 规划内核

规划内核是独立 Python 扩展，应使用板端 Python 和架构显式编译：

```bash
cd "$GRINDER_BASE_DIR/third_party/path_planner/mst27"
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DPython3_EXECUTABLE="$(command -v python3)"
cmake --build build --parallel 2
python3 -c "import numpy, scipy, matplotlib, cv2; import _mst27_cpp; print('MST27 dependencies: OK')"
```

产物为 `mst27.py` 同目录下的 `_mst27_cpp*.so`。日志中的 C++ 后端提示表示使用加速内核；
Python fallback 表示未加载扩展。更换 CPU 架构或 Python ABI 后重新编译扩展。

### 4.4 检查构建产物与环境

每个新运行终端都先加载 ROS 和工作区环境；构建完成后可将这些 source 命令加入 `~/.bashrc`：

```bash
source /opt/ros/noetic/setup.bash
source "$GRINDER_WS/devel/setup.bash"
rospack find super_lio
rospack find livox_ros_driver2
ls -lh "$GRINDER_WS/devel/lib/super_lio/super_lio_node"
ls -lh "$GRINDER_WS/devel/lib/super_lio/relocation_node"
ls -lh "$GRINDER_WS/devel/lib/super_lio_loop/super_lio_loop_node"
ls -lh "$GRINDER_WS/devel/lib/super_lio_loop/super_lio_graph_backend_node"
```

`rospack find` 应指向预期部署目录。工业构建应同时具有轻量节点和工业图优化节点；
仅存在轻量节点不能视为工业后端构建完成。

## 5. 手工运行与地图操作

### 5.1 启动常驻服务

先确认设备配置、串口、雷达网络和构建产物正常。手工启动与 systemd 部署二选一，
不要让两套流程重复启动 Livox、底盘、导航或灯带节点：

```bash
cd "$GRINDER_WS"
source /opt/ros/noetic/setup.bash
source devel/setup.bash
bash ./start_grinder_super_lio_base.sh
```

该脚本启动或复用 ROS master，启动 Livox、底盘、RPP、调度器和模式管理器。
地图模式初始为 `IDLE`，不会自动建图或定位，更不会自动执行任务。
脚本监督子进程并记录日志，同一工作区通过锁拒绝重复启动。

| 环境变量 | 用途 / 默认值 |
| --- | --- |
| `START_LIVOX` | 是否启动 Livox，默认 `1` |
| `START_CHASSIS` | 是否启动真实底盘驱动，默认 `1` |
| `START_NAV` | 是否启动 RPP 导航，默认 `1` |
| `START_INDICATOR` | 是否启动灯带通信服务，默认 `0` |
| `LIVOX_SETUP` | 独立 Livox 工作区环境路径，空时尝试发现驱动 |
| `NAV_MAP_YAML` | 导航地图路径，按设备部署填写 |
| `SUPER_LIO_MAP_ROOT` | 托管地图根目录，默认仓库 `maps/` |

首次只联调传感器和 APP、不接底盘时，可关闭底盘与导航；此模式不用于执行真实任务：

```bash
cd "$GRINDER_WS"
START_CHASSIS=0 START_NAV=0 bash ./start_grinder_super_lio_base.sh
```

独立驱动工作区通过 `LIVOX_SETUP=/absolute/path/devel/setup.bash` 传入。
启动后先检查服务状态和协议监听：

```bash
rosservice call /super_lio_mode/get_status
rosnode info /grinder_scheduler
ss -lntp | grep 8002
```

### 5.2 APP 连接与地图生命周期

APP / 上位机通过 TCP 连接工控机 IP 和 SL-LinkA 端口 `8002`。地图与任务操作由调度器
分发给模式管理器。推荐流程：

1. 启动常驻服务，确认地图模式为 `IDLE`。
2. 请求建图，模式进入 `MAPPING`，按现场流程采集环境。
3. 在 `MAPPING` 仍活动时提交 `MapSaveRequest`，不要先切换定位。
4. 保存成功取得正式 `map_id` 和 `map_revision`；建图停止，定位不会自动启动。
5. 选择已保存地图，启动定位，并提交身份匹配的初始位姿。
6. 轮询定位状态，只有达到 `READY` 后才开始作业任务。

`LIVE_MAP` 是实时建图源保留 ID，不是正式资产目录名。保存后每张地图独立存储：

```text
maps/<map_id>/
├── loc_map.pcd       三维定位点云
├── plan_map.pcd      三维规划点云
├── map.yaml         二维地图元信息
├── map.pgm          二维占据栅格
└── map_info.json    地图身份、版本和文件清单
```

保存通过 `/lio/get_3dmap` 和栅格保存服务获取资产，校验后原子提交目录。
保存失败时建图保持运行，可排查后重试。开启回环时，三维保存入口仍属于 Super-LIO，
应核对保存后的三维 PCD 与校正二维栅格坐标一致，不能只检查实时二维地图。

定位使用 `map_server` 发布二维地图、`relocation_node` 加载三维 PCD。
`/initialpose` 表示地图坐标中的底盘中心位置和航向，结合重力及安装外参换算为 IMU 配准初值。
输入 Z 是地图坐标高度，不一定等于离地高度；APP 二维请求目前发布 Z 为零，
需要明确三维高度时使用模式管理器的三维 Pose 服务。定位通过 NDT / ICP 和后续 LIO 更新，
经新鲜度、配准质量、协方差和稳定帧检查后进入 `READY`。

### 5.3 按组件调试

只在常驻服务和托管地图模式未运行对应组件时单独启动。每个终端先 source 环境，
需要时先运行 `roscore`。下表命令分别在独立终端执行：

| 组件 | 启动命令 |
| --- | --- |
| Livox | `roslaunch livox_ros_driver2 msg_MID360s.launch` |
| 底盘 | `roslaunch grinder_chassis_driver chassis_driver.launch` |
| RPP 导航 | `roslaunch teb_local_planner_tutorials robot_diff_drive.launch local_planner:=rpp` |
| TEB 导航 | `roslaunch teb_local_planner_tutorials robot_diff_drive.launch local_planner:=teb` |
| 调度与模式管理器 | `roslaunch grinder_scheduler scheduler.launch odom_topic:=/lio/odom map_topic:=/map` |

直接启动调度 launch 时，按需增加 `scheduler_local_config` 和 `super_lio_map_root` 参数。
Livox 的 bag 录制可用 `enable_lidar_bag:=false enable_imu_bag:=false` 关闭，
这不会关闭 ROS 日志。

需要绕过模式管理器单独调试建图或定位时，先在各终端设置相同的实测安装参数。
以下零值仅为占位，不能直接当作设备标定：

```bash
BASE_TO_LASER_X=0.0
BASE_TO_LASER_Y=0.0
BASE_TO_LASER_Z=0.0
BASE_TO_LASER_ROLL=0.0
BASE_TO_LASER_PITCH=0.0
BASE_TO_LASER_YAW=0.0
```

单独建图：

```bash
roslaunch cloud_to_occupancy_grid mid360_mapping.launch \
  rviz:=true save_3d_map:=true loop_closure:=true loop_backend:=industrial \
  base_to_laser_x:="$BASE_TO_LASER_X" base_to_laser_y:="$BASE_TO_LASER_Y" \
  base_to_laser_z:="$BASE_TO_LASER_Z" base_to_laser_roll:="$BASE_TO_LASER_ROLL" \
  base_to_laser_pitch:="$BASE_TO_LASER_PITCH" base_to_laser_yaw:="$BASE_TO_LASER_YAW"
```

关闭回环时使用 `loop_closure:=false`，栅格输入改为 `/lio/map_cloud`；开启回环时为
`/lio/loop_cloud`，工业后端负责 `map -> odom` 并支持历史栅格重投影。
`loop_backend:=lightweight` 用于诊断，其二维位姿图不替代工业后端。

单独定位时，将 `MAP_DIR` 替换为实际资产目录。二维地图和三维定位分别在两个终端启动：

```bash
MAP_DIR="$GRINDER_BASE_DIR/maps/实际地图ID"
rosrun map_server map_server "$MAP_DIR/map.yaml"
```

```bash
MAP_DIR="$GRINDER_BASE_DIR/maps/实际地图ID"
roslaunch super_lio relocation.launch rviz:=true map_dir:="$MAP_DIR" map_name:=loc_map.pcd \
  base_to_laser_x:="$BASE_TO_LASER_X" base_to_laser_y:="$BASE_TO_LASER_Y" \
  base_to_laser_z:="$BASE_TO_LASER_Z" base_to_laser_roll:="$BASE_TO_LASER_ROLL" \
  base_to_laser_pitch:="$BASE_TO_LASER_PITCH" base_to_laser_yaw:="$BASE_TO_LASER_YAW"
```

在 RViz 使用 `2D Pose Estimate` 提交初值。建图节点与定位节点互斥，禁止同时运行。
手工产生的地图文件不自动具有托管资产身份，正式地图保存优先使用 APP 生命周期。

## 6. systemd 部署与开机自启

<a id="12-板端开机自启主系统与灯带统一开关"></a>

### 6.1 安装服务

完成构建和手工联调后，先停止任务并正常退出原启动脚本及独立节点，再安装服务：

```bash
cd "$GRINDER_WS"
sudo bash ./install_grinder_service.sh neardi
sudo nano /etc/grinder/startup.conf
```

首次安装默认关闭自启且不启动节点；重新安装保留已有配置。服务以指定普通用户运行。
若提示 `/etc/grinder` 不存在，先完成安装；若缺少 `scripts/grinder-service`，
先同步完整服务脚本。管理命令是安装副本，更新其源码后需要重新执行安装脚本。

### 6.2 配置启动参数

`/etc/grinder/startup.conf` 是 `KEY=VALUE` 文件，不是 YAML，不展开 `$HOME`、`~`
或 shell 命令。用户和文件路径须真实存在；`NAV_MAP_YAML` 必须指向有效地图文件。
没有正式地图时，先按手工流程建图保存，再配置 systemd。未接 ESP32 的基本配置示例：

```ini
AUTOSTART=0
RUN_USER=neardi
WORKSPACE=/home/neardi/work/Grinder/catkin_ws
LIVOX_SETUP=''
NAV_MAP_YAML=/home/neardi/work/Grinder/maps/实际地图ID/map.yaml
INDICATOR_USB_SERIAL_NUMBER=''
INDICATOR_DEVICE=''
START_INDICATOR=0
```

| 字段 | 说明 |
| --- | --- |
| `AUTOSTART` | `1` 下次开机启动整套服务，`0` 不自启；不等于自动执行任务 |
| `RUN_USER` | 已存在的普通运行用户 |
| `WORKSPACE` | 已构建的 Catkin 工作区绝对路径 |
| `LIVOX_SETUP` | 独立 Livox 环境路径；驱动已可发现时可留空 |
| `NAV_MAP_YAML` | 存在的二维地图 YAML 文件 |
| `START_INDICATOR` | `1` 启动板端灯带通信服务，`0` 不启动；不控制 ESP32 USB 供电 |
| `INDICATOR_USB_SERIAL_NUMBER` | ESP32 的实际 USB 序列号 |
| `INDICATOR_DEVICE` | 可替代序列号的 `/dev/serial/by-id/` 稳定路径 |

启用 ESP32 时设置 `START_INDICATOR=1`，两种 USB 身份仅填写一种。
只有 ESP32、没有外接灯带也可开启通信测试。没有 ESP32 时保持 `START_INDICATOR=0`。
旧配置缺省该字段时保留历史默认 `1`，维护时建议显式填写。

### 6.3 管理命令

```bash
sudo grinder-service apply
sudo grinder-service start
grinder-service status
```

确认服务持续运行后按需执行 `sudo grinder-service enable` 开启自启。
地图模式仍由 APP 显式操作，服务启动不表示定位就绪。

| 操作 | 命令 | 效果 |
| --- | --- | --- |
| 应用配置 | `sudo grinder-service apply` | 更新服务定义与自启开关，不重启当前节点 |
| 开启自启 | `sudo grinder-service enable` | 保存开机启动设置，当前节点不变 |
| 关闭自启 | `sudo grinder-service disable` | 关闭下次开机启动，当前节点不变 |
| 启动节点 | `sudo grinder-service start` | 启动当前服务 |
| 停止节点 | `sudo grinder-service stop` | 停止当前服务，不改变自启设置 |
| 重启节点 | `sudo grinder-service restart` | 重新启动整套节点，不重启工控机 |
| 查看状态 | `grinder-service status` | 查看自启与运行状态 |
| 查看日志 | `sudo grinder-service logs` | 查看服务日志 |

修改配置后先 `apply`，需要更新运行中参数时，在停止任务后 `restart`。
服务失败或受监督子进程退出会清理整套子进程，默认不自动重启整套服务。
同一工作区不要再单独启动另一套导航或灯带节点。

已有 PM2 管理 ROS master 的设备，服务会按已有用户 PM2 服务排序，避免重复启动 master；
不会自动启用或接管 PM2。如果 PM2 在运行期间重启 master，需要协调两个管理流程。
执行工控机重启前先停止任务，再确认自启配置。重新登录后检查 `enabled` 与
`active (running)`，并继续完成定位和设备状态检查。

## 7. ESP32 状态灯控制器

### 7.1 固件与接线

固件在 [firmware/status_strip](firmware/status_strip/README.md)，适配微雪 ESP32-S3-Zero
4 MB Flash。Type-C USB 同时提供供电和通信；GPIO4 经电平转换驱动外接 3 颗 WS2812B。
固件不驱动 GPIO21 板载 RGB 灯，因此板载灯不亮不能用于判断程序是否运行。
完整接线、电源和缓冲芯片引脚以固件 README 为准。

在 Windows PowerShell 编译并烧录，端口替换为实际 ESP32 COM 号：

```powershell
python -m pip install platformio==6.1.18
cd 'E:\CODE\C++\Grinder\firmware\status_strip'
python -m platformio device list
python -m platformio run -e esp32-s3-zero
python -m platformio run -e esp32-s3-zero -t upload --upload-port COM9
```

首次烧录连接失败时，按住 BOOT 进入下载模式，按 RESET 或重新连接 USB 后重试。
烧录端口必须是 ESP32，不能使用底盘控制串口。上电黄灯和其他灯效均指外接灯带。

### 7.2 板端识别与通信检查

```bash
python3 -m serial.tools.list_ports -v
```

将实际 USB 序列号填到启动配置。没有运行统一服务时，也可单独启动：

```bash
roslaunch grinder_scheduler status_indicator.launch \
  backend:=serial usb_serial_number:=实际USB序列号
```

单独 launch 默认 `mock`，只有指定 `serial` 并提供设备身份才访问 USB。
统一服务默认使用真实串口后端，自动提供 `dialout` 补充组权限。
检查：

```bash
rostopic echo -n 3 /indicator/state
```

`backend: serial`、`acknowledged: true`、`error: ""` 表示真实 USB 协议通信成功，
没有灯带也能完成这项验证。ACK 不证明灯珠发光或机器已安全停车。

### 7.3 状态含义

| 状态 | 外接灯带效果 |
| --- | --- |
| 启动、状态来源不完整 | 黄色常亮 |
| 待机、准备、正常停止、规划 | 绿色常亮 |
| 自动任务 | 蓝色呼吸 |
| 人为暂停 | 蓝色慢闪 |
| 碰撞风险暂停 | 黄色慢闪 |
| 手动控制或非自动磨盘活动 | 青色常亮 |
| 完成 | 绿色闪三次后常亮 |
| `FAULT` | 红色慢闪 |
| 急停提示 | 红色快闪，优先于其他状态 |

机器人状态与 USB 通信是两项独立检查。底盘 `connected: false`、调度 `ERROR` 或
就绪后状态来源超时等情况会产生 `FAULT`，即使 ACK 完全正常。
未接底盘的联调环境可能显示该状态，没有灯带本身不会触发故障。

急停灯光提示复位接口为 `/indicator/clear_emergency`，只清除显示记录，
不复位真实急停、不清除底盘故障、不使能运动。接口和持久化机制见固件 README。

## 8. 调试与验收

### 8.1 检查顺序

按服务、传感器、地图定位、TF、导航、底盘、外设逐层检查。
`rostopic hz`、`tf_echo` 和持续日志命令用 `Ctrl+C` 结束。
以下检查只读取状态；真实运动和磨盘验收应在现场安全条件下通过既有任务接口执行。

**服务与节点：**

```bash
grinder-service status
rosnode list
rosservice call /super_lio_mode/get_status
```

刚启动地图状态为 `IDLE` 正常。经 APP 选择地图和初始定位后，期望 `state: READY`、
`localization_ready: true`、`registration_quality_valid: true`。

**传感器与地图：**

```bash
rostopic hz /livox/lidar
rostopic hz /livox/imu
rostopic hz /lio/odom
rostopic echo -n 1 /map/info
```

只有启动建图或定位后才应期待 LIO 输出；托管定位还需要有效初始位姿。
工业回环建图时额外检查 `/lio/loop_cloud` 和 `/super_lio_loop/status`。

**安装参数与 TF：**

```bash
rosparam get /lio/ros/legacy_tf
rosparam get /lio/extrinsic
rostopic echo -n 1 /lio/odom
rosrun tf tf_echo map base_link
rosrun tf tf_echo base_link base_laser_link
```

标准模式 `legacy_tf` 为 `false`，里程计帧为 `odom`、子帧为 `base_link`。
LIO 的安装参数与固定 TF 应对应同一实测结果，角度注意度与弧度转换。
固定 TF 时间显示 `0.000` 正常；帧名正确不等于数值已标定，需同时核对位置与地图点云。
固定地图定位时 map 与 odom 重合；有回环时不能把 odom 位姿直接当成 map 位姿。

**底盘与调度：**

```bash
rostopic echo -n 1 /chassis/status
rostopic echo -n 1 /scheduler/status
rostopic echo -n 1 /chassis/task_enable
rostopic hz /odom_wheel
rostopic info /cmd_vel
```

底盘连接以 `connected` 为准，`enabled: true` 不表示串口通信成功。
导航还可检查 `/move_base/global_costmap/costmap`、`/move_base/local_costmap/costmap`
和 `/grinder/navigation/active_segment_plan`。

### 8.2 日志位置

```bash
sudo journalctl -u grinder.service -b -n 100 --no-pager
```

常驻脚本各节点日志在 `$GRINDER_WS/logs/super_lio_base_*`，ROS 日志在运行用户的
`~/.ros/log/`。定位匹配和滤波迭代诊断可查看：

```bash
rostopic echo /super_lio/registration_diagnostics
```

服务显示运行只代表进程存活；地图、定位、视频和底盘状态应分别验收。

### 8.3 常见问题

| 现象 | 检查与处理 |
| --- | --- |
| 找不到 `livox_ros_driver2` | 检查 ROS 1 驱动是否已构建；独立工作区是否已 source；服务的 `LIVOX_SETUP` 是否正确 |
| 缺少 GTSAM 或工业节点 | 执行工业依赖安装脚本，再以 `PROFILE=mapping` 构建；检查 `GTSAMConfig.cmake` 搜索路径 |
| 编译后仍运行旧代码 | 检查 `rospack find`、当前 source 环境和服务 `WORKSPACE`，停止任务后重启对应进程 |
| 雷达端口占用 | 检查是否存在另一套 Livox、ROS2 驱动或手工节点；启动脚本会预检 UDP 端口 |
| 模式一直 `LOCALIZING` | 核对地图身份、初始位姿、传感器频率、配准诊断、协方差与 TF；不要直接放宽就绪条件 |
| 底盘 `Serial read timeout` | 核对底盘电源、串口、RS485 接线、波特率、校验位与从站地址 |
| 有速度但底盘不走 | 检查任务使能、底盘连接、急停和超时；不要绕过使能保护测试 |
| ESP32 不亮 | 区分板载灯与 GPIO4 外接灯带；先查 USB ACK，再检查灯带供电和接线 |
| 灯带 `FAULT` 但 ACK 正常 | 查 `/chassis/status`、`/scheduler/status` 和状态来源新鲜度；USB 不是故障来源 |
| `Failed to load Python extension for LZ4 support` | 提示缺少可选压缩支持；与串口 ACK、底盘连通性分开判断 |
| 视频 MPP 编码错误 | 查调度 / mediamtx 日志、编码器能力和设备本地视频配置；节点存活不等于视频可用 |
| 白名单残留导致漏包 | 查 `build/CMakeCache.txt` 的 `CATKIN_WHITELIST_PACKAGES`，优先用对应构建 profile |

## 9. 日常维护与开发验证

### 9.1 更新与重新构建

1. 停止任务，通过正常流程停止服务或手工节点。
2. 同步源码，保留设备本地配置、标定、地图和必要运行记录。
3. 根据修改范围选择构建 profile；不跨架构复制二进制产物。
4. 更新管理脚本时重新安装服务；修改启动配置后执行 `apply`。
5. 重启节点，重新完成地图选择、定位及状态验收，再允许作业。

修改 `msg/`、`srv/`、`package.xml` 或 CMake 必须 Catkin 构建。
修改 `sl_link.proto` 后从仓库根目录重新生成相关 SDK，不只改生成文件：

```bash
cd "$GRINDER_BASE_DIR"
bash third_party/sl_linka/scripts/generate_proto.sh all
```

仅需部分目标时，将 `all` 替换为 `python` 或 `kotlin`。上位机工具入口为
`third_party/sl_linka/sdk/python/tools/sl_linka_pyqt_debugger.py`，连接工控机 IP 和端口 `8002`。

### 9.2 离线检查

在仓库根目录执行，不连接硬件：

```bash
cd "$GRINDER_BASE_DIR"
PYTHONPATH=catkin_ws/src/grinder_chassis_driver/src python3 -m unittest discover \
  -s catkin_ws/src/grinder_chassis_driver/test -p 'test_*.py'
PYTHONPATH=catkin_ws/src/grinder_scheduler/src python3 -m unittest discover \
  -s catkin_ws/src/grinder_scheduler/test -p 'test_super_lio*.py'
python3 catkin_ws/scripts/test_grinder_service.py -v
python3 -m compileall -q catkin_ws/src/grinder_scheduler/src/grinder_scheduler \
  catkin_ws/src/grinder_chassis_driver/src/grinder_chassis_driver
```

LIO 位姿 / 速度 / 协方差数学回归、灯带协议互通、地图质量与仿真测试见对应专项文档。
离线单测和构建不能替代录包回放及实机验收。

## 10. 专项文档

| 文档 | 内容 |
| --- | --- |
| [开发导航](AGENTS.md) | 按需求定位源码、修改边界与最小验证 |
| [平台构建与启动](catkin_ws/PLATFORM_BUILD_RUN.md) | 架构、profile、启动脚本和平台参数 |
| [Super-LIO 集成说明](catkin_ws/src/localization/Super-LIO/UPSTREAM.md) | 上游版本、外参换算、迭代诊断与数学回归 |
| [二维建图](catkin_ws/src/mapping/cloud_to_occupancy_grid/README.md) | 栅格参数、保存与回环重投影 |
| [回环后端](catkin_ws/src/localization/super_lio_loop/README.md) | 工业 / 轻量后端、依赖与参数 |
| [状态灯固件与部署](firmware/status_strip/README.md) | 接线、烧录、USB 协议、灯效及验收 |
| [SL-LinkA 协议](third_party/sl_linka/sl-link.md) | 地图、任务、重定位和设置接口 |
| [Gazebo 仿真](catkin_ws/src/grinder_gazebo/README.md) | 仿真模型、APP 联调和运行方式 |
