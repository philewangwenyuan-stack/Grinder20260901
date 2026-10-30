# 平台化编译与启动

本工程支持在 `x86_64` 和 `aarch64(RK3588)` 上按平台自动编译与启动。

控制盒 USB 状态灯带默认关闭。ESP32-S3-Zero + 10 cm WS2812B 的接线、
烧录、模拟验证及启用参数见 [`firmware/status_strip/README.md`](../firmware/status_strip/README.md)。
新增灯带入口后需重新构建调度运行包；有 Aurora 链接库时可执行 `PROFILE=runtime ./build_grinder_platform.sh`。
纯 MID-360S 板端使用 [根 README 的选择性构建](../README.md#4-正式-release-编译)，仍需生成 Aurora ROS 消息。

首次部署请先按 [根 README 安装与编译](../README.md#ros-noetic-安装工业级编译与正式启动) 完成依赖安装。
导航运行必须安装 `ros-noetic-spatio-temporal-voxel-layer` 和 `libjemalloc2`；
Ubuntu 20.04 的 move_base 预加载配置见 [STVL/jemalloc](../README.md#deployment-stvl)。
当天完整版本、热点 `.3`、雷达 `.199`、设备配置、自启覆盖和独立踩坑段见
[2026-10-22 RK3588 部署实录](../README.md#deployment-20261022)。

MID-360 + Super-LIO 主系统与灯带支持统一 systemd 开机自启：在任务停止且已有启动脚本退出后，
执行 `sudo bash ./install_grinder_service.sh <运行用户>`，编辑 `/etc/grinder/startup.conf`，
再执行 `sudo grinder-service apply`。首次安装默认关闭，重装保留配置。
`enable` / `disable` 只切换下次开机；`start` / `stop` / `restart` 操作当前节点。
没有 ESP32 时可在配置中设置 `START_INDICATOR=0` 并留空 USB 身份，先验证主系统自启。
完整配置、安装、日志与验收步骤见 [根 README 的板端开机自启说明](../README.md#12-板端开机自启主系统与灯带统一开关)。

新克隆已包含 Super-LIO 源码和不含凭据的 `grinder_scheduler/config/scheduler.yaml`。
Super-LIO 无子模块拉取步骤，上游版本见
`src/localization/Super-LIO/UPSTREAM.md`。
`mapping` / `sim` 首次构建会补齐本地 Livox 依赖：driver2 固定提交
`13eb05e4e6dd7a765b934d0c5fd6236676a57b49`、SDK2 `v1.3.1`，下载后校验
SHA-256。源码位于被忽略的 `src/drivers/ws_livox/src/`，压缩包及 SDK 静态库
缓存在仓库 `temp/`；已有文件（包括设备配置）保留，不初始化子仓库，也不启动驱动。
依赖齐全后重复构建不联网；可单独执行 `bash scripts/build_livox_ros1.sh` 准备。
平台构建显式传 `ROS_EDITION=ROS1`、SDK 静态库及头文件路径，不要求系统安装 SDK。
设备服务配置请从 `src/grinder_scheduler/config/scheduler.local.example.yaml`
复制为被忽略的 `scheduler.local.yaml`；两个真实启动脚本自动发现该文件，
也可通过 `GRINDER_SCHEDULER_LOCAL_CONFIG` 指定绝对路径（显式为空表示不用本地覆盖）。
配置预检需要 Python 3 的 PyYAML（Ubuntu 包 `python3-yaml`）。
仿真脚本默认只用受控配置，平台上传、MQTT 和远端推流均关闭。
直接 roslaunch 可传 `scheduler_local_config:=/absolute/path/device.yaml`。

## 1. 编译（自动识别平台）

脚本：

```bash
cd /home/chersxir/work/Grinder/catkin_ws
./build_grinder_platform.sh
```

默认行为：

- 自动识别 `uname -m`
- 自动处理编译 profile（默认 `full`）
- 若发现 `build/devel/install` 为其它架构缓存，自动备份并重建

可选 profile：

```bash
PROFILE=runtime   ./build_grinder_platform.sh   # 底盘+Aurora+调度，需要 Aurora 链接库
PROFILE=nav       ./build_grinder_platform.sh   # 导航相关
PROFILE=aurora    ./build_grinder_platform.sh   # slamware_ros_sdk
PROFILE=scheduler ./build_grinder_platform.sh
PROFILE=chassis   ./build_grinder_platform.sh
PROFILE=sim       ./build_grinder_platform.sh   # Gazebo + 调度 + RPP/move_base 仿真链路
PROFILE=mapping   ./build_grinder_platform.sh   # Super-LIO + 固定内存二维栅格建图
```

内部轮速融合新增 C++ 代码及底盘消息依赖后，目标 Linux 需构建 `PROFILE=mapping`
和 `PROFILE=runtime`。两种模式融合默认关闭；受控验证后在调度包
`config/super_lio.yaml` 分别开启 `super_lio_wheel_fusion_mapping_enabled` 或
`super_lio_wheel_fusion_localization_enabled`，重启对应模式生效。
线速度和角速度另有独立开关（默认 true/false）；观测源为 `/chassis/wheel_twist`。
直接 roslaunch 可传 `wheel_fusion_enabled:=true wheel_fuse_angular_velocity:=true`；
这只应在完成时间与噪声标定后使用。检查 `/super_lio/wheel_fusion_diagnostics`，
不通过降低定位质量门槛掩盖轮速打滑或雷达退化。

`mapping` profile 默认要求工业回环后端。首次在 Ubuntu 20.04 / RK3588 上构建时先安装固定版本依赖：

```bash
cd /home/neardi/work/Grinder/catkin_ws
JOBS=2 bash ./install_super_lio_loop_deps.sh
PROFILE=mapping ./build_grinder_platform.sh
```

建图默认启动 GTSAM iSAM2 + Scan Context + FastVGICP 后端：

```bash
roslaunch cloud_to_occupancy_grid mid360_mapping.launch loop_backend:=industrial
```

`loop_backend:=lightweight` 仅用于依赖故障时的台架诊断，不作为生产配置。工业后端接受回环后，二维栅格节点会通过关键帧服务在后台重投影历史地图并原子替换，避免旧栅格形成双墙。

可选参数：

```bash
CATKIN_JOBS=4 CATKIN_LOAD=4 ./build_grinder_platform.sh
SKIP_G2O=1 ./build_grinder_platform.sh
CLEAN_ON_ARCH_CHANGE=0 ./build_grinder_platform.sh
CLEAN_ON_PATH_CHANGE=0 ./build_grinder_platform.sh
```

## 2. 启动（自动选择 mediamtx 二进制）

MID-360S + Super-LIO 主链路使用 `./start_grinder_super_lio_base.sh`，
显式设置 `LIVOX_SETUP` 和 `NAV_MAP_YAML`；统一开机自启见上文链接。
下列 `start_grinder_stack.sh` 示例用于 Aurora 链路，纯 MID-360S 部署不使用 `AURORA_IP`。
Linux MediaMTX 可执行文件需另行安装为对应架构，仓库文件存在不代表包含 Linux 二进制。

脚本：

```bash
cd /home/chersxir/work/Grinder/catkin_ws
AURORA_IP=192.168.0.114 ./start_grinder_stack.sh
```

默认 mediamtx 选择优先级：

1. `MEDIAMTX_BIN`（环境变量显式指定）
2. `tools/mediamtx/mediamtx_aarch64` 或 `tools/mediamtx/mediamtx_x86_64`
3. `tools/mediamtx/mediamtx`

手动指定示例：

```bash
MEDIAMTX_BIN=/home/neardi/work/Grinder/tools/mediamtx/mediamtx AURORA_IP=192.168.0.114 ./start_grinder_stack.sh
```

## 3. 常见问题

- `Exec format error`：通常是二进制架构不匹配（例如在 RK3588 上用了 x86_64 的 mediamtx）。
- `catkin_make` 指向旧路径：跨机器拷贝后需要重新编译，脚本会自动备份路径错配或架构冲突的 `build/devel/install`。
- `Cannot apply task process speed to navigation`：先检查地图到车体 TF，再查 move_base 的 STVL 插件加载、jemalloc 环境和 RPP 参数服务。节点存在不代表服务已就绪，见 [当天排障记录](../README.md#deployment-pitfalls-20261022)。
