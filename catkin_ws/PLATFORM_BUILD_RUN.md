# 平台化编译与启动

本工程支持在 `x86_64` 和 `aarch64(RK3588)` 上按平台自动编译与启动。

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
PROFILE=runtime   ./build_grinder_platform.sh   # 底盘+调度
PROFILE=nav       ./build_grinder_platform.sh   # 导航相关
PROFILE=aurora    ./build_grinder_platform.sh   # slamware_ros_sdk
PROFILE=scheduler ./build_grinder_platform.sh
PROFILE=chassis   ./build_grinder_platform.sh
PROFILE=sim       ./build_grinder_platform.sh   # Gazebo + 调度 + RPP/move_base 仿真链路
PROFILE=mapping   ./build_grinder_platform.sh   # Super-LIO + 固定内存二维栅格建图
```

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
