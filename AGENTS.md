# Grinder 智能体开发导航

本文件是仓库级快速索引。所有路径均相对仓库根目录；先按下表定位，再读取目标文件的局部内容，避免全仓扫描。

## 先读这些

1. `README.md`：系统目标、核心链路、启动与联调概览。
2. `catkin_ws/PLATFORM_BUILD_RUN.md`：x86_64 / RK3588 的构建和启动参数。
3. 只读取本次任务对应的包、配置和 launch 文件；不要先递归读取整个 `catkin_ws/src`。

注意：仓库根目录是唯一 Git 仓库。原有子仓库的源码已经吸收到根仓库；`.git.nested-backup-*` 仅作为本地迁移备份并被忽略。Git 操作统一从仓库根目录执行，不要在源码子目录重新初始化仓库。

## 系统主链路

```text
APP/上位机
  -> SL-LinkA TCP/Protobuf
  -> grinder_scheduler（协议分发、地图、规划、任务状态机）
  -> move_base + TEB/RPP（导航，输出 /cmd_vel）
  -> grinder_chassis_driver（任务使能、速度换算、Modbus RTU）
  -> 研磨机底盘

Aurora/slamware -> /map、/odom、图像、雷达状态 -> scheduler/navigation
```

## 文件定位表

| 要修改/排查的内容 | 首选文件 | 相关文件 |
| --- | --- | --- |
| 调度入口、任务状态机、协议处理函数 | `catkin_ws/src/grinder_scheduler/src/grinder_scheduler/scheduler_node.py` | `models.py`、`scripts/scheduler_node.py` |
| 调度参数 | `catkin_ws/src/grinder_scheduler/config/scheduler.yaml` | `launch/scheduler.launch`、`launch/grinder_system.launch` |
| 地图编辑、区域叠加、预览、持久化 | `catkin_ws/src/grinder_scheduler/src/grinder_scheduler/map_service.py` | `scheduler_node.py` 中 `handle_map_*`、`_map_*` |
| 路径规划输入/输出适配 | `catkin_ws/src/grinder_scheduler/src/grinder_scheduler/planner_adapter.py` | `third_party/path_planner/mst27/mst27.py`、`mst27/cpp/mst27_cpp.cpp` |
| SL-LinkA 消息接收与分发 | `catkin_ws/src/grinder_scheduler/src/grinder_scheduler/sl_linka_adapter.py` | `sl_link_loader.py`、`scheduler_node.py` 中 `handle_*` / `build_*` |
| 协议定义与说明 | `third_party/sl_linka/sl_linka/proto/sl_link.proto` | `third_party/sl_linka/sl_linka/sl-link.md`、`scripts/generate_proto.sh` |
| Aurora 地图、位姿和双目图像接入 | `catkin_ws/src/grinder_scheduler/src/grinder_scheduler/aurora_bridge.py` | `catkin_ws/src/2-dnavigation-package/slamware_ros_sdk/` |
| 地图目录响应 | `catkin_ws/src/grinder_scheduler/src/grinder_scheduler/map_catalog_response.py` | `scheduler_node.py` 中 `handle_map_catalog_request` |
| RTSP / FFmpeg 视频链路 | `local_rtsp_server.py`、`media_streamer.py`（均在 scheduler 模块目录） | `runtime/mediamtx`、`temp/mediamtx.generated.yml` |
| 平台文件上传、鉴权和 MQTT 上报 | `catkin_ws/src/grinder_scheduler/src/grinder_scheduler/platform_integration.py` | `mqtt_interface.md`、`scheduler.yaml` |
| 底盘 ROS 节点、`/cmd_vel`、任务使能 | `catkin_ws/src/grinder_chassis_driver/src/grinder_chassis_driver/chassis_driver_node.py` | `config/chassis_driver.yaml`、`launch/chassis_driver.launch` |
| Modbus 帧、串口、CRC | `catkin_ws/src/grinder_chassis_driver/src/grinder_chassis_driver/modbus_transport.py` | `test/test_modbus_transport.py` |
| 寄存器地址、编解码、底盘命令模型 | `catkin_ws/src/grinder_chassis_driver/src/grinder_chassis_driver/register_map.py` | `test/test_register_map.py`、`doc/底盘485指令.png` |
| ROS 消息/服务 | 两个自研包各自的 `msg/`、`srv/` | 对应 `CMakeLists.txt`、`package.xml` |
| move_base / TEB / RPP 参数 | `catkin_ws/src/2-dnavigation-package/2dnavigation/teb_local_planner_tutorials/cfg/diff_drive/` | 同包 `launch/robot_diff_drive.launch` |
| 激光里程计与 EKF | `catkin_ws/src/rf2o_laser_odometry/` | `launch/`、`config/ekf.yaml` |
| 一键构建/启动 | `catkin_ws/build_grinder_platform.sh`、`catkin_ws/start_grinder_stack.sh` | `catkin_ws/PLATFORM_BUILD_RUN.md` |
| X920 Gazebo 全链路仿真、STEP 模型、APP 联调 | `catkin_ws/src/grinder_gazebo/README.md` | `catkin_ws/src/grinder_gazebo/launch/grinder_sim.launch`、`catkin_ws/src/grinder_gazebo/urdf/x920_grinder.urdf.xacro`、`catkin_ws/start_grinder_sim.sh` |

`scheduler_node.py` 约 9000 行，不要整文件加载。先用符号搜索定位，例如：

```bash
rg -n "def handle_task_|def handle_map_|def handle_path_|def _start_execution|def _stop_execution" catkin_ws/src/grinder_scheduler/src/grinder_scheduler/scheduler_node.py
rg -n "参数名|话题名|消息名" catkin_ws/src/grinder_scheduler catkin_ws/src/grinder_chassis_driver
```

## 常见需求的最短阅读路径

- 新增/修改协议：`sl_link.proto` -> `sl_linka_adapter.py` -> `scheduler_node.py` 对应处理函数；之后重新生成相关 SDK。
- 地图问题：`scheduler.yaml` 的存储参数 -> `map_service.py` -> `scheduler_node.py` 中 `handle_map_*`。
- 路径问题：`scheduler.yaml: planner_script_path` -> `planner_adapter.py` -> `mst27.py`；C++ 加速问题再读 `mst27_cpp.cpp`。
- 任务不走：`handle_task_config` / `handle_task_command` -> `_plan_current_task` -> `_start_execution` -> `_dispatch_current_goal` / `_tick_path_execution`。
- 底盘不动：先查 `/chassis/task_enable`，再查 `/cmd_vel`，然后读 `chassis_driver_node.py` 的 `_handle_cmd_vel` 和 `chassis_driver.yaml`。
- 导航行为异常：先查 `robot_diff_drive.launch` 选用的 `teb` 或 `rpp`，再读对应 YAML；不要先改上游 ROS navigation 源码。
- 视频问题：`local_rtsp_server.py` -> `media_streamer.py` -> `start_grinder_stack.sh` 的 mediamtx 选择逻辑。
- 状态丢失/串图：查看 `temp/grinder_scheduler_state/` 中 registry 和 `map_states/`，但不要把运行时数据当作源码修改。

## 源码、外部代码与生成物边界

- 自研主代码：`catkin_ws/src/grinder_scheduler/`、`catkin_ws/src/grinder_chassis_driver/`。
- 项目集成代码：`third_party/path_planner/`、导航参数与启动文件；修改前确认任务确实涉及算法或导航配置。
- 外部代码：`2-dnavigation-package` 的通用 ROS navigation 源码、`slamware_ros_sdk`、`rf2o_laser_odometry`、`third_party/sl_linka/sl_linka/nanopb`。优先通过项目配置或适配层解决问题。
- 不要索引或手改生成/运行目录：`catkin_ws/build/`、`catkin_ws/devel/`、`catkin_ws/logs/`、各处 `build/`、`__pycache__/`、`*.pyc`、`temp/`。
- `maps/raw/*.stcm`、`*.pgm`、预览图、路径 JSON、编译出的 `*.so` 都是数据或产物，除非任务明确要求，否则不要修改。
- 协议 Python/Android/Embedded 生成代码位于 `third_party/sl_linka/sl_linka/sdk/`；应修改 `.proto` 后运行生成脚本，而非只改生成文件。

## 构建与验证

项目目标环境是 Linux + ROS1 Noetic；Windows 工作区主要用于编辑，完整 ROS 构建/启动需在目标 Linux 环境执行。

```bash
cd catkin_ws
./build_grinder_platform.sh                 # 全量、自动识别架构
PROFILE=scheduler ./build_grinder_platform.sh
PROFILE=chassis ./build_grinder_platform.sh
PROFILE=nav ./build_grinder_platform.sh
```

底盘纯 Python 单测（不连接硬件）：

```bash
cd catkin_ws
PYTHONPATH=src/grinder_chassis_driver/src python3 -m unittest discover -s src/grinder_chassis_driver/test -p 'test_*.py'
```

轻量语法检查：

```bash
python3 -m compileall -q catkin_ws/src/grinder_scheduler/src/grinder_scheduler catkin_ws/src/grinder_chassis_driver/src/grinder_chassis_driver
```

启动与联调：

```bash
cd catkin_ws
AURORA_IP=<雷达IP> ./start_grinder_stack.sh
rosnode info /grinder_scheduler
rostopic echo /chassis/task_enable
rostopic hz /cmd_vel
```

涉及串口、轮速、磨盘、急停或真实地图切换的验证会影响硬件；未明确授权时只做静态检查、单测或仿真验证。

## 修改约束

- Python 可执行入口只是 `scripts/*.py` 包装器，业务实现应改 `src/<package>/` 下模块。
- 新增或修改 ROS 参数时，同步检查 YAML 默认值、`rospy.get_param` 回退值和 launch 透传，避免三处不一致。
- 修改 `msg/`、`srv/`、`package.xml` 或 `CMakeLists.txt` 后必须重新 Catkin 构建。
- 修改 `sl_link.proto` 后运行 `third_party/sl_linka/sl_linka/scripts/generate_proto.sh` 的对应目标，并检查调度适配器与各 SDK 一致。
- 保持 Python 3.8 兼容（板端已有 CPython 3.8 构建产物）；不要无意引入更高版本专用语法。
- 安全链路优先：任务停止/暂停/异常必须继续确保 `task_enable=false`、轮速清零和磨盘安全状态；不要绕过现有超时与急停保护。
- `README.md` 与包内 README 可能滞后。已知底盘 README 仍称未实现 `/cmd_vel` 转换，但当前源码和配置已实现；事实以源码、YAML 和 launch 为准，必要时同步修正文档。

## 完成任务前

1. 只报告实际改动的源码/配置/文档，不把运行时文件算入改动。
2. 运行与改动范围相称的最小验证，并说明未运行的硬件/ROS 验证。
3. 若新增了模块、协议、启动入口或重要数据流，同步更新本文件的定位表，保持它可用且简短。
