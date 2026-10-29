# Grinder 智能体开发导航

本文件是仓库级快速索引。所有路径均相对仓库根目录；先按下表定位，再读取目标文件的局部内容，避免全仓扫描。

## 定位顺序与目录约定

1. 根据问题在「文件定位表」选定模块；先用 `rg --files <目标目录>` 确认文件名。
2. 在目标文件中用 `rg -n` 查函数、参数、话题或消息名，再读取命中位置附近的代码；不要依赖本文未固定的行号。
3. 沿调用方、实现、配置、测试核对同一条链路。搜不到时先检查目录迁移和模块拆分，再扩大到相关包。
4. 事实优先级：当前源码与 launch/YAML > 包内说明 > 历史资料。不要把相邻 APP 仓库或本地归档中的代码当成本仓实现。

| 范围 | 实际目录 / 约定 |
| --- | --- |
| 调度包 / 调度模块 | `catkin_ws/src/grinder_scheduler/` / `catkin_ws/src/grinder_scheduler/src/grinder_scheduler/` |
| 底盘包 / 底盘模块 | `catkin_ws/src/grinder_chassis_driver/` / `catkin_ws/src/grinder_chassis_driver/src/grinder_chassis_driver/` |
| Super-LIO 算法包 | `catkin_ws/src/localization/Super-LIO/src/super_lio/`；注意仓库目录与 ROS 包名 `super_lio` 不同 |
| 定位相关包 | `catkin_ws/src/localization/`，含 `super_lio_loop/`、`rf2o_laser_odometry/`、`fast_gicp/` |
| 建图相关包 | `catkin_ws/src/mapping/`，含 `cloud_to_occupancy_grid/`、`pcd2pgm/` |
| 协议工程 | `third_party/sl_linka/`；其下直接是 `proto/`、`scripts/`、`sdk/`、`nanopb/`，没有第二层 `sl_linka/` |

下表中的裸模块名（如 `scheduler_node.py`）指对应包的业务模块；`config/`、`launch/`、`test/`、`scripts/` 指对应包目录下的文件。跨包路径使用仓库相对路径；Super-LIO 行中的 `include/`、`src/`、`test/` 相对其算法包。

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
MID-360 -> Super-LIO -> /lio/odom、点云 -> 可选回环 -> 二维栅格 /map
                     -> super_lio_mode_manager -> 定位就绪/质量 -> scheduler
```

## 文件定位表

| 要修改/排查的内容 | 首选文件 | 相关文件 |
| --- | --- | --- |
| 调度入口、任务状态机、协议处理函数 | `catkin_ws/src/grinder_scheduler/src/grinder_scheduler/scheduler_node.py` | `models.py`、`scripts/scheduler_node.py` |
| 调度参数与设备覆盖 | `catkin_ws/src/grinder_scheduler/config/scheduler.yaml` | `config/scheduler.local.example.yaml`、被忽略的 `scheduler.local.yaml`；`config/super_lio.yaml`、`launch/scheduler.launch` |
| 调度配置启动预检 | `catkin_ws/scripts/check_scheduler_config.py` | `scripts/scheduler_config.sh`、`start_grinder_stack.sh`、`start_grinder_super_lio_base.sh`、`start_grinder_sim.sh` |
| Super-LIO 源码与上游版本 | `catkin_ws/src/localization/Super-LIO/UPSTREAM.md` | 同目录 `LICENSE`、`src/super_lio/`、`src/basic/`；普通源码，无子模块初始化步骤 |
| Super-LIO 每帧迭代收敛诊断 | `catkin_ws/src/localization/Super-LIO/src/super_lio/src/lio/ESKF.cpp` | `include/lio/ESKF.h` 保存迭代记录；`src/apps/relocation.cpp` 输出 `/super_lio/registration_diagnostics`；字段说明见 `UPSTREAM.md` |
| Super-LIO IMU 到运动中心位姿、速度与协方差换算 | `catkin_ws/src/localization/Super-LIO/src/super_lio/include/lio/base_kinematics.h` | `src/lio/ROSWrapper.cpp` 发布底盘状态；`src/apps/relocation.cpp` 换算底盘初始位姿；`test/test_base_kinematics.cpp`；标定参数与验证见 `UPSTREAM.md` |
| Super-LIO 内部轮速融合（默认关闭） | `catkin_ws/src/localization/Super-LIO/src/super_lio/src/lio/wheel_ros.cpp`、`wheel_eskf.cpp` | `include/lio/wheel_fusion.h`、`src/lio/wheel_math.cpp`；底盘有效观测 `/chassis/wheel_twist`；调度 `config/super_lio.yaml` 两模式及两观测开关；`test/test_wheel*.cpp`、底盘 `test_wheel_feedback.py`、调度 `test_super_lio_wheel_config.py` |
| 地图编辑、区域叠加、预览、持久化 | `catkin_ws/src/grinder_scheduler/src/grinder_scheduler/map_service.py` | `scheduler_node.py` 中 `handle_map_*`、`_map_*` |
| 路径规划输入/输出适配、请求内共享预处理 | `catkin_ws/src/grinder_scheduler/src/grinder_scheduler/planner_adapter.py` | `models.py: explicit_region_start` 区分旧默认角点和指定入口，`map_service.py` 保留未指定起点为空；`third_party/path_planner/mst27/mst27.py` 中 `PreparedPlanningMap`、`mst27/cpp/mst27_cpp.cpp`；中间作业线严格间距、末端边缘吸收余量：`mst27/test/test_spacing.py`；离线回归 `mst27/test/test_preprocessing.py`、scheduler `test/test_planner_preprocessing.py`、`test/test_planning_direction.py` |
| 手动停止后的任务续跑、作业线方向与轮次 | `catkin_ws/src/grinder_scheduler/src/grinder_scheduler/task_continuation.py` | `scheduler_node.py` 中 `_capture_task_continuation` / `_prepare_task_continuation_start`；`planner_adapter.py`；离线回归 `test/test_task_continuation.py`、`test/test_planner_continuation.py` |
| MST27短折返、等价膨胀提速及阶段计时 | `third_party/path_planner/mst27/cpp/mst27_cpp.cpp`、同包 `mst27.py` | 安全外侧共同掉头线；方形膨胀可分离滤波、最终复检分批向量化；`plan_core_profiled` / `planner.last_profile`；同包 `test/test_optimizations.py`，包README |
| 全局关键点、segment 切片/投影与局部补点 | `path_geometry.py`、`scheduler_node.py` 中 `_build_path_direction_runs` / `_slice_path_points_by_arc` / `_publish_active_segment_plan` | `planner_adapter.py` 关闭全局补点；`scheduler.yaml: active_segment_point_spacing`；`test/test_sparse_segments.py`、`test/test_sparse_planner.py`、`test/benchmark_sparse_segments.py --baseline-ref <修改前提交>`；根 README 7.5 |
| SL-LinkA 消息接收与分发 | `catkin_ws/src/grinder_scheduler/src/grinder_scheduler/sl_linka_adapter.py` | `sl_link_loader.py`、`scheduler_node.py` 中 `handle_*` / `build_*` |
| 原始取图LUT、实时栅格共享与有界缓存 | scheduler `map_grid_utils.py`、`scheduler_node.py: build_map_chunks / _build_raw_map_snapshot` | `aurora_bridge.py: get_map_with_seq`、`map_service.py` 只读栅格副本；`map_preview_grayscale`默认关闭；`test/test_map_request_payload.py` |
| TCP接收轮询与有界发送期限分离 | scheduler `socket_io.py`、`sl_linka_adapter.py` | 数据每帧默认2秒、控制默认0.5秒，接收轮询100ms；`test/test_socket_io.py`（背压Linux验证）；包README |
| SL-LinkA 双端口、单平板连接组与亮屏恢复 | scheduler `channel_sessions.py`、`sl_linka_adapter.py` | 数据8002/控制8003；会话 `0x0403/0x0404`；`test/test_dual_channel.py`，根README 7.7；APP 在独立APP仓库 `core:tcp` |
| 遥控指令跟手、忙时松手停车 | `sl_linka_adapter.py` 中手动最新指令队列、scheduler `capture_control_generation` | 排队绑定执行代号；底盘零速绕过斜坡；`test_manual_control_delivery.py`、`test_motion_safety.py`；部署与超时见根 README 7.3 |
| 遥控有效期/连接会话/控制确认优先 | scheduler `manual_delivery.py`、`priority_wire.py`、`sl_linka_adapter.py` | `scheduler_node.py`提交校验；底盘`motion_safety.py`交付检查；`test_field_safety.py`，根README 7.4 |
| 规划跨请求基础缓存与定位失败分项 | scheduler `planning_cache.py`、`registration_health.py` | `planner_adapter.py`、`super_lio_mode_manager.py`；缓存默认2项/128MiB，门限保持不变 |
| RPP扫描诊断、自滤波与离线采样回放 | RPP `include/regulated_pure_pursuit_controller/scan_classifier.hpp` | `/rpp/collision_detail`；`catkin_ws/scripts/capture_field_diagnostics.py`、`replay_collision_scan.py`；自滤波默认关闭，根README 7.4 |
| 手动差速单轮过零 | scheduler `manual_wheel_transition.py`、`scheduler_node.py` | 单轮制动/实测零速/反向加速，另一轮协调；完整前后换向等待；`test_manual_wheel_transition.py`、`test_manual_steering_guard_activity.py`；根 README 7.3 |
| 当前任务临时直行速度与磨盘转速 | `catkin_ws/src/grinder_scheduler/src/grinder_scheduler/task_runtime_parameters.py` | `scheduler_node.py` 中 `handle_task_runtime_settings`；协议 `0x0538/0x0539`；`test/test_task_runtime_parameters.py` |
| 独立累计/今日作业时间、历史删除后的保留 | `work_time_statistics.py`、scheduler `_begin_task_execution_record` / `_finalize_task_result` | `work_time_statistics.json` 独立文件；现有历史JSON根对象 `work_time_statistics`；`test/test_work_time_statistics.py`，包README；不随历史删除重建或清零 |
| 任务启动时下发工艺速度 | `catkin_ws/src/grinder_scheduler/src/grinder_scheduler/scheduler_node.py` 中 `_execution_run_speed`、`_start_execution` | `test/test_execution_settings.py`；底盘使能前将工艺速度下发到导航控制器 |
| APP 消息中心的设备/定位告警 | `scheduler_node.py` 中 `build_device_status_report` | `third_party/sl_linka/proto/sl_link.proto` 的 `DeviceStatusReport`；`test/test_chassis_connection_report.py`；APP 展示逻辑需到 APP 仓库定位 |
| 平板单按钮急停/解除状态 | `scheduler_node.py` 中 `build_device_status_report` / `_reset_emergency_stop` | `DeviceStatusReport.emergency_stop_latched=22` 上报调度运动锁存；协议生成 Python/Android/nanopb SDK 同步 |
| 板端日志采样与指标导出 | `catkin_ws/src/grinder_scheduler/src/grinder_scheduler/metrics_registry.py` | `metrics_exporter.py`、`grinder_metrics_node.py`、`launch/scheduler.launch` |
| 协议定义与说明、地图请求结果/快照分片 | `third_party/sl_linka/proto/sl_link.proto` | `third_party/sl_linka/sl-link.md`、`third_party/sl_linka/scripts/generate_proto.sh` |
| Aurora 地图、位姿和双目图像接入 | `catkin_ws/src/grinder_scheduler/src/grinder_scheduler/aurora_bridge.py` | `catkin_ws/src/2-dnavigation-package/slamware_ros_sdk/` |
| 地图目录响应 | `catkin_ws/src/grinder_scheduler/src/grinder_scheduler/map_catalog_response.py` | `scheduler_node.py` 中 `handle_map_catalog_request` |
| RTSP / FFmpeg 视频链路 | `local_rtsp_server.py`、`media_streamer.py`（均在调度模块目录） | `tools/mediamtx/`；`catkin_ws/start_grinder_stack.sh` 选择二进制；`LocalRtspStreamServer` 在日志目录生成 `mediamtx.generated.yml` |
| 平台文件上传、鉴权和 MQTT 上报 | `catkin_ws/src/grinder_scheduler/src/grinder_scheduler/platform_integration.py` | `mqtt_interface.md`、`scheduler.yaml` |
| 底盘 ROS 节点、`/cmd_vel`、任务使能 | `catkin_ws/src/grinder_chassis_driver/src/grinder_chassis_driver/chassis_driver_node.py` | `config/chassis_driver.yaml`、`launch/chassis_driver.launch` |
| RK3588 控制盒状态灯带、USB 协议与急停提示 | `catkin_ws/src/grinder_scheduler/src/grinder_scheduler/status_indicator.py`、`status_indicator_node.py` | `launch/status_indicator.launch`、`config/status_indicator.yaml`（`brightness_percent` 1～100，默认65）；scheduler `_pause_execution(reason)` 区分人为/碰撞暂停；底盘 `indicator_emergency.py`；固件、接线、亮度升级见 `firmware/status_strip/README.md` |
| Modbus 帧、串口、CRC | `catkin_ws/src/grinder_chassis_driver/src/grinder_chassis_driver/modbus_transport.py` | `test/test_modbus_transport.py` |
| 连续遥控、普通控制阻塞与串口日志争用 | scheduler `sl_linka_adapter.py` 的独立控制 FIFO；chassis `modbus_transport.py` / `raw_log.py` | 完整事务总时限、写优先、有界后台原始日志；`test_continuous_manual.py`、底盘 `test_serial_scheduling.py` / `test_raw_log.py`；根 README 7.6 |
| 软件急停锁存、取消代号与显式解除 | `catkin_ws/src/grinder_chassis_driver/src/grinder_chassis_driver/motion_safety.py` | 驱动统一输出提交／安全同步；scheduler `_reset_emergency_stop`、`_restore_indicator_reset`（重启恢复已确认解除的清灯请求）；`ignore_chassis_status=true` 允许无底盘测试解除调度锁存；服务及回归见根 README 7.2、`test_motion_safety.py`、`test_emergency_stop_execution.py` |
| 寄存器地址、编解码、底盘命令模型 | `catkin_ws/src/grinder_chassis_driver/src/grinder_chassis_driver/register_map.py` | `test/test_register_map.py`、`test/test_motor_faults.py`；`0x4019/0x401A` 左右电机故障码原值上报 `/chassis/status` 和诊断；`doc/底盘485指令.png` |
| ROS 消息/服务 | 两个自研包各自的 `msg/`、`srv/` | 对应 `CMakeLists.txt`、`package.xml` |
| move_base / TEB / RPP 参数 | `catkin_ws/src/2-dnavigation-package/2dnavigation/teb_local_planner_tutorials/cfg/diff_drive/` | 同包 `launch/robot_diff_drive.launch` |
| 激光里程计与 EKF | `catkin_ws/src/localization/rf2o_laser_odometry/` | `launch/rf2o_laser_odometry.launch`、`launch/ekf.launch`、`config/ekf.yaml` |
| MID-360S + Super-LIO 实时二维栅格建图 | `catkin_ws/src/mapping/cloud_to_occupancy_grid/src/cloud_to_occupancy_grid_node.cpp` | 该包 `config/mid360.yaml`、`launch/mid360_mapping.launch`、`launch/mapper.launch` |
| Super-LIO 生命周期/定位质量/地图版本 | `catkin_ws/src/grinder_scheduler/src/grinder_scheduler/super_lio_mode_manager.py`、`map_asset_revision.py` | `/super_lio/localization_status` 10 Hz 心跳；调度 `_localization_watchdog_tick`；`config/super_lio.yaml`、`srv/*SuperLio*.srv` |
| 托管建图/定位启动、停止与保存 | `super_lio_mode_manager.py` 中 `_start_mapping_locked`、`_start_localization_locked`、`_stop_owned_launch`、`_handle_save_map` | 调度包 `launch/super_lio_mapping_managed.launch`、`launch/super_lio_localization_managed.launch`；算法包 `launch/Livox_mid360.launch`、`launch/relocation.launch` |
| 初始位姿、重定位、定位质量门控 | `super_lio_mode_manager.py` 中 `_handle_set_initial_pose`、`_evaluate_localization_sample`、`_refresh_localization_health_locked` | `scheduler_node.py` 中 `handle_radar_relocalization_request`、`_localization_watchdog_tick`；`test/test_relocalization_initial_pose.py`、`test/test_super_lio_mode_manager.py` |
| Super-LIO 回环检测、位姿图和协方差 | `catkin_ws/src/localization/super_lio_loop/` | 工业后端 `src/super_lio_graph_backend_node.cpp`；轻量后端 `src/super_lio_loop_node.cpp`；`config/mid360_loop.yaml`、`launch/loop_closure.launch`、`srv/GetKeyframe.srv` |
| 导航控制器实现与代价地图 | `catkin_ws/src/2-dnavigation-package/2dnavigation/regulated_pure_pursuit_controller/` | 同级 `teb_local_planner/`；优先检查 tutorials 的 `rpp_local_planner_params.yaml`、`teb_local_planner_params.yaml`、`*costmap*params.yaml`，再读控制器源码；低速补偿见 `include/regulated_pure_pursuit_controller/low_speed_steering.hpp`、`test/low_speed_steering_test.cpp`，配置为 `low_speed_angular_deadband` / `low_speed_min_angular_vel` |
| ESP32 灯带固件、串口协议 | `firmware/status_strip/src/main.cpp`、`firmware/status_strip/include/indicator_protocol.h` | `firmware/status_strip/platformio.ini`、`firmware/status_strip/test/protocol_host.cpp`；调度 `status_indicator.py` 与跨语言回归 `test/test_status_indicator_cross_language.py` |
| 一键构建/启动 | `catkin_ws/build_grinder_platform.sh`、`catkin_ws/start_grinder_stack.sh`、`catkin_ws/start_grinder_super_lio_base.sh` | Livox UDP 端口预检：`catkin_ws/scripts/check_livox_udp_ports.py`；`catkin_ws/PLATFORM_BUILD_RUN.md` |
| 板端整套开机自启、配置与 systemd 管理 | `catkin_ws/scripts/grinder-service`、`catkin_ws/install_grinder_service.sh` | 板端 `/etc/grinder/startup.conf`；`start_grinder_super_lio_base.sh` 先启动灯带并监督子进程；离线验证 `scripts/test_grinder_service.py`；根 README 第 12 节 |
| RK3588 / MID-360S 完整部署、STVL/jemalloc 与部署踩坑 | 根 `README.md` 的安装编译节、`deployment-stvl` / `deployment-20261022` / `deployment-pitfalls-20261022` 锚点 | `catkin_ws/PLATFORM_BUILD_RUN.md` 为快捷入口；当天板端 launch、网络和 systemd 额外配置需按记录复现，不能假定 main 自带 |
| X920 Gazebo 全链路仿真、STEP 模型、APP 联调 | `catkin_ws/src/grinder_gazebo/README.md` | `catkin_ws/src/grinder_gazebo/launch/grinder_sim.launch`、`catkin_ws/src/grinder_gazebo/urdf/x920_grinder.urdf.xacro`、`catkin_ws/start_grinder_sim.sh`；远端 WSL 同步/编译/启动与 MuMu 隧道：`catkin_ws/scripts/remote_sim.ps1`、`remote_sim.sh` |
| 虚拟 MID-360S、IMU 噪声、B1 梁柱墙板场景与全链路验证 | `catkin_ws/src/grinder_gazebo/src/mid360_sensor_plugin.cpp`、`config/sensor_noise.yaml`（相对 Gazebo 包） | `urdf/x920_grinder.gazebo.xacro`、`meshes/b1_structure/source_report.json`、`test/check_mid360_chain.py --map-cycle`（仅独立仿真实例）、`test/test_support_geometry.py`；Livox 仿真构建：`catkin_ws/scripts/build_sim_livox.sh` |

`scheduler_node.py` 超过一万行，不要整文件加载。先用符号搜索定位，例如（均从仓库根目录执行）：

```bash
rg -n "def handle_task_|def handle_map_|def handle_path_|def _start_execution|def _stop_execution" catkin_ws/src/grinder_scheduler/src/grinder_scheduler/scheduler_node.py
rg -n "参数名|话题名|消息名" catkin_ws/src/grinder_scheduler catkin_ws/src/grinder_chassis_driver
rg --files catkin_ws/src/grinder_scheduler/test
rg -n 'super_lio|cloud_to_occupancy_grid|include' catkin_ws/src/grinder_scheduler/launch
```

读取局部代码：Linux 可用 `sed -n '起始行,结束行p' 文件`；PowerShell 可用 `Get-Content 文件 | Select-Object -Skip 起始行减一 -First 行数`。先用上面的搜索获得实际行号。

## 常见需求的最短阅读路径

- 新增/修改协议：`sl_link.proto` -> `sl_linka_adapter.py` -> `scheduler_node.py` 对应处理函数；之后重新生成相关 SDK。
- 地图问题：`scheduler.yaml` 的存储参数 -> `map_service.py` -> `scheduler_node.py` 中 `handle_map_*`。
- 路径问题：`scheduler.yaml: planner_script_path` -> `planner_adapter.py` -> `mst27.py`；C++ 加速问题再读 `mst27_cpp.cpp`。
- 任务不走：`handle_task_config` / `handle_task_command` -> `_plan_current_task` -> `_start_execution` -> `_publish_active_segment_plan` / `_send_active_segment_goal` / `_tick_path_execution`。
- 底盘不动：先查 `/chassis/task_enable`，再查 `/cmd_vel`，然后读 `chassis_driver_node.py` 的 `_handle_cmd_vel` 和 `chassis_driver.yaml`。
- 导航行为异常：先查 `robot_diff_drive.launch` 选用的 `teb` 或 `rpp`，再读对应 YAML；不要先改上游 ROS navigation 源码。
- 视频问题：`local_rtsp_server.py` -> `media_streamer.py` -> `start_grinder_stack.sh` 的 mediamtx 选择逻辑。
- 建图/定位启停或保存失败：调度 `handle_map_mode_request` / `handle_map_save_request` -> `super_lio_mode_manager.py` -> 对应 managed launch -> 建图包或算法包 launch。
- 定位不就绪/任务因定位暂停：模式管理器 `_evaluate_localization_sample` / `_refresh_localization_health_locked` -> `/super_lio/localization_status` -> 调度 `_localization_watchdog_tick`；配准细节再查 `relocation.cpp`、`ESKF.cpp`。
- 雷达安装偏移/TF 问题：调度 `base_laser_*` -> 模式管理器 `_base_laser_launch_args` -> managed launch 的 `base_to_laser_*` -> `base_kinematics.h` / `ROSWrapper.cpp`；同时查标定透传回归。
- 状态丢失/串图：查看 `temp/grinder_scheduler_state/` 中 registry 和 `map_states/`，但不要把运行时数据当作源码修改。

## 源码、外部代码与生成物边界

- 自研主代码：`catkin_ws/src/grinder_scheduler/`、`catkin_ws/src/grinder_chassis_driver/`。
- 项目集成代码：`third_party/path_planner/`、导航参数与启动文件；修改前确认任务确实涉及算法或导航配置。
- 外部代码：`2-dnavigation-package` 的通用 ROS navigation 源码、`slamware_ros_sdk`、`catkin_ws/src/localization/rf2o_laser_odometry/`、`third_party/sl_linka/nanopb/`。优先通过项目配置或适配层解决问题。
- 不要索引或手改生成/运行目录：`catkin_ws/build/`、`catkin_ws/devel/`、`catkin_ws/logs/`、各处 `build/`、`__pycache__/`、`*.pyc`、`temp/`。
- `.planning/`、`artifacts/`、整个 `docs/` 均为被忽略的本地资料，不提交任务计划、归档文档、渲染产物或工具缓存。需随源码分发的说明维护在根 README 或包内 README；不要将本地历史报告当成当前代码事实。
- `maps/raw/*.stcm`、`*.pgm`、预览图、路径 JSON、编译出的 `*.so` 都是数据或产物，除非任务明确要求，否则不要修改。
- 协议生成代码分别位于 `third_party/sl_linka/sdk/python/sl_link/message_gen/`、`third_party/sl_linka/sdk/android/src/message_gen/`、`third_party/sl_linka/sdk/embedded/message_gen/`；应修改 `.proto` 后运行生成脚本，而非只改生成文件。

## 构建与验证

项目目标环境是 Linux + ROS1 Noetic；Windows 工作区主要用于编辑，完整 ROS 构建/启动需在目标 Linux 环境执行。

按改动范围选择已有测试；下表的测试文件均位于调度包 `test/`，底盘测试例外：

| 改动范围 | 首选离线回归 |
| --- | --- |
| 执行速度、任务续跑、临时参数 | `test_execution_settings.py`、`test_task_continuation.py`、`test_planner_continuation.py`、`test_task_runtime_parameters.py` |
| 遥控、急停与运动安全 | `test_manual_control_delivery.py`、`test_emergency_stop_execution.py`；底盘包 `test/test_motion_safety.py` |
| 规划预处理、方向 | `test_planner_preprocessing.py`、`test_planning_direction.py`；`third_party/path_planner/mst27/test/test_preprocessing.py` |
| 地图保存、定位与安装标定 | `test_live_map_save.py`、`test_super_lio_mode_manager.py`、`test_relocalization_initial_pose.py`、`test_super_lio_launch_calibration.py` |
| 灯带、指标、平台接入 | `test_status_indicator*.py`、`test_metrics.py`、`test_platform_integration.py` |
| 启动配置与 systemd 管理 | `catkin_ws/scripts/test/test_scheduler_config.py`、`catkin_ws/scripts/test/test_livox_environment.py`（外部环境隔离）、`catkin_ws/scripts/test_grinder_service.py` |

例如从仓库根目录运行单项离线回归（Windows 将 `python3` 换为实际 Python 命令）：

```bash
python3 -m unittest discover -s catkin_ws/src/grinder_scheduler/test -p 'test_execution_settings.py'
```

其他测试先看文件头的依赖和入口；不要假定所有测试均无需 ROS 或第三方库。

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
- 修改 `sl_link.proto` 后运行 `bash third_party/sl_linka/scripts/generate_proto.sh <c|python|kotlin|all>` 的对应目标，并检查调度适配器与各 SDK 一致。
- 保持 Python 3.8 兼容（板端已有 CPython 3.8 构建产物）；不要无意引入更高版本专用语法。
- 安全链路优先：任务停止/暂停/异常必须继续确保 `task_enable=false`、轮速清零和磨盘安全状态；不要绕过现有超时与急停保护。
- `README.md` 与包内 README 可能滞后。已知底盘 README 仍称未实现 `/cmd_vel` 转换，但当前源码和配置已实现；事实以源码、YAML 和 launch 为准，必要时同步修正文档。

## 完成任务前

1. 只报告实际改动的源码/配置/文档，不把运行时文件算入改动。
2. 运行与改动范围相称的最小验证，并说明未运行的硬件/ROS 验证。
3. 若新增了模块、协议、启动入口或重要数据流，同步更新本文件的定位表，保持它可用且简短。
