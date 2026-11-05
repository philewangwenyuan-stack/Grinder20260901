# grinder_scheduler 包说明

## 导航页只读路径查询

`PathPointPlanRequest.read_only=true`（0x0505）只返回当前任务已保存的完整路径，允许在运行或暂停中查询，沿用0x0506分片响应及request_id。不调用规划器、不切换地图、不修改任务、路径版本或执行游标；任务/地图不匹配或没有已存路径时明确返回失败。`read_only=false`保留原规划语义及运行中拒绝重规划的行为。APP导航页使用只读请求，补取失败或超时保留同任务旧路线，成功后替换。此字段需要两端配套更新。离线回归：`test/test_current_task_path.py`。

## 无相机设备的 ROS 输入

纯 Super-LIO 调度不再订阅旧 Slamware `system_status` 和
`relocalization_status`；定位就绪、质量和状态仍来自 Super-LIO 管理器。
当前公开默认配置按无相机板端设置：

```yaml
image_inputs_enabled: false
local_rtsp_enabled: false
```

前者同时关闭左右图像或深度彩图订阅，地图、里程计订阅保持启用；后者关闭本地
RTSP 服务链路。两项均在节点初始化时读取，重启后生效。有相机设备须在自己的
`scheduler.local.yaml` 显式开启这两项；Gazebo 在独立仿真覆盖配置中开启图像和 RTSP。
设备账号、端点等覆盖配置保存在被 Git 忽略的 `scheduler.local.yaml`；复制板端部署时
同时携带该设备文件，不要将它提交到仓库或用于仿真。
输入订阅验证见 `test/test_optional_ros_inputs.py`，在 ROS Noetic 环境运行，使用 mock，
不连接 ROS master 或硬件。

## 双通道通信

调度同时监听数据 8002 与控制 8003。`channel_sessions.py` 管理单平板连接组及共享大数据预算，
`sl_linka_adapter.py` 按通道准入，控制断开/遥控过期时撤销遥控，自动任务继续受既有保护。
协议 `0x0403/0x0404` 为握手/心跳/恢复/释放，需同步升级 APP；数据端拒绝运动控制。
配置、亮屏恢复、日志与受控验收见根 README 7.7。无新增 ROS 消息/服务。

控制会话心跳回复通过有界 TX 队列发送，避免接收线程等待发送锁或 socket。
`sl_linka_adapter.py` 的 `channel_close` 事件区分接收遥控超时、发送截止、对端断开、
会话替换/释放/租约失效及服务停止；先执行控制撤销，再记录关闭事件。
`channel_{data,control}_wire_wait_ms` 与 `channel_{data,control}_socket_send_ms` 分别统计
发送锁等待和 socket 写入。旧 `control_ack_wait_ms` 是两端口控制类响应的锁等待，不能当作遥控 RTT。
接收轮询100ms、控制每帧发送1秒、会话3秒门限。回归包括心跳写入受阻时遥控/停车继续接收，
以及 EOF、发送超时、接收遥控超时的原因归属；不连接真实设备。

本文档说明 `catkin_ws/src/grinder_scheduler` 目录下各文件/子目录的作用，便于开发、联调和排障。

## 原始地图取图与缓存

保存地图后的云平台上传仅包含 `loc_map.pcd`、`map.pgm`、`map.yaml`、`map_info.json`，
不上传 `plan_map.pcd` 或目录中的其他文件。本地完整资产包仍保留原文件名供 Super-LIO 使用。
云端目录仍为 `GrinderProject/maps/<mapId>/`，文件名加地图名称前缀，例如
`一楼大厅_loc_map.pcd`、`一楼大厅_map.pgm`、`一楼大厅_map.yaml`、`一楼大厅_map_info.json`。
名称中的路径分隔符、控制字符和 Windows 文件名非法字符替换为下划线，名称前缀最多 80 个字符；
名称为空时使用地图 ID。上传版 YAML 的 `image` 和元数据的 `files`、对应资产路径同步指向新文件名，
这些转换只发生在上传内容中。已有云端旧文件不自动删除，地图改名后的旧文件也不自动清理。
离线回归：`test/test_platform_integration.py`。

`map_grid_utils.py` 不依赖 ROS。保存地图的 MapRequest PNG 直接从 PGM 以 LUT
映射为底图，不再经过 OccupancyGrid 整数列表往返；任务结果的原始地图快照复用同一条路径。
规划/导航仍使用原来的栅格坐标方向。RosDataBridge 用 `numpy_msg(OccupancyGrid)` 接收地图，
保留接收缓冲上的只读 int8 数组视图，不先解成 Python 元组，也不在取图时再次遍历转换。
同一进程同一地图话题的其他订阅也必须使用相同的 NumPy 消息类；独立模式管理器进程不受影响。
若将 NumPy 数据发布为 ROS 地图，Publisher 同样应声明 `numpy_msg(OccupancyGrid)`。
MapService 持有原始数组；保存地图通过 `set_raw_grid(source, grid)` 直接交付元数据和只读数组，
不创建 OccupancyGrid 整数列表。合成、擦除前复制为可写 int16，彩色叠加逻辑保持原样。
兼容入口仍接受普通 ROS 平面整数序列，仅在交付时转换一次；生产实时入口不依赖它。

实时转换的消息身份缓存已移除，消息/MapService的数组引用负责快照生命周期，
新实时帧不会挤占离线栅格。保存图像4项/32MiB、保存栅格4项/32MiB、编码负载8项/32MiB。
这些缓存同时限制条数和保留数据的估算字节。超大项仍可处理，
但不进入缓存；上限不包含正在处理的请求及其他业务内存。同一键的并发构建合并，
耗时转换期间不持有全局缓存锁。保存文件以路径、设备/inode、mtime_ns、ctime_ns和大小
校验，并在读取前后复查；删除不复用旧缓存，项目保存/删除也主动清理缓存。
仅缓存图像/栅格/编码字节，每次请求重新分片并填写 request_id、snapshot_id、地图版本及位姿信息。

`map_preview_grayscale` 默认 `false`，经平板和任务结果接收平台核对后可开启。
只影响原始底图 PNG/JPEG，不影响带区域叠加的彩色预览；关闭时维持旧三通道像素路径。
参数可通过 scheduler.yaml、本地覆盖或 scheduler.launch 同名参数设置，launch 参数留空时
保留 YAML 值。没有修改 cloud_to_occupancy_grid 的建图/发布频率。

细分指标：`map_saved_file_read_ms`（PGM读取）、`map_saved_gray_to_grid_ms`、
`map_colorize_ms`
（LUT与实时图翻转）、`map_resize_ms`、`map_png_encode_ms`/`map_jpg_encode_ms`。
`map_payload_cache_hit_total`/`map_payload_cache_miss_total` 统计编码缓存，保存图像另有
`map_saved_cache_*`。`map_saved_grid_to_list_ms`、`map_grid_convert_ms`、`map_grid_cache_*`
随对应链路删除而下线。命中不重复记录转换/编码耗时，因此须结合计数解读；
`map_load_ms`、`map_build_ms`、`map_request_total_ms` 等外层指标继续保留。

`socket_io.py` 将100ms接收轮询与发送总期限分离。数据通道每帧发送默认2秒
（限制0.1～5秒），控制通道默认1秒（限制0.05～1秒），分别配置
`sl_linka_data_send_timeout_sec` 和 `sl_linka_control_send_timeout_sec`。
非阻塞发送期间检查会话有效性，超过期限关闭连接，不能在部分帧后直接重发整帧。
遥控指令有效期为1秒，断线停车及急停保护保持原样。

离线验证：`test/test_map_request_payload.py`、`test/test_numpy_map_transport.py`（ROS生成消息）、`test/test_socket_io.py`、
`test/test_dual_channel.py`、`test/test_continuous_manual.py`。实际背压用目标Linux环境验证；
Windows回环会缓冲完整写入，对应3项背压用例在Windows跳过。

## 独立作业时间累计

`work_time_statistics.py` 将任务起止经过时长（含暂停，不含手动遥控/空闲/服务停机）独立保存在
`persist_state_dir/work_time_statistics.json`，不受 `persist_runtime_task_state` 或历史删除影响。
今日值按板端本地午夜切分；累计值为全部日期之和。APP从历史响应JSON根对象中的
`work_time_statistics` 读取秒数，即使 `records=[]` 或查询带地图/日期过滤，计数仍为设备全局值。
面积仍使用原有历史口径，本次不改变面积统计。

首次部署从尚保留的历史迁入有效的开始/结束时长，同一执行ID只迁一次；后续启动只读独立计数，
不再根据历史重新求和。已经删除的旧任务和无有效结束时间的任务无法恢复为完整累计值。
缺失计时保留 `incomplete_record_count`，APP以“≥”和“计时不完整”标明下界；删除遗留任务也不清除此标记。
升级前应保留现有 `task_registry.json`，不要先清空历史或整个状态目录。

任务开始、结束与结果图片生成解耦；结果材料失败不影响独立计时停止。计时基于进程单调时钟，
写盘由独立线程处理，每5秒及开始/结束时唤醒，原子替换并fsync，空闲无变化时不重复写盘。
正常关闭等待保存；突然掉电仅恢复最后成功检查点，不把停机间隔加入累计，可能未落盘的尾段
（通常不超过5秒，磁盘阻塞/故障时可能更长）标为不完整。重启不自动恢复任务时钟。

损坏或不支持版本的独立文件会明确返回不可用，绝不静默清零/从剩余历史重建；写盘失败保留旧文件并后台重试。
计数不可用时拒绝删除历史，以保留迁移/恢复来源。备份和迁移设备时应同时保留独立计数文件，
不提供普通历史删除入口清零它。修改运行代码需同时更新 `scheduler_node.py` 和新模块；无新增ROS消息或protobuf字段。

离线验证：`python3 -m unittest discover -s catkin_ws/src/grinder_scheduler/test -p test_work_time_statistics.py -v`。

## 1. 目录结构与作用

```text
grinder_scheduler/
├── CMakeLists.txt
├── package.xml
├── launch/
├── config/
├── msg/
├── scripts/
├── src/grinder_scheduler/
├── test/
└── README.md
```

## 2. 顶层文件

- `CMakeLists.txt`  
  Catkin 构建配置。定义消息生成、安装脚本、依赖链接等。

- `package.xml`  
  ROS 包元信息与依赖声明（编译依赖、运行依赖、版本、维护者等）。

## 3. 子目录说明

## 3.1 launch/

- 存放调度系统启动文件（`.launch`）。
- 常见用途：
  - 启动调度节点；
  - 组合启动底盘驱动、导航、雷达相关节点；
  - 暴露可配置参数（如 IP、地图路径、是否启用导航等）。

## 3.2 config/

- 调度程序 YAML 配置目录。
- `scheduler.yaml`：受控公共默认配置，外部服务默认关闭。
- `scheduler.local.example.yaml`：无凭据的设备覆盖示例。
- `scheduler.local.yaml`：可选的本地覆盖，被 Git 忽略且不随 Catkin 安装；
  真实启动脚本自动发现，直接 roslaunch 使用 `scheduler_local_config:=绝对路径`。
- 典型内容：
  - 机器人参数（宽度、长度、路径间距、转弯半径等）；
  - 运行时策略参数（话题名、频率、功能开关、持久化路径等）。

## 3.3 msg/

- 自定义 ROS 消息定义目录（`.msg`）。
- 例如：
  - `SchedulerStatus.msg`：调度状态发布结构；
  - `MapPreviewMetadata.msg`：地图预览元信息结构。

## 3.4 scripts/

- 可执行脚本入口（通常包含 ROS 节点启动脚本）。
- 常见是将 `src/grinder_scheduler/` 中核心 Python 模块通过脚本方式运行。

## 3.5 src/grinder_scheduler/

核心业务代码目录（Python 模块）。

- `scheduler_node.py`  
  调度主节点。  
  负责：
  - 任务配置/任务控制（TaskConfig/TaskCommand）；
  - 路径规划请求与状态推进；
  - 地图管理（保存、查询、删除、切换、预览）；
  - 与导航、底盘、雷达、SL-LinkA 协议的总调度逻辑；
  - 状态上报与本地持久化（map/task registry 等）。

- `models.py`  
  数据模型定义（任务配置、路径模型、状态模型等），用于统一内部结构体。

- `planner_adapter.py`  
  规划器适配层。把调度输入转换为规划器可用输入，并标准化输出路径结果。

- `map_service.py`  
  地图数据处理服务。包含地图叠加区域管理、地图预览相关处理等。

- `ros_data_bridge.py`
  通用 ROS 数据桥接层。接收地图、里程计和相机帧，适用于 MID-360＋Super-LIO 等数据来源。

- `sl_linka_adapter.py`  
  SL-LinkA 协议适配入口。解析协议消息并分发到调度处理函数。

- `map_catalog_response.py`  
  地图查询返回拼装工具（如 MapCatalogResponse 填充、缩略图预算控制）。

- `local_rtsp_server.py`  
  本地 RTSP 服务封装（配合 mediamtx/ffmpeg），用于视频流转发。

- `media_streamer.py`  
  媒体流推送状态管理，维护流在线状态与流地址信息。

## 3.6 test/

- 测试或联调用脚本目录（若存在）。  
- 用于本地开发验证，不属于线上运行主链路。

## 4. 运行主链路（简要）

1. 上位机/安卓通过 SL-LinkA 下发请求；  
2. `sl_linka_adapter.py` 解析后调用 `scheduler_node.py` 对应处理函数；  
3. `scheduler_node.py` 调用地图/规划/底盘/导航接口执行；  
4. 通过 TaskStatus/DeviceStatus/MapCatalog 等消息回传状态与结果。

## 5. 常见排查入口

APP 未指定规划起点时，路径从当前车位出发；区域未指定起点时不再自动补外接矩形左上角。
用户指定的区域起点作为覆盖入口，车位到入口的连接段也包含在无任务 ID 的路径预览中。
旧地图里未记录来源、且恰好等于旧默认左上角的起点按自动值忽略，以消除正矩形和倾斜区域的差异。
若旧地图的这个角点原本是手工指定的，需要重新设置一次；新编辑使用内部 `start_pose_explicit`
标记保留用户选择，保存和重新加载后仍有效，不改变协议。

手动停止后，在同一调度进程中再次开始同一任务，可恢复停止时的路径进度和往返方向。
停止先清零运动并关闭磨盘，不将未完成轮次标记为完成；重复停止保留同一个检查点。
地图修订、几何和任务配置未变时复用原路径，并将位置投影限制在停止时的执行段。
需要重新规划时，用原路径推导当前/下一条未完成作业线的方向，跳过已经完成的区域和轮次；
只缩短中断轮次，后续完整轮次仍使用原起终点。连接段和转弯朝向不作为扫描方向。

检查点仅在内存中，不保证调度进程重启后的续跑。任务 ID、区域/安全参数、地图坐标系或
对齐角变化，或机器人离停止位置超过 0.5 米时，不继承旧进度；显式选取不同起终点也按新规划处理。
新任务按区域配置的 `x/-x/y/-y` 开始。继续旧任务应使用同一任务 ID；创建新任务不会自动续跑。
`TaskConfigModel.continuation` 是规划内部数据，不修改协议、区域配置或 APP 中保存的方向。

- 任务不启动：优先看 `scheduler_node.py` 中 `handle_task_config` / `handle_task_command`。  
- 地图查询异常：看 `handle_map_catalog_request` 和 `map_registry` 持久化文件。  
- 路径预览异常：看 `handle_path_plan_request`、`planner_adapter.py`、预览图生成逻辑。  
- 底盘无动作：看 `cmd_vel` 转发、底盘驱动话题连接和 `chassis_driver` 日志。

## 5.1 稀疏全局路径与局部导航段

`planner_adapter.py` 对 MST27 固定设置 `output_point_spacing=0`，使用 `path_geometry.py`
只移除语义相同且同向共线的中间点，保留转折、回退、连接/区域/轮次边界。
全局 ROS/协议路径是关键点折线；旧协议 `output_point_spacing` 保留兼容，不再补全局点。

`scheduler_node.py` 的方向段以边为单位推进，共享边界点。切片和进度投影按弧长二分定位当前窗口；
工艺状态使用当前所在边，任务进度按路径米数计算。续跑检查点仍以原路径和弧长恢复。
`active_segment_point_spacing` 默认 0.3 米、最小 0.01 米；逐边采样不会越过中间角点。
路径/版本/窗口/间距改变才重建局部 ROS Path，周期发布继续使用同一结果。

离线测试 `test_sparse_segments.py` 覆盖边界插值、方向段完整性、属性切换、转弯合段、
续跑、缓存及间距校验；`test_sparse_planner.py` 对照 Python/C++ 后端稀疏/密集输出的
几何、长度、轮次和全线碰撞检查。`test/benchmark_sparse_segments.py --baseline-ref <提交>`
使用同一输入对比旧版切片/投影；本机微基准不能代替板端或控制器实测。

## 5.2 连续手动控制的接收隔离

`sl_linka_adapter.py` 的普通控制业务使用独立有界 FIFO，满时返回失败，不能把已排队的
开关操作静默覆盖。急停解除可等待服务，但不再阻塞 TCP 接收；急停触发仍立即处理。
手动非零最新值队列、独立零速队列保持不变，请求有效期为1秒。
所有异步控制绑定连接有效性，断连或会话替换后，在途业务不能恢复发布旧非零输出。
`test_continuous_manual.py` 用真实本机 TCP/Protobuf 验证普通控制阻塞期间仍持续应用遥控、
松手优先停车及业务队列有界拒绝。不修改超时门限，不以缓存旧速度代替新控制输入。

## 6. 板端日志与轻量指标

`grinder_scheduler` 初始化默认日志等级为 WARN，保留警告及错误，下面的 INFO 抽样和耗时日志默认不输出。
排查时可调用 `/grinder_scheduler/set_logger_level`，将 `rosout` 调回 `info`；重启后恢复 WARN。
`robot_diff_drive.launch` 单独为 move_base 加载 `cfg/rosconsole_navigation.config`，默认 ERROR，
同时适用于 TEB/RPP；其余节点不继承该配置。定位、通信和碰撞保护仍按原逻辑运行。
这两个等级不控制 Python `print`、Livox SDK 自身日志、原始485文件或指标JSON。

`sl_linka_adapter.py` 默认每秒只记录一帧 RX 帧头样本，每 60 秒输出接收量统计。异常仍按原有 WARN/ERROR 日志记录。诊断时可在启动参数中设置 `sl_linka_rx_diagnostic_logging:=true`，结束后恢复为 `false`；该开关只记录帧头，不解析或记录 Protobuf 载荷。

`metrics_registry.py` 在各进程内维护计数器、Gauge、固定毫秒桶直方图和最多 128 条事件。`metrics_exporter.py` 每 10 秒将当前快照原子替换为一个 JSON 文件。协议收发线程和地图构建函数不会直接写指标文件。计数器为进程启动以来的累计值；直方图的 `buckets_ms` 与 `histogram_bucket_limits_ms` 一一对应，`over_max_ms` 表示超过最大桶的样本数。

启动示例：

```bash
roslaunch grinder_scheduler scheduler.launch enable_metrics_node:=true metrics_export_dir:=/tmp/grinder_metrics
```

默认会导出 `grinder_scheduler.json` 和 `super_lio_mode_manager.json`。可选系统采样节点 `grinder_metrics.json` 每秒采样 CPU、内存和进程数量，每 10 秒查询一次 ROS 节点数。文件名固定且只保留最新快照，不会因重启或长期运行无限增长。长期留档可由外部采集程序定期复制快照。`metrics_export_enabled:=false` 可关闭调度器和模式管理器的指标导出；`metrics_export_interval_sec:=30` 可降低写盘频率，需要每秒快照时可设为 `1`。

常用指标：`sl_rx_frames`、`control_queue_wait_ms`、`ordered_queue_wait_ms`、`map_build_ms`、`map_chunks_built`、`map_chunks_sent`、`super_lio_start_mapping_*`、`super_lio_save_map_*`、`super_lio_start_localization_*`、`super_lio_localization_ready` 和 `super_lio_timeout`。其中 `map_chunks_built` 表示已构建，`map_chunks_sent` 只统计发送调用成功的分片，不代表 APP 已确认接收。

地图请求的耗时可分开查看：`map_load_ms`、`map_grid_convert_ms`、`map_colorize_ms`、`map_vertical_flip_ms`、`map_resize_ms`、`map_png_encode_ms`、`map_chunk_serialize_ms`。`map_build_ms` 覆盖整个构建函数。当前没有旋转处理，`map_vertical_flip_ms` 是上下翻转；原尺寸输出不会产生 `map_resize_ms` 样本。原始栅格编码走 `map_grid_encode_ms`，不会产生 PNG 指标。

发送侧的 `map_frame_pack_ms`、`map_socket_send_ms` 是单分片样本；`map_send_total_ms`、`map_socket_send_total_ms`、`map_request_total_ms` 是单次地图请求样本。`map_send_total_ms` 包括帧打包、发送锁等待与 `sendall` 阻塞，`map_socket_send_total_ms` 是所有分片 `sendall` 耗时之和，`map_request_total_ms` 从开始构建算到最后一片发送调用返回。它们都不代表 APP 已收到或重组完成。

`scheduler.launch` 的指标快照默认写到 `/tmp/grinder_metrics/grinder_scheduler.json`；Super-LIO 模式管理和可选系统采样节点分别写同目录中的 `super_lio_mode_manager.json`、`grinder_metrics.json`。`start_grinder_super_lio_base.sh` 的普通控制台日志写到 `catkin_ws/logs/super_lio_base_时间戳/scheduler.log`，ROS 自身日志仍在 `~/.ros/log/`。指标文件只保留最新快照；需要历史曲线时应由外部程序定期采集。

### APP 系统状态遥测

调度器通过既有 DeviceStatusReport 增加左右电机故障码（可选 int32 字段26/27，值保持底盘 int16 原值）及 SystemHealth 消息（字段28）。仅在底盘反馈3秒内且已连接时发送故障码；旧驱动缺字段则不发送，不能将缺失值当作0正常。

`system_health.py`独立线程每秒采集CPU差分占用率、CPU/SoC温度、基于MemAvailable的内存占用、运行工程所在磁盘空间和开机时长。状态发送线程只读缓存；首个CPU差分样本及读取失败项不填字段，超过5秒无成功采样时不填资源消息。该只读链路不修改使能、速度或急停。APP须同步更新协议解析；旧APP忽略新增字段，新APP连接旧板端显示未提供。部署需要一起更新scheduler_node.py、system_health.py、Python生成协议文件并正常重载调度器；不需要修改ROS消息或底盘寄存器布局。

纯Python回归：`python3 catkin_ws/src/grinder_scheduler/test/test_system_health.py`。设备运行期间只读验证资源可用性不能替代更新后的端到端验收。

系统状态页面采用2秒刷新后，资源线程改为2秒采样/5秒缓存过期。SystemHealth字段9～11新增板端蜂窝信号百分比、连接状态、接入制式。ModemManager只读命令每条400ms超时，仅单个明确设备且SignalQuality标记recent时报告信号值；无工具、无设备、模糊多设备、缓存信号或外置路由器无接口时不填百分比。该诊断不会拨号、发AT命令或修改无线配置。数据含义见[官方SignalQuality说明](https://www.freedesktop.org/software/ModemManager/doc/latest/ModemManager/gdbus-org.freedesktop.ModemManager1.Modem.html)。APP链路RTT利用既有会话ACK测量，无新增设备控制请求。

### 历史浏览与区域工艺参数（2026-11-02）

历史轨迹页只查询记录、实际轨迹、底图和尺寸，不再通过 PathPointPlan 重新规划。没有归档规划路径时明确显示仅有实际轨迹；板端执行活动、RUNNING或PAUSED期间拒绝外部规划，避免改 task_id、切图或把状态改成READY。

每个作业区域独立拥有一套工艺，身份为 `(map_id, region_id)`。工艺类型、磨料、混凝土强度、压力、行走速度和磨盘RPM，保存在地图overlay的 `regions.work_regions[].process_settings`；不再保存/采用地图共用工艺。
工艺编辑页只修改当前工作区草稿；保存工作区时，以确定的region_id提交 SettingsWriteRequest.process/process_map_id/process_region_id，并等待精确ACK及区域回显。保存失败不会标记工作区完成。新地图保存时复制所有区域及各自工艺；复制失败不能报告全部保存成功。区域改名/修改几何保留自己的工艺，删除区域同时删除其工艺。
MapRegionPointResponse.work_regions[].region.process 返回各区域工艺，APP按map_id与region_id恢复各自草稿，不从邻区或地图共用值补齐。

任务启动先校验参与区域工艺并复制为执行快照，从当前实际路径进度对应的区域恢复速度与RPM；缺失工艺的区域明确阻止启动，需在APP重新确认并保存。旧地图/旧区域不会自动继承先前错误的地图共用工艺或手动遥控速度。
运行中按当前路径投影点的region/lap选择工艺，不按前视目标点提前切区。同一区域重复轮次沿用该区域工艺；连接段不切换工艺，继续原有转场磨盘策略。
切区先关闭轮速转发并清零，再应用区域速度与磨盘RPM，成功后恢复；失败暂停，STOP/急停可撤销在途切换。恢复任务前也先核对当前位置区域工艺。切区不重新读地图文件。
TaskRuntimeSettings的显式临时任务参数保持更高优先级；切区更新其恢复基准，结束临时设置后回到当前区域工艺。普通遥控设置不修改区域工艺；运行/暂停期间全局底盘设置及工艺编辑仍拒绝。
压力等元数据仅保存/显示，现有底盘执行链实际下发行走速度和磨盘RPM；速度仍受导航安全上限约束。

需配套更新APP与板端。旧区域须逐个确认保存工艺。本次仅源码和离线验证，未部署、未安装、未进行实机作业验收。
回归：scheduler `test_history_process.py`、`test_execution_settings.py`、`test_task_runtime_parameters.py`、急停/续跑回归；APP `core:tcp/MapProcessSettingsTest`、`feature:map/MapProcessSettingsTest`。


## DXF 图纸地基独立障碍层

`drawing_obstacles.py` 管理地图向量 JSON、分片草稿、pending 保存日志和任务版本授权；`drawing_scheduler.py` 接入现有保存、查询、任务门禁及路径检查。协议 `0x053A/0x053B`，必须配套更新生成 SDK。APP 负责 ASCII DXF 解析、配准和地基选择，板端复核控制点残差、多边形和地图绑定；确认页只做几何检查，划分工作区后的原有预览才规划。

图纸保存于 `persist_state_dir/drawing_layers/<map-id-sha256>.json`，仅在全局规划请求的栅格副本上合入，绝不写 MapService、overlay.npy、原始或导航 PGM，不改变局部 costmap、扫描或 RPP。建图预览、任务规划、任务启动和恢复均不追加图纸专用的路径/起步连接/车体包络复检；图纸边界及膨胀仍合入全局规划栅格，规划算法原有碰撞处理保持不变。旧普通禁区和任务障碍仍走原有逻辑。地图几何/原始资产变更使层失效，默认阻止任务；明确跳过仅绑定该地图、任务和图纸版本，停止/结束撤销。

保存地图的图纸绑定直接读取对应保存栅格的几何，不使用启动时尚未就绪或属于其他地图的 MapService。底图未就绪不会持久化为图纸失效；旧版误标为“地图底图或坐标几何发生变化”的图纸，仅在完整几何和非空原始地图版本均匹配、且没有 pending/损坏状态时自动恢复，不改控制点、变换或图纸版本。

上传单片48KiB、总量8MiB，SHA-256校验；半包5分钟、完整草稿24小时。最终地图保存前先持久化pending，保存成功再发布版本；相同request_id可重试，重启后pending也支持原请求重试。中断/pending期间禁止启动，地图几何改变须重新对齐。两个文件保存不是跨文件原子事务。关闭现有persist_state_enabled时图纸不能跨重启保留。

离线回归逐文件运行：`test/test_drawing_obstacles.py`、`test/test_drawing_integration.py`、`test/test_drawing_planning.py`，另跑 `test_live_map_save.py`、`test_planner_preprocessing.py`。纯编辑环境需NumPy、OpenCV、protobuf及规划器测试依赖；scheduler/chassis源码目录和Python SDK加入PYTHONPATH。真实ROS/运动急停回归需Linux与Catkin，Windows缺少termios，不能代替实机验证。

未部署硬件。现场须用真实二维基础DXF独立校核配准点外的位置、车体转弯/起步连接、窄通道、地图失效、保存中断恢复和原有禁区；核对发布前后PGM/overlay哈希不变。完整APP交互及现场清单见配套APP仓库 `docs/drawing_obstacles.md`。

区域查询超过65000字节时在原0x052F响应上分片回读，单片48KiB、总量16MiB、独立transfer_id及SHA-256；APP校验完整快照后一次发布，断线清理半包。小响应保持原格式。测试覆盖分片顺序/重复/损坏/过期及完整地基回读。


## 固定地图编辑草稿（2026-11-04）

APP 扫描页“下一步”使用 `MapDraftRequest/Response (0x053C/0x053D)`：先在 LIO 帧边界冻结处理，二维 mapper 等待最后发布的扫描并完成当前回环重投影，再保存 PCD、PGM、YAML，确认建图组已停止后才进入编辑。二维投影算法、分辨率和高度范围不由本功能修改。

- `FREEZE` 创建固定 `draft_` 地图 ID；草稿不进入正式目录响应、不上传云端，也不能启动作业。需要 `persist_state_enabled: true`。
- `QUERY` 查询最新未发布草稿，`RESUME` 激活它；正常退出不放弃，重新进入继续使用同一底图。`RESTART` 明确开始新扫描，旧草稿资产保留，不将旧编辑套用到新地图。`DISCARD` 放弃草稿但不自动删除资产。
- 编辑、图纸校验、路径预览、区域工艺和保存请求携带固定底图 revision 与编辑版本。旧版本拒绝，重复编辑请求返回已提交结果，不重复增加版本。
- 编辑层保持独立；每次确认的编辑先写不可变状态快照，再原子更新 `.map-edit-drafts.json` 中的指针和幂等回复。每份草稿保留当前及上一份完整编辑快照，避免全栅格快照随操作无限增长。
- 最后 `MapSaveRequest` 发布当前草稿：先确认元信息、编辑层和图纸层已持久化，再提交发布状态，之后才允许目录可见及云端上传。发布与导航加载分开；不自动启动定位、不自动切换导航地图。
- 草稿底图直接读取捕获资产包中的 PGM/YAML，避免使用桥接节点最后一次接收的另一帧。新扫描清空桥接缓存并拒绝前一会话的迟到地图。

APP 需要同步升级协议。APP 本地尚未提交的擦除、撤销记录、工作区编辑和地图名按草稿 ID/底图 revision 保存；已确认编辑以设备端状态为准。换平板时只恢复设备已确认的内容。本流程不修改实时避障算法；停止建图组后不会假定 `/scan` 或位姿仍在发布。现有回环三维保存坐标一致性限制仍需按根 README 的说明验证，本改动不改回环后端。

离线回归：`test_map_edit_drafts.py`、`test_live_map_save.py`、`test_super_lio_mode_manager.py`、`test_history_process.py`、`test_drawing_*.py`。Linux 构建 `PROFILE=mapping`；独立 ROS 验证运行 mapping 包 `test/test_mapper_freeze_ros.py`，只启动端口11379的私有 master 和合成点云 mapper，不连接实车。


图纸校验数值精度修复（2026-11-04）：MapChunk/Pose2D的原点、朝向及分辨率以float32传输，APP再写入DrawingMapBinding的double字段。几何比较须识别这种量化往返，不能将-43.6与-43.599998474121094判成底图变化。只容许相同float32编码且有上限的舍入差异（原点不超过1mm、朝向不超过0.001度，分辨率保持原阈值）；尺寸、坐标系和资产版本仍校验，非有限值拒绝。`test_drawing_obstacles.py`覆盖该实测值、草稿提交全链路、真实变更和大坐标量化混叠。修复后运行中的调度进程需要加载新代码，APP可重新检查，无须因该误报重选控制点。
