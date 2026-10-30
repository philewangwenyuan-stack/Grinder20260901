# grinder_scheduler 包说明

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
接收500ms、控制每帧发送500ms、会话3秒门限不变。回归包括心跳写入受阻时遥控/停车继续接收，
以及 EOF、发送超时、接收遥控超时的原因归属；不连接真实设备。

本文档说明 `catkin_ws/src/grinder_scheduler` 目录下各文件/子目录的作用，便于开发、联调和排障。

## 原始地图取图与缓存

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
（限制0.1～5秒），控制通道默认0.5秒（限制0.05～0.5秒），分别配置
`sl_linka_data_send_timeout_sec` 和 `sl_linka_control_send_timeout_sec`。
非阻塞发送期间检查会话有效性，超过期限关闭连接，不能在部分帧后直接重发整帧。
遥控指令500ms有效期、断线停车及急停保护保持原样。

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
手动非零最新值队列、独立零速队列及 500ms 请求有效期保持不变。
所有异步控制绑定连接有效性，断连或会话替换后，在途业务不能恢复发布旧非零输出。
`test_continuous_manual.py` 用真实本机 TCP/Protobuf 验证普通控制阻塞期间仍持续应用遥控、
松手优先停车及业务队列有界拒绝。不修改超时门限，不以缓存旧速度代替新控制输入。

## 6. 板端日志与轻量指标

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
