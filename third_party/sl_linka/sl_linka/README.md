# SL-LinkA

`SL-LinkA` 是一套用于 `APP(0x01)` 与单下位机 `LOWER(0x10)` 通信的轻量协议实现。当前这份协议已经扩展到研磨机器人场景，除基础配置和控制外，还覆盖了任务调度、地图预览/编辑、路径回传和视频流查询。

当前协议源仅保留一个文件：[proto/sl_link.proto](/home/chersxir/work/Grinder/doc/SL-LinkA/proto/sl_link.proto)。

## 概览

| 项目 | 说明 |
|---|---|
| 协议名称 | `SL-LinkA` |
| 通信模型 | `APP <-> LOWER` 单下位机 |
| 主要传输 | TCP |
| 默认端口 | `8002` |
| 编码方式 | 帧协议 + protobuf |
| 当前场景 | 研磨机器人调度、地图、视频、底盘控制 |

## 能力范围

| 能力分类 | 内容 |
|---|---|
| WiFi 配网 | 下发 WiFi 参数，回传配网结果 |
| 参数读写 | 读取/写入底盘参数、地图参数 |
| 运行状态 | 自动上报设备状态、位置和工作状态 |
| 控制指令 | 手动控制、磨盘升降、灯光、底盘使能 |
| 任务调度 | 下发任务配置、开始/暂停/继续/停止、回传任务状态 |
| 地图服务 | 请求缩略图、获取地图元数据、下发地图编辑命令 |
| 视频服务 | 查询推流地址、流状态和基础视频参数 |

## 设备与组件

### 设备 ID

| ID | 设备 |
|---|---|
| `0x01` | `APP` |
| `0x10` | `LOWER` |
| `0xFF` | `BROADCAST` |

### 组件 ID

| ID | 组件 | 说明 |
|---|---|---|
| `0x00` | `COMP_SYSTEM` | 系统状态、设备上报 |
| `0x04` | `COMP_WIFI` | WiFi 配网 |
| `0x05` | `COMP_SETTINGS` | 参数读写 |
| `0x06` | `COMP_MEDIA` | 图像、地图、多媒体相关 |
| `0x07` | `COMP_CONTROL` | 控制指令 |
| `0x08` | `COMP_SCHEDULER` | 调度、地图编辑、路径、视频流查询 |

## 消息总表

### 基础协议

| MSG_ID | 名称 | 方向 | 组件 | 说明 |
|---|---|---|---|---|
| `0x0201` | `WifiConfig` | `APP -> LOWER` | `COMP_WIFI` | 下发 WiFi 配网 |
| `0x0202` | `WifiStatusReport` | `LOWER -> APP` | `COMP_WIFI` | 返回 WiFi 配网结果 |
| `0x0203` | `SettingsReadRequest` | `APP -> LOWER` | `COMP_SETTINGS` | 读取底盘/地图参数 |
| `0x0204` | `SettingsReadResponse` | `LOWER -> APP` | `COMP_SETTINGS` | 返回底盘/地图参数 |
| `0x0205` | `SettingsWriteRequest` | `APP -> LOWER` | `COMP_SETTINGS` | 写入底盘/地图参数 |
| `0x0206` | `SettingsWriteResponse` | `LOWER -> APP` | `COMP_SETTINGS` | 返回写入结果 |
| `0x0301` | `DeviceStatusReport` | `LOWER -> APP` | `COMP_SYSTEM` | 自动 `1Hz` 上报运行状态 |
| `0x0302` | `CameraFrameRequest` | `APP -> LOWER` | `COMP_MEDIA` | 请求相机画面快照 |
| `0x0303` | `CameraFrameChunk` | `LOWER -> APP` | `COMP_MEDIA` | 返回相机画面分片 |
| `0x0304` | `MapRequest` | `APP -> LOWER` | `COMP_MEDIA` | 请求地图快照 |
| `0x0305` | `MapChunk` | `LOWER -> APP` | `COMP_MEDIA` | 返回地图分片 |
| `0x0401` | `ControlCommand` | `APP -> LOWER` | `COMP_CONTROL` | 下发控制指令 |
| `0x0402` | `ControlCommandResponse` | `LOWER -> APP` | `COMP_CONTROL` | 返回控制结果 |

### 调度扩展

| MSG_ID | 名称 | 方向 | 组件 | 说明 |
|---|---|---|---|---|
| `0x0501` | `TaskConfig` | `APP -> LOWER` | `COMP_SCHEDULER` | 下发作业区域、障碍区和规划参数 |
| `0x0502` | `TaskCommand` | `APP -> LOWER` | `COMP_SCHEDULER` | 下发开始、暂停、继续、停止 |
| `0x0503` | `TaskCommandResponse` | `LOWER -> APP` | `COMP_SCHEDULER` | 返回任务控制结果 |
| `0x0504` | `TaskStatusReport` | `LOWER -> APP` | `COMP_SCHEDULER` | 上报任务状态、进度与当前位置 |
| `0x0505` | `TaskPathRequest` | `APP -> LOWER` | `COMP_SCHEDULER` | 请求完整规划路径 |
| `0x0506` | `TaskPathChunk` | `LOWER -> APP` | `COMP_SCHEDULER` | 返回路径分片 |
| `0x0507` | `MapPreviewRequest` | `APP -> LOWER` | `COMP_SCHEDULER` | 请求地图缩略图、元数据和编辑层 |
| `0x0508` | `MapPreviewResponse` | `LOWER -> APP` | `COMP_SCHEDULER` | 返回地图缩略图与编辑层 |
| `0x0509` | `MapEditCommand` | `APP -> LOWER` | `COMP_SCHEDULER` | 下发地图编辑操作 |
| `0x050A` | `MapEditResponse` | `LOWER -> APP` | `COMP_SCHEDULER` | 返回地图编辑结果 |
| `0x050B` | `MapEditStatusReport` | `LOWER -> APP` | `COMP_SCHEDULER` | 上报地图版本和规划层应用状态 |
| `0x050C` | `VideoStreamInfoRequest` | `APP -> LOWER` | `COMP_SCHEDULER` | 请求视频流信息 |
| `0x050D` | `VideoStreamInfoResponse` | `LOWER -> APP` | `COMP_SCHEDULER` | 返回视频流地址和在线状态 |
| `0x050E` | `PathPlanRequest` | `APP -> LOWER` | `COMP_SCHEDULER` | 请求立即执行路径规划，可选同时返回路径分片 |
| `0x050F` | `PathPlanResponse` | `LOWER -> APP` | `COMP_SCHEDULER` | 返回规划结果摘要（是否成功、版本、点数、里程） |
| `0x0510` | `MapSyncRequest` | `APP -> LOWER` | `COMP_SCHEDULER` | 请求 STCM 下载/上传与导航地图刷新 |
| `0x0511` | `MapSyncResponse` | `LOWER -> APP` | `COMP_SCHEDULER` | 返回地图同步结果 |
| `0x0512` | `MapModeRequest` | `APP -> LOWER` | `COMP_SCHEDULER` | 请求雷达切换建图/定位模式 |
| `0x0513` | `MapModeResponse` | `LOWER -> APP` | `COMP_SCHEDULER` | 返回模式切换结果 |
| `0x0526` | `RadarSystemStatusRequest` | `APP -> LOWER` | `COMP_SYSTEM` | 查询雷达最新系统状态 |
| `0x0527` | `RadarSystemStatusResponse` | `LOWER -> APP` | `COMP_SYSTEM` | 返回雷达状态和时间戳 |
| `0x0528` | `RadarMapSyncRequest` | `APP -> LOWER` | `COMP_SYSTEM` | 触发雷达同步当前地图 |
| `0x0529` | `RadarMapSyncResponse` | `LOWER -> APP` | `COMP_SYSTEM` | 返回雷达地图同步指令发送结果 |
| `0x052A` | `RadarRelocalizationRequest` | `APP -> LOWER` | `COMP_SYSTEM` | 触发雷达地图重定位 |
| `0x052B` | `RadarRelocalizationResponse` | `LOWER -> APP` | `COMP_SYSTEM` | 返回雷达是否受理重定位及当前聚合状态 |
| `0x052C` | `RadarRelocalizationStatusRequest` | `APP -> LOWER` | `COMP_SYSTEM` | 查询雷达地图重定位聚合状态 |
| `0x052D` | `RadarRelocalizationStatusResponse` | `LOWER -> APP` | `COMP_SYSTEM` | 返回原始状态、系统状态和聚合结果 |

## 参数模型

`SettingsRead/Write` 当前分为两组参数。

### `ChassisSettings`

| 字段类别 | 内容 |
|---|---|
| 运动参数 | `run_speed` |
| 磨盘参数 | `disc_speed_rpm`、`disc_enabled` |
| 模式参数 | `work_mode` |

### `MapSettings`

| 字段类别 | 内容 |
|---|---|
| 车体尺寸 | `vehicle_width`、`vehicle_length` |
| 规划参数 | `default_path_spacing`、`turn_radius`、`overlap_ratio`、`inflation_radius` |
| 区域集合 | `obstacle_regions[]`、`work_regions[]` |

### 区域表达

| 结构 | 说明 |
|---|---|
| `PolygonPoint` | 单个二维点，字段为 `x`、`y` |
| `PolygonRegion` | 具名多边形区域，字段为 `name` 和 `points[]` |

这意味着障碍区和作业区都通过数组表达，不需要为每个区域单独定义消息。

## 运行数据

### `DeviceStatusReport`

默认 `1Hz` 上报，当前关注字段如下：

| 字段类别 | 内容 |
|---|---|
| 轮速 | 左轮速度、右轮速度 |
| 磨盘 | 磨盘转速、磨盘开关、磨盘升降状态 |
| 工作模式 | 自动/手动等模式状态 |
| 照明 | 灯光开关状态 |
| 底盘 | 底盘使能状态 |
| 位置 | 当前 `x/y/heading`、定位协方差、定位质量（0～100） |

### 快照读取

| 请求 | 响应 | 说明 |
|---|---|---|
| `CameraFrameRequest` | `CameraFrameChunk` | 相机画面通过分片回传 |
| `MapRequest` | `MapChunk` | 地图快照通过分片回传 |
| `RadarSystemStatusRequest` | `RadarSystemStatusResponse` | 返回雷达 `/slamware_ros_sdk_server_node/system_status` 最新状态 |
| `RadarMapSyncRequest` | `RadarMapSyncResponse` | 向雷达 `/slamware_ros_sdk_server_node/sync_map` 发布一次同步请求 |

`RadarSystemStatusResponse` 中，`available` 表示调度程序是否已经收到雷达状态，
`status` 保留雷达原始状态字符串，`timestamp_ns` 保留雷达消息时间戳。尚未收到状态时
返回 `RESULT_FAILED` 和 `radar_system_status_unavailable`。

## 控制指令

`ControlCommand` 当前支持的控制项如下：

| 控制类型 | 内容 |
|---|---|
| 磨盘升降 | 升起、下降 |
| 照明控制 | 打开、关闭 |
| 手动运动 | 前进、后退、前左、前右、后左、后右 |
| 底盘使能 | 开启、关闭 |

## 调度扩展

调度扩展由 `COMP_SCHEDULER(0x08)` 承载，主要服务于 [grinder_scheduler](/home/chersxir/work/Grinder/catkin_ws/src/grinder_scheduler/README.md) 节点。

### 任务相关

| 消息 | 作用 | 关键内容 |
|---|---|---|
| `TaskConfig` | 下发任务配置 | 工作区、障碍区、路径间距、转弯半径、重叠率、膨胀半径 |
| `TaskCommand` | 下发任务控制 | `START`、`PAUSE`、`RESUME`、`STOP` |
| `TaskCommandResponse` | 返回任务控制结果 | `result`、`message`、`task_id` |
| `TaskStatusReport` | 周期上报任务状态 | `state`、`progress`、`position`、`path_version` |
| `TaskPathRequest` | 请求完整路径 | 支持设置分片大小 |
| `TaskPathChunk` | 回传路径数据 | 当前由 UTF-8 JSON 分片组成 |
| `PathPlanRequest` | 请求立即规划 | 可带 `force_replan` 和 `return_path_chunks` |
| `PathPlanResponse` | 返回规划结果 | `result`、`path_version`、`path_point_count`、`path_length_m` |

### 地图与视频相关

| 消息 | 作用 | 关键内容 |
|---|---|---|
| `MapPreviewRequest` | 请求地图缩略图 | 最大边长、图片格式、是否包含覆盖层 |
| `MapPreviewResponse` | 返回地图预览 | 缩略图、元数据、覆盖层 JSON |
| `MapEditCommand` | 下发地图编辑 | 区域更新、局部擦除、障碍修补 |
| `MapEditResponse` | 返回地图编辑结果 | `result`、`message`、`map_version` |
| `MapEditStatusReport` | 上报地图编辑应用状态 | 地图版本、是否已应用到规划层 |
| `VideoStreamInfoRequest` | 请求视频流信息 | 查询预览流状态 |
| `VideoStreamInfoResponse` | 返回视频流信息 | 流地址、编码、宽高、在线状态 |

### 当前支持的地图编辑操作

| 操作 | 说明 |
|---|---|
| `UPSERT_WORK_REGION` | 新增或更新工作区域 |
| `UPSERT_OBSTACLE_REGION` | 新增或更新障碍区域 |
| `DELETE_REGION` | 删除指定区域 |
| `PAINT_FREE` | 将局部区域改为空闲 |
| `PAINT_OCCUPIED` | 将局部区域改为障碍 |
| `PAINT_UNKNOWN` | 将局部区域改为未知 |
| `CLEAR_OVERLAY_PATCH` | 清除覆盖层局部修补 |

## 三端同步状态

### 已同步文件

| 端 | 位置 | 状态 |
|---|---|---|
| 协议源 | [proto/sl_link.proto](/home/chersxir/work/Grinder/doc/SL-LinkA/proto/sl_link.proto) | 已更新 |
| Python | [sdk/python/sl_link/message_gen/sl_link_pb2.py](/home/chersxir/work/Grinder/doc/SL-LinkA/sdk/python/sl_link/message_gen/sl_link_pb2.py) | 已更新 |
| Android 生成代码 | [sdk/android/src/message_gen/sl_link/SlLink.java](/home/chersxir/work/Grinder/doc/SL-LinkA/sdk/android/src/message_gen/sl_link/SlLink.java) | 已更新 |
| Android builder | [sdk/android/src/frame/sl_link/SlMessageBuilder.kt](/home/chersxir/work/Grinder/doc/SL-LinkA/sdk/android/src/frame/sl_link/SlMessageBuilder.kt) | 已同步 |
| Android 示例 | [sdk/android/examples/Example.kt](/home/chersxir/work/Grinder/doc/SL-LinkA/sdk/android/examples/Example.kt) | 已同步 |
| Embedded 生成代码 | [sdk/embedded/message_gen/sl_link.pb.c](/home/chersxir/work/Grinder/doc/SL-LinkA/sdk/embedded/message_gen/sl_link.pb.c) | 需按需生成 |
| Embedded 头文件 | [sdk/embedded/message_gen/sl_link.pb.h](/home/chersxir/work/Grinder/doc/SL-LinkA/sdk/embedded/message_gen/sl_link.pb.h) | 需按需生成 |

### 同步说明

| 项目 | 说明 |
|---|---|
| Python/ROS | 已可直接配合 `grinder_scheduler` 使用（含 PathPlanRequest/Response） |
| Android | 已可直接使用 `SlLink.java`（当前环境 `protoc 3.6.1` 生成 Java 代码） |
| Embedded | 如有协议变更，执行 `generate_proto.sh c` 更新 `pb.c/.h` |
| 后续维护 | 修改 `proto` 后建议三端一起生成，避免版本漂移 |

## 代码入口

| 类型 | 文件 |
|---|---|
| Android builder | [SlMessageBuilder.kt](/home/chersxir/work/Grinder/doc/SL-LinkA/sdk/android/src/frame/sl_link/SlMessageBuilder.kt) |
| Android example | [Example.kt](/home/chersxir/work/Grinder/doc/SL-LinkA/sdk/android/examples/Example.kt) |
| Python simulator handler | [messages.py](/home/chersxir/work/Grinder/doc/SL-LinkA/sdk/python/sl_link/frame/messages.py) |
| Python simulator server | [simulator.py](/home/chersxir/work/Grinder/doc/SL-LinkA/sdk/python/sl_link/frame/simulator.py) |
| Embedded example | [main.c](/home/chersxir/work/Grinder/doc/SL-LinkA/sdk/embedded/examples/main.c) |
| Embedded tests | [test_nanopb.c](/home/chersxir/work/Grinder/doc/SL-LinkA/sdk/embedded/tests/test_nanopb.c) |
| PyQt 调试工具 | [sl_linka_pyqt_debugger.py](/home/chersxir/work/Grinder/doc/SL-LinkA/sdk/python/tools/sl_linka_pyqt_debugger.py) |
| 集成说明 | [INTEGRATION_NOTES.md](/home/chersxir/work/Grinder/doc/SL-LinkA/sdk/INTEGRATION_NOTES.md) |

## PyQt 调试工具

用于现场联调 `SL-LinkA` TCP 链路，支持：

- 连接 `host:port`（默认 `127.0.0.1:8002`）
- 一键发送 `MapPreviewRequest` / `PathPlanRequest` / `MapSyncRequest` / `VideoStreamInfoRequest`
- 一键发送遥感控制 `remote_x / remote_y`
- 实时显示收包日志并解析常用响应
- 自动把地图缩略图、路径预览图保存到本地目录

安装依赖：

```bash
sudo apt-get install -y python3-pyqt5
```

启动命令：

```bash
cd /home/chersxir/work/Grinder/doc/SL-LinkA/sdk/python
python3 tools/sl_linka_pyqt_debugger.py
```

## 默认端口

| 项目 | 值 |
|---|---|
| TCP 服务端口 | `8002` |
| Python 模拟器监听 | `0.0.0.0:8002` |
| Python 测试客户端默认连接 | `127.0.0.1:8002` |

## 生成代码

### 脚本方式

```bash
./scripts/generate_proto.sh all
```

在当前开发机（`protoc 3.6.1`）下：

- `python`：正常生成 `sl_link_pb2.py`
- `kotlin/android`：脚本会自动走兼容模式，仅生成 `SlLink.java`
- 如需 `kotlin_out`/`lite` 产物，请升级到较新 `protoc`

### 手动同步时至少需要更新

| 端 | 需要更新的产物 |
|---|---|
| Python | `sl_link_pb2.py` |
| Android | `sdk/android/src/message_gen/sl_link/SlLink.java` |
| Embedded | `sdk/embedded/message_gen/sl_link.pb.c/.h` |

## 相关文档

| 文档 | 说明 |
|---|---|
| [sl-link.md](/home/chersxir/work/Grinder/doc/SL-LinkA/sl-link.md) | 更详细的协议说明 |
| [sdk/INTEGRATION_NOTES.md](/home/chersxir/work/Grinder/doc/SL-LinkA/sdk/INTEGRATION_NOTES.md) | 三端集成说明 |
| [grinder_scheduler README](/home/chersxir/work/Grinder/catkin_ws/src/grinder_scheduler/README.md) | 机器人调度节点如何消费这些消息 |
