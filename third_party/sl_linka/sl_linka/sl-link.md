# SL-LinkA 单下位机协议

## 1. 总览

SL-LinkA 当前面向单下位机模型：
- `0x01` 为 `APP`
- `0x10` 为唯一 `LOWER`
- `0x20` 已删除

协议能力分为 6 类：
- 配网
- 参数读写
- 运行数据读取
- 控制指令下发
- 任务调度
- 地图与视频服务

## 2. 帧格式

| 偏移 | 字段 | 长度 | 说明 |
|------|------|------|------|
| 0 | `STX1` | 1 | `0xFD` |
| 1 | `STX2` | 1 | `0x55` |
| 2 | `VER` | 1 | 协议版本 |
| 3 | `FLAGS` | 1 | 标志位 |
| 4 | `SEQ` | 2 | 序列号，小端 |
| 6 | `ACK_SEQ` | 2 | 确认序列号，小端 |
| 8 | `SRC_ID` | 1 | 源设备 ID |
| 9 | `DST_ID` | 1 | 目标设备 ID |
| 10 | `COMP_ID` | 1 | 组件 ID |
| 11 | `MSG_ID` | 2 | 消息 ID，小端 |
| 13 | `LEN` | 2 | Payload 长度，小端 |
| 15 | `PAYLOAD` | N | protobuf 数据 |
| 15+N | `CRC16` | 2 | CRC16-CCITT |
| 17+N | `TAIL` | 1 | `0xFE` |

## 3. 设备与组件

### 3.1 设备 ID

| ID | 名称 |
|----|------|
| `0x01` | `APP` |
| `0x10` | `LOWER` |
| `0xFF` | `BROADCAST` |

### 3.2 组件 ID

| ID | 名称 |
|----|------|
| `0x00` | `COMP_SYSTEM` |
| `0x04` | `COMP_WIFI` |
| `0x05` | `COMP_SETTINGS` |
| `0x06` | `COMP_MEDIA` |
| `0x07` | `COMP_CONTROL` |
| `0x08` | `COMP_SCHEDULER` |

## 4. 消息定义

### 4.1 配网

| MSG_ID | 名称 | 方向 | 说明 |
|--------|------|------|------|
| `0x0201` | `WifiConfig` | `APP -> LOWER` | 下发 WiFi 配网 |
| `0x0202` | `WifiStatusReport` | `LOWER -> APP` | 返回配网结果 |

```proto
message WifiConfig {
  string ssid = 1;
  string password = 2;
}

message WifiStatusReport {
  WifiResult result = 1;
  string message = 2;
}
```

### 4.2 参数读取与写入

| MSG_ID | 名称 | 方向 | 说明 |
|--------|------|------|------|
| `0x0203` | `SettingsReadRequest` | `APP -> LOWER` | 读取参数 |
| `0x0204` | `SettingsReadResponse` | `LOWER -> APP` | 返回参数 |
| `0x0205` | `SettingsWriteRequest` | `APP -> LOWER` | 写入参数 |
| `0x0206` | `SettingsWriteResponse` | `LOWER -> APP` | 返回写入结果 |

参数按两组组织：

#### 底盘相关 `ChassisSettings`

- 运行速度 `run_speed`，单位 m/s；设置为 `0` 时立即停止车辆并禁止任务及手动移动，重新设置为正值后恢复速度许可
- 磨盘转速 `disc_speed_rpm`
- 磨盘开关 `disc_enabled`
- 模式设置 `work_mode`
- 最大转弯速度比例 `max_turn_speed_ratio`，范围 `(0,1]`；例如 70% 传 `0.7`

#### 地图相关 `MapSettings`

- 小车宽度 `vehicle_width`
- 小车长度 `vehicle_length`
- 默认路径间距 `default_path_spacing`
- 转弯半径 `turn_radius`
- 重叠比例 `overlap_ratio`
- 膨胀半径 `inflation_radius`
- 障碍区域 `obstacle_regions[]`
- 运行区域 `work_regions[]`

区域统一使用多边形结构：

```proto
message PolygonPoint {
  float x = 1;
  float y = 2;
}

enum RegionType {
  REGION_TYPE_UNKNOWN = 0;
  REGION_TYPE_WORK = 1;
  REGION_TYPE_OBSTACLE = 2;
  REGION_TYPE_ERASE = 3;
  REGION_TYPE_CROP = 4;
}

message PolygonRegion {
  string name = 1;
  repeated PolygonPoint points = 2;
  string region_id = 3;
  uint32 priority = 4;
  bool enabled = 5;
  uint32 color_argb = 6;
  bool closed = 7;
  RegionType region_type = 8;
}
```

因此 `障碍区域1/2/3/N` 与 `运行区域1/2/3/N` 都通过数组表达，不再为每个区域单独定义消息。

`PolygonRegion` 扩展字段说明：

| 字段 | 含义 |
|------|------|
| `region_id` | 区域唯一 ID，用于更新/删除同一区域（推荐 UUID 或业务唯一字符串） |
| `priority` | 区域优先级，冲突时高优先级覆盖低优先级 |
| `enabled` | 区域是否启用；`false` 时仅保留不参与生效 |
| `color_argb` | 区域显示颜色（`0xAARRGGBB`） |
| `closed` | 多边形是否闭合；建议 `true` 才作为有效区域 |
| `region_type` | 区域类型（工作区/障碍区/擦除区/裁减区） |

### 4.3 上位机读取的数据

#### 4.3.1 运行状态

| MSG_ID | 名称 | 方向 | 说明 |
|--------|------|------|------|
| `0x0301` | `DeviceStatusReport` | `LOWER -> APP` | 自动 `1Hz` 上报运行状态 |

字段包括：
- 左轮速度 `left_wheel_speed`
- 右轮速度 `right_wheel_speed`
- 磨盘转速 `disc_speed_rpm`
- 磨盘开关 `disc_enabled`
- 工作模式 `work_mode`
- 磨盘升降状态 `disc_lift_state`
- 照明灯 `light_enabled`
- 位置 `position`
- 底盘开关状态 `chassis_enabled`
- 地图旋转角 `alignment_yaw_deg`
- 定位质量 `localization_quality_available`、`localization_quality`（0～100）

角度约定：
- `position.heading_deg`：小车在当前上报坐标系下的朝向角，`0°` 表示地图坐标 `+X`，`90°` 表示地图坐标 `+Y`，逆时针为正。
- `alignment_yaw_deg`：地图从原始 `map` 坐标系旋转到 `map_aligned` 坐标系的角度；当 `frame_id=map_aligned` 时，`position.x/y/heading_deg` 已经应用该旋转角。
- APP 若以“屏幕正上方为 0°、顺时针为正”渲染车辆图标，可使用 `screen_angle = 90 - position.heading_deg`，不要再额外叠加 `alignment_yaw_deg`。

#### 4.3.2 视像头画面

| MSG_ID | 名称 | 方向 | 说明 |
|--------|------|------|------|
| `0x0302` | `CameraFrameRequest` | `APP -> LOWER` | 请求画面快照 |
| `0x0303` | `CameraFrameChunk` | `LOWER -> APP` | 返回画面分片 |

说明：
- 当前按“快照 + 分片回传”定义
- `CameraFrameChunk.data` 为单片数据
- `chunk_index` 与 `total_chunks` 用于上位机重组完整画面

#### 4.3.3 地图

| MSG_ID | 名称 | 方向 | 说明 |
|--------|------|------|------|
| `0x0304` | `MapRequest` | `APP -> LOWER` | 请求地图快照 |
| `0x0305` | `MapChunk` | `LOWER -> APP` | 返回地图分片 |

说明：
- 当前按“快照 + 分片回传”定义
- `MapRequest.map_id` 可选；空值或 `LIVE_MAP` 表示请求实时原始地图，非空历史地图 ID 表示读取该地图保存的离线原始栅格并分片返回
- `MapRequest.snapshot` 表示请求当前地图快照；`max_chunk_size` 控制单个分片大小；`map_id` 为字符串地图 ID
- `MapRequest` 返回原始 `map` 坐标系栅格，不应用 `alignment_yaw`、APP 旋转角或二者差值
- `MapPreviewRequest` 同样基于原始 `map` 坐标生成图片，可按请求叠加区域，但不应用三个旋转角；三个角度字段仅作为元数据返回给 APP
- 支持 `OCCUPANCY_GRID / PNG / JSON` 三种编码标识
- `MapChunk` 统一携带地图信息：`width/height/resolution/origin/frame_id/preview_scale_x/preview_scale_y`

## 5. 控制指令

| MSG_ID | 名称 | 方向 | 说明 |
|--------|------|------|------|
| `0x0401` | `ControlCommand` | `APP -> LOWER` | 控制指令 |
| `0x0402` | `ControlCommandResponse` | `LOWER -> APP` | 控制结果 |

当前支持的控制项：
- 磨盘升降控制 `DiscLiftControl`
- 磨盘控制 `DiscControl`（`enabled + speed_rpm`）
- 照明控制 `LightingControl`
- 手动控制 `ManualDriveControl`
  - 兼容旧模式：`motion + speed_ratio`
  - 摇杆模式：`remote_x + remote_y + speed_ratio`
  - 可选 `max_speed_mps`：本次指令的最大直行速度，单位 m/s；程序限制在 `0～max_chassis_run_speed`
  - 可选 `max_turn_speed_ratio`：最大转弯差速比例，范围 `(0,1]`；例如 70% 传 `0.7`
  - `max_speed_mps` 和 `max_turn_speed_ratio` 未传或传 `0` 时保持原有速度计算逻辑
  - 坐标约定：
    - 上：`(0.00, -1.00)`
    - 下：`(0.00, 1.00)`
    - 左：`(-1.00, 0.00)`
    - 右：`(1.00, 0.00)`

摇杆控制示例：

```json
{
  "manual_drive": {
    "remote_x": 0.2,
    "remote_y": -0.8,
    "speed_ratio": 1.0,
    "max_speed_mps": 0.15,
    "max_turn_speed_ratio": 0.7
  }
}
```
- 底盘开关控制 `ChassisPowerControl`

## 5.1 调度扩展

| MSG_ID | 名称 | 方向 | 说明 |
|--------|------|------|------|
| `0x0500` | `TaskConfig` | `APP -> LOWER` | 下发任务配置 |
| `0x0501` | `TaskConfigResponse` | `LOWER -> APP` | 返回任务配置结果 |
| `0x0502` | `TaskCommand` | `APP -> LOWER` | 开始/暂停/继续/停止 |
| `0x0503` | `TaskCommandResponse` | `LOWER -> APP` | 返回任务控制结果 |
| `0x0504` | `TaskStatusReport` | `LOWER -> APP` | 周期上报任务状态 |
| `0x0505` | `TaskPathRequest` | `APP -> LOWER` | 请求规划路径 |
| `0x0506` | `TaskPathChunk` | `LOWER -> APP` | 返回路径分片 |
| `0x0507` | `MapPreviewRequest` | `APP -> LOWER` | 请求地图缩略图 |
| `0x0508` | `MapPreviewResponse` | `LOWER -> APP` | 返回地图缩略图与编辑层 |
| `0x0509` | `MapEditCommand` | `APP -> LOWER` | 下发地图编辑 |
| `0x050A` | `MapEditResponse` | `LOWER -> APP` | 返回地图编辑结果 |
| `0x050B` | `MapEditStatusReport` | `LOWER -> APP` | 上报地图编辑应用状态 |
| `0x050C` | `VideoStreamInfoRequest` | `APP -> LOWER` | 请求视频流信息 |
| `0x050D` | `VideoStreamInfoResponse` | `LOWER -> APP` | 返回视频流信息 |
| `0x050E` | `PathPlanRequest` | `APP -> LOWER` | 请求立即路径规划 |
| `0x050F` | `PathPlanResponse` | `LOWER -> APP` | 返回路径规划摘要结果 |
| `0x0510` | `MapSyncRequest` | `APP -> LOWER` | 地图同步请求（雷达上传/下载） |
| `0x0511` | `MapSyncResponse` | `LOWER -> APP` | 返回地图同步结果 |
| `0x0512` | `MapModeRequest` | `APP -> LOWER` | 请求雷达切换建图/定位模式 |
| `0x0513` | `MapModeResponse` | `LOWER -> APP` | 返回模式切换指令处理结果 |
| `0x0514` | `MapCatalogRequest` | `APP -> LOWER` | 请求本地地图列表（名称/数量） |
| `0x0515` | `MapCatalogResponse` | `LOWER -> APP` | 返回本地地图列表（含面积/预计耗时/缩略图） |
| `0x0516` | `MapDeleteRequest` | `APP -> LOWER` | 按 map_id 删除本地地图 |
| `0x0517` | `MapDeleteResponse` | `LOWER -> APP` | 返回删除结果 |
| `0x0518` | `MapSaveRequest` | `APP -> LOWER` | 请求从雷达保存地图到本地 |
| `0x0519` | `MapSaveResponse` | `LOWER -> APP` | 返回保存结果与 map_id（含面积/预计耗时/创建时间） |
| `0x051A` | `MapMetricsRequest` | `APP -> LOWER` | 按 map_id 请求地图面积与预计耗时 |
| `0x051B` | `MapMetricsResponse` | `LOWER -> APP` | 返回 map_id/name + 工作区域面积/耗时明细 |
| `0x051C` | `TaskResultRequest` | `APP -> LOWER` | 请求上次任务执行结果（可带 map_id/task_id） |
| `0x051D` | `TaskResultResponse` | `LOWER -> APP` | 返回任务结果图与区域执行结果明细 |
| `0x051E` | `LiveMapCacheClearRequest` | `APP -> LOWER` | 请求清除 LIVE_MAP 缓存（区域缓存/原始地图缓存） |
| `0x051F` | `LiveMapCacheClearResponse` | `LOWER -> APP` | 返回清理结果 |
| `0x0520` | `RadarMapCacheClearRequest` | `APP -> LOWER` | 请求清除雷达侧地图缓存/地图数据 |
| `0x0521` | `RadarMapCacheClearResponse` | `LOWER -> APP` | 返回雷达清图指令下发结果 |
| `0x0522` | `MapImportToRadarRequest` | `APP -> LOWER` | 按 map_id 将本地保存地图导入雷达（先清雷达地图缓存，导入后进入纯定位模式） |
| `0x0523` | `MapImportToRadarResponse` | `LOWER -> APP` | 返回地图导入雷达结果 |
| `0x0524` | `MapAlignmentRequest` | `APP -> LOWER` | 设置独立的 APP 地图规划/显示旋转角，不覆盖雷达启动对齐角 |
| `0x0525` | `MapAlignmentResponse` | `LOWER -> APP` | 返回 APP 旋转角、雷达对齐角及二者差值 |
| `0x0526` | `RadarSystemStatusRequest` | `APP -> LOWER` | 查询 `/slamware_ros_sdk_server_node/system_status` 最新状态 |
| `0x0527` | `RadarSystemStatusResponse` | `LOWER -> APP` | 返回状态可用性、原始状态字符串和时间戳 |
| `0x0528` | `RadarMapSyncRequest` | `APP -> LOWER` | 触发 `/slamware_ros_sdk_server_node/sync_map` 地图同步 |
| `0x0529` | `RadarMapSyncResponse` | `LOWER -> APP` | 返回地图同步消息是否成功发布 |
| `0x052A` | `RadarRelocalizationRequest` | `APP -> LOWER` | 调用 `/slamware_ros_sdk_server_node/relocalization` 触发地图重定位 |
| `0x052B` | `RadarRelocalizationResponse` | `LOWER -> APP` | 返回雷达是否受理异步重定位请求及当前聚合状态 |
| `0x052C` | `RadarRelocalizationStatusRequest` | `APP -> LOWER` | 查询雷达地图重定位聚合状态 |
| `0x052D` | `RadarRelocalizationStatusResponse` | `LOWER -> APP` | 返回原始状态、系统状态和聚合结果 |

#### 5.1.1 新增协议速查（建议优先对接）

- `TaskConfig/TaskConfigResponse`：`0x0500/0x0501`
- `PathPlanRequest/PathPlanResponse`：`0x050E/0x050F`
- `MapCatalog/MapSave/MapDelete/MapMetrics`：`0x0514~0x051B`
- `TaskResultRequest/TaskResultResponse`：`0x051C/0x051D`
- `LiveMapCacheClearRequest/Response`：`0x051E/0x051F`
- `RadarMapCacheClearRequest/Response`：`0x0520/0x0521`
- `MapImportToRadarRequest/Response`：`0x0522/0x0523`
- `RadarRelocalizationRequest/Response`：`0x052A/0x052B`
- `RadarRelocalizationStatusRequest/Response`：`0x052C/0x052D`

`TaskConfig (0x0500)` 关键字段：

| 字段 | 说明 |
|------|------|
| `task_id` | 必填，任务唯一 ID，用于绑定地图、临时障碍区、路径及执行状态 |
| `map_id` | 任务绑定地图 ID（空值表示当前运行地图） |
| `selected_work_region_ids[]` | 本次执行区域 ID 列表 |
| `region_repeats[]` | 每区域重复次数（`{region_id, repeat}`，默认 `repeat=1`） |
| `obstacle_regions[]` | 当前任务绑定的临时障碍区完整列表；重新下发时整体替换，传空列表表示清空 |

`PathPlanRequest (0x050E)` 关键字段：

| 字段 | 说明 |
|------|------|
| `request_id` | 请求跟踪 ID |
| `map_id` | 目标地图 ID；空值或 `LIVE_MAP` 表示使用实时地图路径规划，非空历史地图 ID 表示使用该地图保存的离线栅格与区域状态进行路径规划 |
| `global_direction` | 首条作业线方向，支持 `x` / `-x` / `y` / `-y`；正负号在地图规划旋转后的坐标轴上生效，非法或缺省默认 `x` |
| `start_pose` / `end_pose` | 可选起终点（不传由 LOWER 自动选取） |

`TaskConfig.obstacle_regions` 仅表示该任务绑定的临时障碍区，不会写入地图公共 overlay，也不会影响
其他任务。处理 `PathPlanRequest` 时，LOWER 按 `map_id + task_id` 读取这些临时障碍区，在地图公共
障碍区基础上追加后参与路径规划。路径规划预览图使用相同的合并结果；普通地图预览不显示任务临时
障碍区。

任务配置示例：

```json
{
  "taskId": "task_001",
  "mapId": "map_001",
  "selectedWorkRegionIds": ["work_001"],
  "obstacleRegions": [
    {
      "regionId": "task_obstacle_001",
      "name": "临时堆料区",
      "points": [
        {"x": 1.0, "y": 1.0},
        {"x": 2.0, "y": 1.0},
        {"x": 2.0, "y": 2.0},
        {"x": 1.0, "y": 2.0}
      ]
    }
  ]
}
```

删除部分障碍区时，重新下发 `TaskConfig` 并只保留仍需使用的 `obstacleRegions`。清空当前任务全部
临时障碍区时传空列表：

```json
{
  "taskId": "task_001",
  "mapId": "map_001",
  "selectedWorkRegionIds": ["work_001"],
  "obstacleRegions": []
}
```

任务临时障碍区配置持久化到：

```text
temp/grinder_scheduler_state/task_obstacle_regions.json
```

`TaskResultRequest (0x051C)` 关键字段：

| 字段 | 说明 |
|------|------|
| `map_id` | 查询目标地图 ID（可空；不传时可根据 `task_id` 从任务绑定中反查对应地图） |
| `task_id` | 查询目标任务 ID（可空；仅传 `task_id` 时按任务绑定的 `map_id` 查询；两者都空时返回最近任务结果） |

`TaskResultResponse (0x051D)` 关键字段：

| 字段 | 说明 |
|------|------|
| `map_id` / `task_id` | 返回结果对应地图与任务 |
| `final_state` / `all_completed` / `stop_reason` | 任务结束状态 |
| `image_data` / `image_format` / `image_width` / `image_height` | 任务结果图（二进制） |
| `selected_work_region_ids[]` | 本次任务实际执行区域 |
| `region_results[]` | 每区域目标遍数、执行遍数、是否完成、未完成原因 |

说明：如果指定 `map_id` 但该地图尚无任务执行结果，LOWER 会返回 `RESULT_SUCCESS`，`message` 为 `fallback_to_map_thumbnail: ...`，并在 `image_data` 中返回该 `map_id` 对应的地图缩略图；此时 `all_completed=false`、`stop_reason=no_task_result`、`region_results[]` 为空。

`LiveMapCacheClearRequest (0x051E)` 关键字段：

| 字段 | 说明 |
|------|------|
| （无） | 该请求不带参数；清理范围由 LOWER（调度程序）本地配置决定 |

`LiveMapCacheClearResponse (0x051F)` 关键字段：

| 字段 | 说明 |
|------|------|
| `result` / `message` | 清理执行结果 |

`RadarMapCacheClearRequest (0x0520)` 关键字段：

| 字段 | 说明 |
|------|------|
| （无） | 该请求不带参数；LOWER 收到后向雷达 SDK 发布清图命令，并在清图指令发出后切换雷达到建图模式 |

`RadarMapCacheClearResponse (0x0521)` 关键字段：

| 字段 | 说明 |
|------|------|
| `result` / `message` | 雷达清图与建图模式切换指令下发结果；成功表示已向雷达 SDK 发送清图请求并发送建图模式请求，不代表雷达地图已重新构建完成 |

`MapImportToRadarRequest (0x0522)` 关键字段：

| 字段 | 说明 |
|------|------|
| `map_id` | 必填；本地地图列表中的地图 ID，LOWER 根据该 ID 查找 `.stcm` 文件，先清除雷达地图缓存，再导入雷达，导入成功后下发进入纯定位模式 |

`MapImportToRadarResponse (0x0523)` 关键字段：

| 字段 | 说明 |
|------|------|
| `result` / `message` | 导入结果 |
| `map_id` / `map_name` | 实际导入的地图 ID 与名称 |
| `imported` | 是否已成功调用雷达 SDK 导入服务 |

`MapEditOperation` 推荐使用（独立操作，不混用）：

- `MAP_EDIT_OP_UPSERT_WORK_REGION`：新增/更新工作区
- `MAP_EDIT_OP_UPSERT_OBSTACLE_REGION`：新增/更新障碍区
- `MAP_EDIT_OP_UPSERT_ERASE_REGION`：新增/更新擦除区
- `MAP_EDIT_OP_UPSERT_CROP_REGION`：新增/更新裁减区
- `MAP_EDIT_OP_DELETE_REGION`：删除区域
- `MAP_EDIT_OP_PAINT_FREE / OCCUPIED / UNKNOWN`：即时刷图

`MapSyncOperation` 语义（同步）：

- `MAP_SYNC_OP_DOWNLOAD_FROM_AURORA`：从雷达下载地图到本地并注册 `map_id`
- `MAP_SYNC_OP_UPLOAD_TO_AURORA`：按 `map_id` 从本地记录地图上传到雷达

本地地图管理新增独立消息（不再依赖 `MapSyncOperation`，并以 `map_id` 作为唯一操作标识）：

- `MapSaveRequest/Response`：从雷达保存地图到本地（支持中文地图名 + 时间戳），并记录当前任务工作区总面积、预计耗时与地图旋转角；`map_id` 强制唯一，同一 `map_id` 再次保存时会先删除旧地图文件、旧区域状态和旧任务绑定，再保存最新数据
- `MapSaveResponse.created_at`：地图创建时间（`YYYY-MM-DD HH:MM:SS`，精确到秒）
- `MapCatalogRequest/Response`：查询本地地图名称与数量，并返回地图元信息（面积/预计耗时/缩略图base64）
- `MapDeleteRequest/Response`：按 `map_id` 删除地图
- `MapMetricsRequest/Response`：按 `map_id` 查询地图区域指标，返回 `map_name` 与区域明细
- `TaskResultRequest/Response`：按 `map_id/task_id` 查询任务执行结果（结果图 + 区域遍数完成情况）
- `MapMetricsResponse.region_metrics[]`：工作区域明细（`region_id`、`region_name`、`repeat`、`area_m2`、`estimated_time_h`）
- `LiveMapCacheClearRequest/Response`：清除 LIVE_MAP 缓存（区域状态 + live_map 目录缓存）
- `RadarMapCacheClearRequest/Response`：清除雷达侧地图缓存/地图数据（调度程序转发到 `/slamware_ros_sdk_server_node/clear_map`，随后向 `/slamware_ros_sdk_server_node/set_map_update` 下发建图模式）
- `MapImportToRadarRequest/Response`：按 `map_id` 将本地保存的 `.stcm` 地图导入雷达；导入前先向 `/slamware_ros_sdk_server_node/clear_map` 下发清图，再调用 `/slamware_ros_sdk_server_node/sync_set_stcm`，成功后向 `/slamware_ros_sdk_server_node/set_map_localization` 下发纯定位模式
- `MapAlignmentRequest/Response`：按 `map_id` 保存 APP 地图旋转角及其与雷达 `alignment_yaw` 的角差；路径规划角为 `alignment_yaw_deg + rotation_alignment_delta_deg`，LOWER 保持地图和区域在原始 `map` 坐标系并将该角度直接传给 mst27，生成的路径也保持原始 `map` 坐标和朝向；`map_id` 为空时作用于当前地图
- `RadarRelocalizationRequest/Response`：请求无参数，LOWER 调用 `/slamware_ros_sdk_server_node/relocalization`；`accepted=true` 仅表示雷达已受理，不能表示地图重定位已经成功
- `RadarRelocalizationStatusRequest/Response`：查询稳定聚合结果；只有 `RelocalizationSucceed` 会聚合为成功，`DeviceRunning` 仅作为系统状态返回，odom 数据不参与也不返回

`PathPlanResponse` 地图信息字段（用于上位机直接渲染坐标）：

- `width / height`
- `resolution`
- `origin (x, y, heading_deg)`
- `frame_id`
- `preview_scale_x / preview_scale_y`
- `total_work_area_m2`（工作区总面积，平方米）
- `estimated_time_s`（本次规划预计执行时间，秒，`<0` 表示暂不可估计）

### 5.2 坐标系约定（当前工程）

当前联调配置下，地图与导航主链路统一使用以下 TF 关系：

- `slamware_map -> odom -> base_link -> laser`

说明：

- `frame_id` 表示坐标所属坐标系名；返回坐标（`origin`、路径点、区域点）都需与该 `frame_id` 一起使用。
- `MapChunk (0x0305)`、`MapPreviewResponse (0x0508)`、`PathPlanResponse (0x050F)` 均携带 `frame_id`、地图几何信息（`width/height/resolution/origin`）以及地图旋转信息（`alignment_yaw_deg/app_rotation_deg/rotation_alignment_delta_deg`）。
- 历史示例或测试中可能出现 `map`，以运行时返回的 `frame_id` 为准。

## 6. 时序约定

### 6.1 配网

1. APP 发送 `WifiConfig`
2. LOWER 返回 `WifiStatusReport`

### 6.2 参数读取

1. APP 发送 `SettingsReadRequest`
2. LOWER 返回 `SettingsReadResponse`

### 6.3 参数写入

1. APP 发送 `SettingsWriteRequest`
2. LOWER 应用参数
3. LOWER 返回 `SettingsWriteResponse`

### 6.4 状态上报

1. LOWER 建立链路后自动上报 `DeviceStatusReport`
2. 默认周期固定为 `1Hz`

### 6.5 媒体/地图读取

1. APP 发送 `CameraFrameRequest` 或 `MapRequest`；`MapRequest.map_id` 为空取实时地图，非空取历史保存地图
2. LOWER 按分片返回 `CameraFrameChunk` 或 `MapChunk`
3. APP 根据 `chunk_index/total_chunks` 重组数据

### 6.6 控制下发

1. APP 发送 `ControlCommand`
2. LOWER 执行后返回 `ControlCommandResponse`

### 6.7 任务调度

1. APP 发送 `TaskConfig`
2. LOWER 返回 `TaskConfigResponse`（成功后缓存任务参数并准备规划）
3. APP 发送 `TaskCommand`
4. LOWER 返回 `TaskCommandResponse`
5. LOWER 周期性发送 `TaskStatusReport`
6. APP 如需全路径则发送 `TaskPathRequest`
7. LOWER 按 `TaskPathChunk` 分片返回
8. APP 可发送 `TaskResultRequest`
9. LOWER 返回 `TaskResultResponse`（任务结果图 + 区域明细）

`TaskConfig` 地图绑定字段：

| 字段 | 含义 |
|------|------|
| `map_id` | 任务绑定地图 ID。传空表示使用当前运行地图；传非空时 LOWER 会校验是否与当前地图一致，不一致将拒绝。 |
| `selected_work_region_ids` | 本次任务要执行的工作区 `region_id` 列表（由安卓侧勾选）。为空时默认执行全部有效工作区。 |
| `region_repeats` | 每个工作区的重复执行次数数组，元素为 `{region_id, repeat}`，`repeat>=1`。未配置的区域默认执行 `1` 次。 |

实现约定：
- 实时建图阶段，当前运行地图 ID 固定为 `LIVE_MAP`（可通过参数 `~live_map_id` 调整）。
- 执行 `MapSave` 成功后，LOWER 会切换到保存生成的真实地图 ID（如 `20260427_102229`）。

`TaskStatusReport` 补充字段（用于任务看板）：

| 字段 | 含义 |
|------|------|
| `progress` | 任务进度（0~1 小数，不是百分比） |
| `total_work_area_m2` | 工作区总面积（平方米） |
| `remaining_work_area_m2` | 剩余待覆盖面积（平方米） |
| `remaining_time_s` | 预计剩余时间（秒，`<0` 表示暂不可估计） |
| `current_region_id` | 当前执行中的基础工作区 `region_id`（不带 `__lap_N` 后缀） |
| `current_region_repeat_index` | 当前区域执行到第几遍（从 `1` 开始） |
| `current_region_repeat_total` | 当前区域总共需要执行几遍 |
| `alignment_yaw_deg` | 地图从原始 `map` 坐标系旋转到当前上报坐标系的角度，单位度 |

`PathPlanRequest` 方向字段：

| 字段 | 含义 |
|------|------|
| `global_direction` | 首条作业线方向，支持 `x` / `-x` / `y` / `-y`。例如规划角为 `30°` 时，`x` 沿 `30°`，`-x` 沿 `210°`，`y` 沿 `120°`，`-y` 沿 `300°`。未传或非法值时默认 `x`。 |

`PathPlanResponse` 关键字段（新增关注）：

| 字段 | 含义 |
|------|------|
| `path_version` | 路径版本号 |
| `path_point_count` | 路径点数 |
| `path_length_m` | 路径总长度（米） |
| `total_work_area_m2` | 工作区总面积（平方米） |
| `estimated_time_s` | 预计执行时间（秒） |
| `preview_image` / `preview_format` | 路径预览图（二进制 + 格式） |
| `alignment_yaw_deg` | 当前预览图实际使用的地图对齐旋转角，单位度 |
| `app_rotation_deg` | APP 最后明确设置并随地图保存的旋转角，单位度 |
| `rotation_alignment_delta_deg` | `app_rotation_deg - alignment_yaw_deg` 的原始有符号差值，不做角度范围规范化 |

### 6.8 地图预览与编辑

1. APP 发送 `MapPreviewRequest`
2. LOWER 返回 `MapPreviewResponse`
3. APP 发送 `MapEditCommand`
4. LOWER 返回 `MapEditResponse`
5. LOWER 可额外发送 `MapEditStatusReport`

`MapPreviewRequest` 地图选择字段：

| 字段 | 含义 |
|------|------|
| `map_id` | 区域信息/预览读取目标地图 ID。传空表示当前运行地图；传非空时 LOWER 会校验是否与当前地图一致。 |

本地地图管理消息（`0x0514~0x051B`）入参建议：

| 消息 | 核心入参 | 说明 |
|------|----------|------|
| `MapCatalogRequest` | 无必填 | 可按实现决定是否附带缩略图 |
| `MapSaveRequest` | `map_name`（建议）、`map_id`（可选）、`has_rotation_deg`、`rotation_deg` | 地图保存成功后返回 `map_id`、面积、耗时、创建时间；`has_rotation_deg=true` 时保存 `rotation_deg`，否则沿用当前地图已记录旋转角；同一 `map_id` 再次保存按替换处理，只保留最新地图数据 |
| `MapDeleteRequest` | `map_id` | 按 ID 删除，不依赖本地文件名 |
| `MapMetricsRequest` | `map_id` | 返回区域面积/耗时明细（单位小时） |

`MapAlignmentRequest (0x0524)` 关键字段：

| 字段 | 含义 |
|------|------|
| `map_id` | 目标地图 ID。为空时使用当前地图；为 `LIVE_MAP` 时记录实时地图的 APP 角度；为历史地图时写入该地图记录 |
| `rotation_deg` | APP 设置的、基于地图 X 轴正方向的规划/显示旋转角，单位度，APP 图片坐标中顺时针为正；保存其与 `alignment_yaw` 的差值，但不会覆盖 `alignment_yaw` |

`MapAlignmentResponse (0x0525)` 关键字段：

| 字段 | 含义 |
|------|------|
| `map_id` | 实际应用的地图 ID |
| `rotation_deg` | 实际保存的 APP 显示旋转角，单位度 |
| `rotation_rad` | 实际保存的 APP 显示旋转角，单位弧度 |
| `alignment_yaw_deg` | 雷达启动时确定且未被本接口修改的地图对齐旋转角，单位度 |
| `rotation_alignment_delta_deg` | 保存的 `rotation_deg - alignment_yaw_deg` 原始有符号差值，不做范围规范化；路径规划直接使用 `alignment_yaw_deg + rotation_alignment_delta_deg`，结果应与 APP 下发的 `rotation_deg` 一致 |

`MapEditCommand` 新增细化控制字段：

| 字段 | 含义 |
|------|------|
| `map_id` | 地图编辑目标地图 ID。传空表示当前运行地图；传非空时 LOWER 会校验是否与当前地图一致，不一致直接拒绝编辑。 |
| `target_region_id` | 指定目标区域 ID（尤其用于 `DELETE_REGION` / 定点更新） |
| `target_region_type` | 指定目标区域类型（工作区/障碍区） |
| `expected_map_version` | 乐观锁版本；不匹配时可拒绝编辑，避免并发覆盖 |
| `dry_run` | 仅校验不落库；用于预检查编辑是否合法 |

`MapPreviewResponse` 关键字段说明：

| 字段 | 含义 |
|------|------|
| `result` / `message` | 结果码与说明 |
| `map_version` | 地图版本号（编辑或刷新后递增） |
| `width` / `height` | 地图原始栅格尺寸（cell） |
| `resolution` | 地图分辨率（米/格） |
| `origin.x` / `origin.y` | 地图原点在 `frame_id` 坐标系下的位置 |
| `frame_id` | 地图坐标系名（如 `slamware_map`） |
| `preview_scale_x` / `preview_scale_y` | 预览图相对原始栅格的缩放比例（像素换算可用） |
| `image_data` | 缩略图二进制（`jpg/png`） |
| `overlay_json` | 编辑层 JSON（区域、多边形、掩膜等） |
| `alignment_yaw_deg` | 当前预览图实际使用的地图对齐旋转角，单位度 |
| `app_rotation_deg` | APP 最后明确设置并随地图保存的旋转角，单位度 |
| `rotation_alignment_delta_deg` | `app_rotation_deg - alignment_yaw_deg` 的原始有符号差值，不做角度范围规范化 |

### 6.9 视频流信息

1. APP 发送 `VideoStreamInfoRequest`
2. LOWER 返回 `VideoStreamInfoResponse`

### 6.10 安卓请求地图示例（联调用）

1. APP 发送 `MapPreviewRequest`（`MSG_ID=0x0507`）
2. LOWER 返回 `MapPreviewResponse`（`MSG_ID=0x0508`）

请求示例（逻辑字段）：

```json
{
  "msg_id": "0x0507",
  "payload": {
    "max_edge": 512,
    "image_format": "jpg",
    "include_overlay": true,
    "map_id": "20260427_102229"
  }
}
```

响应示例（逻辑字段）：

```json
{
  "msg_id": "0x0508",
  "payload": {
    "result": "RESULT_OK",
    "message": "ok",
    "map_version": 12,
    "width": 260,
    "height": 211,
    "resolution": 0.05,
    "origin": {"x": -5.35, "y": -5.70, "heading_deg": 0.0},
    "frame_id": "slamware_map",
    "preview_scale_x": 1.0,
    "preview_scale_y": 1.0,
    "image_data": "<jpg/png binary>",
    "overlay_json": "{\"work_regions\":[],\"obstacle_regions\":[],\"updated_at\":1776145413}"
  }
}
```

说明：
- `image_data` 是二进制，不是文本；TCP 层会按帧长度分包传输，接收端按协议重组。
- 安卓端可直接使用 `width/height/resolution/origin` 做像素坐标与地图米制坐标换算。
- 如仅需底图，可请求时设置 `include_overlay=false`。

### 6.11 雷达模式切换

1. APP 发送 `MapModeRequest`
2. LOWER 发布到 Aurora ROS 话题：
   - `MAP_MODE_MAPPING` -> `/slamware_ros_sdk_server_node/set_map_update`
   - `MAP_MODE_LOCALIZATION` -> `/slamware_ros_sdk_server_node/set_map_localization`
3. LOWER 返回 `MapModeResponse`

请求示例（进入建图模式）：

```json
{
  "msg_id": "0x0512",
  "payload": {
    "mode": "MAP_MODE_MAPPING",
    "enabled": true,
    "map_kind": 0
  }
}
```

### 6.12 雷达地图重定位

APP 发送空的 `RadarRelocalizationRequest (0x052A)`：

```json
{}
```

LOWER 调用：

```bash
rosservice call /slamware_ros_sdk_server_node/relocalization "{}"
```

返回 `RadarRelocalizationResponse (0x052B)`：

```json
{
  "result": "RESULT_SUCCESS",
  "message": "radar_relocalization_accepted",
  "accepted": true,
  "status": "running"
}
```

`accepted=true` 只表示异步重定位请求已被雷达接受，不表示重定位已经完成。

APP 可发送空的 `RadarRelocalizationStatusRequest (0x052C)` 查询最终状态：

```json
{}
```

返回 `RadarRelocalizationStatusResponse (0x052D)`：

```json
{
  "result": "RESULT_SUCCESS",
  "message": "radar_relocalization_status_ready",
  "available": true,
  "status": "succeeded",
  "raw_status": "RelocalizationNone",
  "system_status": "DeviceRunning",
  "timestamp_ns": 1780000000000000000
}
```

聚合规则：

- `RelocalizationSucceed`、`RelocalizationFailed`、`RelocalizationCanceled` 分别聚合为 `succeeded`、`failed`、`canceled`，结果保持到下一次重定位请求。
- `RelocalizationNone` 不覆盖已经记录的有效结果。
- `DeviceRunning` 仅作为系统状态返回，不会推断为 `succeeded`；odom 数据和速度不参与聚合，也不在该响应中返回。
- `status` 取值为 `idle`、`running`、`succeeded`、`failed`、`canceled`。

请求示例（进入定位模式）：

```json
{
  "msg_id": "0x0512",
  "payload": {
    "mode": "MAP_MODE_LOCALIZATION",
    "enabled": true,
    "map_kind": 0
  }
}
```

## 7. 方向约定

- 下发类消息：`SRC_ID=0x01`，`DST_ID=0x10`
- 上报类消息：`SRC_ID=0x10`，`DST_ID=0x01`

## 8. 端口约定

- 默认 TCP 端口使用 `8002`
- 模拟器与测试客户端如未显式指定端口，均按 `8002` 处理
