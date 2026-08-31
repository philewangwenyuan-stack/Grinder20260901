# 地坪研磨机器人 MQTT 数据上报接口说明

本文档用于地坪研磨机器人调度程序与物联网平台之间的 MQTT 数据上报对接。机器人端作为 MQTT 客户端，连接平台 MQTT 服务，将实时定位、轨迹、实时速度、作业面积、设备状态、任务状态等数据上传到平台。路径规划预览图等图片类数据不做周期主动上传，仅在平台订阅/请求指定 Topic 时返回。

## 1. 平台 MQTT 连接信息

| 项目 | 内容 |
|------|------|
| MQTT 地址 | `14.18.91.10` |
| MQTT 端口 | `1883` |
| MQTT 版本 | MQTT 3.1.1 |
| Payload 编码 | UTF-8 JSON |
| 时间戳格式 | 毫秒级 Unix 时间戳字符串，例如 `"1725959546400"` |
| 设备编码 | `{devCode}`，物联网平台中的设备编码 |
| 用户名 | 物联网平台创建的 MQTT 用户名，例如 `MQTT` |
| accessKey | 物联网平台创建 MQTT 用户时生成 |
| accessSecret | 物联网平台创建 MQTT 用户时生成 |

MQTT 账户认证信息在物联网平台创建。创建完成后，需要将账户状态更改为“启用”，并在平台中给当前 MQTT 账户划分设备权限，使该账户具备对应 `{devCode}` 的数据上报权限。

安全要求：

- `accessKey/accessSecret` 属于敏感信息，不建议写入 git 仓库文档或提交到代码仓库。
- 工控机部署时建议通过本地配置文件或环境变量注入。
- 如果必须写入配置文件，应确保该配置文件不提交到仓库，并限制文件读取权限。

## 2. Topic 规范

平台当前基础 Topic：

| Topic | 方向 | Payload | 描述 |
|------|------|---------|------|
| `rg/cloud/deviceData/002/{devCode}` | 机器人 -> 平台 | JSON | 设备通用数据上报主题 |
| `rg/cloud/deviceState/002/{devCode}/online` | 机器人 -> 平台 | JSON | 设备在线状态上报主题 |
| `rg/cloud/deviceState/002/{devCode}/offline` | 机器人 -> 平台 | JSON | 设备离线状态上报主题 |
| `rg/cloud/deviceData/002/{devCode}` | 平台订阅 | JSON | 平台订阅设备数据 |

参考在线/离线 Topic 的后缀形式，机器人业务数据建议扩展以下类型 Topic：

| Topic | 方向 | Payload | 描述 |
|------|------|---------|------|
| `rg/cloud/deviceData/002/{devCode}/status` | 机器人 -> 平台 | JSON | 综合状态上报，包含任务、位置、速度、面积、设备状态 |
| `rg/cloud/deviceData/002/{devCode}/pose` | 机器人 -> 平台 | JSON | 实时定位上报 |
| `rg/cloud/deviceData/002/{devCode}/trajectory` | 机器人 -> 平台 | JSON | 轨迹点上报 |
| `rg/cloud/deviceData/002/{devCode}/task/start` | 机器人 -> 平台 | JSON | 任务开始事件上报 |
| `rg/cloud/deviceData/002/{devCode}/task/result` | 机器人 -> 平台 | JSON | 任务结束结果上报 |
| `rg/cloud/deviceData/002/{devCode}/map/info` | 机器人 -> 平台 | JSON | 地图响应，包含地图元信息和路径规划预览图，仅在平台订阅/请求时返回 |
| `rg/cloud/deviceData/002/{devCode}/event` | 机器人 -> 平台 | JSON | 告警、异常和关键事件上报 |
| `rg/cloud/deviceCommand/002/{devCode}/map/request` | 平台 -> 机器人 | JSON | 平台请求地图信息和路径规划预览图 |

说明：

- `{devCode}` 为物联网平台中的设备编码。
- 如果平台只支持基础 Topic，可全部统一通过 `rg/cloud/deviceData/002/{devCode}` 上报。
- 如果平台支持后缀 Topic，建议按业务类型使用上述扩展 Topic，便于服务器按类型处理。
- 地图类 Topic 不做周期主动上传，只有平台订阅/请求时，机器人再返回当前最新地图信息和路径规划预览图。
- 平台请求地图时，向 `rg/cloud/deviceCommand/002/{devCode}/map/request` 发布请求；机器人统一通过 `rg/cloud/deviceData/002/{devCode}/map/info` 返回。
- `reported` 中的 key 必须使用物联网平台配置的“属性编码”。
- `reported` 中的 value 为对应属性的测点值。

## 3. 设备数据上报

### 3.1 Topic

```text
rg/cloud/deviceData/002/{devCode}
```

### 3.2 Payload 格式

```json
{
  "reported": {
    "meterValue": "2141.1"
  },
  "timestamp": "1725959546400"
}
```

字段说明：

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `reported` | object | 是 | 设备属性数据集合 |
| `reported.{propertyCode}` | string/number/bool | 是 | `{propertyCode}` 为平台属性编码 |
| `timestamp` | string | 是 | 数据上报时间戳，毫秒 |

## 4. 在线状态上报

### 4.1 Topic

```text
rg/cloud/deviceState/002/{devCode}/online
```

### 4.2 Payload

```json
{
  "message": "设备在线",
  "timestamp": "1725953725936"
}
```

字段说明：

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `message` | string | 是 | 在线说明，建议固定为 `设备在线` |
| `timestamp` | string | 是 | 在线状态上报时间戳，毫秒 |

建议机器人 MQTT 客户端连接成功后立即上报一次在线状态。

## 5. 离线状态上报

### 5.1 Topic

```text
rg/cloud/deviceState/002/{devCode}/offline
```

### 5.2 Payload

```json
{
  "message": "设备因程序退出离线",
  "timestamp": "1725953725936"
}
```

字段说明：

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `message` | string | 是 | 离线原因，例如 `设备因程序退出离线`、`设备因网络断开离线` |
| `timestamp` | string | 是 | 离线状态上报时间戳，毫秒 |

建议配置 MQTT Last Will 到该离线 Topic，避免机器人异常断电或网络断开时平台无法感知离线。

Last Will 示例：

```json
{
  "message": "设备因网络断开离线",
  "timestamp": "1725953725936"
}
```

字段说明：

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `reported` | object | 是 | 平台属性数据集合，key 为平台属性编码 |
| `reported.taskId` | string | 否 | 当前任务 ID |
| `reported.taskState` | string | 是 | 当前任务状态，见“任务状态枚举建议” |
| `reported.taskProgress` | number | 是 | 任务进度百分比，范围 0-100 |
| `reported.taskMessage` | string | 否 | 任务状态说明或错误信息 |
| `reported.mapId` | string | 否 | 当前地图 ID |
| `reported.mapName` | string | 否 | 当前地图名称 |
| `reported.poseX` | number | 是 | 机器人 X 坐标，单位 m |
| `reported.poseY` | number | 是 | 机器人 Y 坐标，单位 m |
| `reported.poseHeading` | number | 是 | 机器人朝向，单位 deg |
| `reported.linearSpeed` | number | 否 | 机器人线速度，单位 m/s |
| `reported.angularSpeed` | number | 否 | 机器人角速度，单位 rad/s |
| `reported.leftWheelSpeed` | number | 否 | 左轮速度，单位 rpm |
| `reported.rightWheelSpeed` | number | 否 | 右轮速度，单位 rpm |
| `reported.totalWorkArea` | number | 否 | 总作业面积，单位 m2 |
| `reported.finishedWorkArea` | number | 否 | 已完成作业面积，单位 m2 |
| `reported.remainingWorkArea` | number | 否 | 剩余作业面积，单位 m2 |
| `reported.remainingTime` | number | 否 | 预计剩余时间，单位 s |
| `reported.chassisEnabled` | bool | 否 | 底盘是否使能 |
| `reported.taskEnable` | bool | 否 | 是否允许底盘处理导航 `/cmd_vel` |
| `reported.discEnabled` | bool | 否 | 磨盘是否开启 |
| `reported.discSpeed` | number | 否 | 磨盘转速，单位 rpm |
| `reported.discLiftState` | string | 否 | 磨盘升降状态，例如 `UP`、`DOWN` |
| `reported.lightEnabled` | bool | 否 | 照明是否开启 |
| `reported.batteryPercent` | number | 否 | 电量百分比，范围 0-100 |
| `reported.deviceErrorCode` | number | 否 | 设备错误码，0 表示无错误 |
| `reported.deviceErrorMessage` | string | 否 | 设备错误信息 |
| `reported.currentRegionId` | string | 否 | 当前执行区域 ID |
| `reported.currentRegionRepeatIndex` | number | 否 | 当前区域正在执行第几遍 |
| `reported.currentRegionRepeatTotal` | number | 否 | 当前区域总遍数 |
| `timestamp` | string | 是 | 数据上报时间戳，毫秒 |

## 6. 机器人属性编码规划

以下属性编码用于平台建模。最终编码以物联网平台创建的属性编码为准；机器人端 MQTT 客户端通过配置文件维护“内部字段 -> 平台属性编码”的映射。

### 6.1 任务状态类

| 平台属性编码建议 | 类型 | 单位 | 说明 | 调度程序数据来源 |
|------------------|------|------|------|------------------|
| `taskId` | string | - | 当前任务 ID | `task_config.task_id` |
| `taskState` | string | - | 当前任务状态 | `state` |
| `taskProgress` | number | % | 任务进度，0-100 | `_task_progress()` |
| `taskMessage` | string | - | 任务状态说明/错误信息 | `last_error` 或 `state` |
| `currentRegionId` | string | - | 当前执行区域 ID | `_current_region_repeat_progress()` |
| `currentRegionRepeatIndex` | number | 次 | 当前区域正在执行第几遍 | `_current_region_repeat_progress()` |
| `currentRegionRepeatTotal` | number | 次 | 当前区域总遍数 | `_current_region_repeat_progress()` |

任务状态枚举建议：

| 值 | 说明 |
|----|------|
| `IDLE` | 空闲 |
| `READY` | 已配置任务/路径可执行 |
| `PLANNING` | 路径规划中 |
| `RUNNING` | 执行中 |
| `PAUSED` | 暂停 |
| `COMPLETED` | 已完成 |
| `STOPPED` | 已停止 |
| `ERROR` | 异常 |

### 6.2 地图与定位类

| 平台属性编码建议 | 类型 | 单位 | 说明 | 调度程序数据来源 |
|------------------|------|------|------|------------------|
| `mapId` | string | - | 当前地图 ID | 当前任务/地图绑定 |
| `mapName` | string | - | 当前地图名称 | 当前任务/地图绑定 |
| `mapVersion` | number | - | 地图版本 | `map_service.get_map_info()` |
| `mapFrameId` | string | - | 坐标系，默认 `map_aligned` | 调度配置 |
| `mapOriginX` | number | m | 地图原点 X 坐标 | `map_service.get_map_info()` |
| `mapOriginY` | number | m | 地图原点 Y 坐标 | `map_service.get_map_info()` |
| `mapOriginHeading` | number | deg | 地图原点朝向 | `map_service.get_map_info()` |
| `mapResolution` | number | m/cell | 地图分辨率 | `map_service.get_map_info()` |
| `mapWidth` | number | cell 或 px | 地图宽度 | `map_service.get_map_info()` 或预览结果 |
| `mapHeight` | number | cell 或 px | 地图高度 | `map_service.get_map_info()` 或预览结果 |
| `pathPreviewScaleX` | number | - | 路径规划预览图 X 方向缩放比例 | 地图响应/渲染结果 |
| `pathPreviewScaleY` | number | - | 路径规划预览图 Y 方向缩放比例 | 地图响应/渲染结果 |
| `poseX` | number | m | 机器人 X 坐标 | `aurora_bridge.get_pose()` |
| `poseY` | number | m | 机器人 Y 坐标 | `aurora_bridge.get_pose()` |
| `poseHeading` | number | deg | 机器人朝向 | `aurora_bridge.get_pose()` |


### 6.3 速度类

| 平台属性编码建议 | 类型 | 单位 | 说明 | 数据来源 |
|------------------|------|------|------|----------|
| `linearSpeed` | number | m/s | 线速度 | `/cmd_vel` 或调度缓存 |
| `angularSpeed` | number | rad/s | 角速度 | `/cmd_vel` 或调度缓存 |
| `leftWheelSpeed` | number | rpm | 左轮速度 | `/chassis/status` 或下发缓存 |
| `rightWheelSpeed` | number | rpm | 右轮速度 | `/chassis/status` 或下发缓存 |

### 6.4 作业面积类

| 平台属性编码建议 | 类型 | 单位 | 说明 | 调度程序数据来源 |
|------------------|------|------|------|------------------|
| `totalWorkArea` | number | m2 | 总作业面积 | `_total_work_area_m2()` |
| `finishedWorkArea` | number | m2 | 已完成作业面积 | `totalWorkArea * progress` |
| `remainingWorkArea` | number | m2 | 剩余作业面积 | `totalWorkArea * (1 - progress)` |
| `remainingTime` | number | s | 预计剩余时间 | `_estimate_remaining_time_s(progress)` |

### 6.5 设备状态类

| 平台属性编码建议 | 类型 | 单位 | 说明 | 数据来源 |
|------------------|------|------|------|----------|
| `chassisEnabled` | bool | - | 底盘是否使能 | 底盘状态/调度缓存 |
| `taskEnable` | bool | - | 是否允许处理导航 `/cmd_vel` | `/chassis/task_enable` 状态缓存 |
| `discEnabled` | bool | - | 磨盘是否开启 | 底盘状态/调度缓存 |
| `discSpeed` | number | rpm | 磨盘转速 | 底盘状态/下发缓存 |
| `discLiftState` | string | - | 磨盘升降状态 | 底盘状态/下发缓存 |
| `lightEnabled` | bool | - | 照明是否开启 | 底盘状态/调度缓存 |
| `batteryPercent` | number | % | 电量百分比 | 底盘状态，若当前无字段需后续补充 |
| `deviceErrorCode` | number | - | 设备错误码 | 底盘状态/调度异常 |
| `deviceErrorMessage` | string | - | 设备错误信息 | 底盘状态/调度异常 |

### 6.6 图片/大字段类

图片按需响应同时支持 base64 和 URL 两种字段：

- `xxxImage`：通过 MQTT 直接返回 base64 图片内容。
- `xxxImageUrl`：机器人先通过 HTTP/对象存储上传图片文件，平台返回 URL，随后 MQTT 返回图片 URL。
- 实际响应时可同时返回两种字段，也可按平台能力只返回其中一种。

| 平台属性编码建议 | 类型 | 说明 |
|------------------|------|------|
| `pathPreviewImage` | string | 路径规划图 base64 |
| `taskResultImage` | string | 任务结果图 base64 |
| `pathPreviewImageUrl` | string | 路径规划图 URL |
| `taskResultImageUrl` | string | 任务结果图 URL |

## 7. 设备数据上报示例

### 7.1 综合状态上报示例

Topic：

```text
rg/cloud/deviceData/002/GRINDER_001/status
```

Payload：

```json
{
  "reported": {
    "taskId": "task_20260625_001",
    "taskState": "RUNNING",
    "taskProgress": 56.0,
    "taskMessage": "running",
    "mapId": "map001",
    "mapName": "车间A区-1楼",
    "poseX": -0.256,
    "poseY": 0.084,
    "poseHeading": 63.6,
    "linearSpeed": 0.2,
    "angularSpeed": 0.0,
    "leftWheelSpeed": 120,
    "rightWheelSpeed": 118,
    "totalWorkArea": 55.9,
    "finishedWorkArea": 31.3,
    "remainingWorkArea": 24.6,
    "remainingTime": 420,
    "chassisEnabled": true,
    "taskEnable": true,
    "discEnabled": true,
    "discSpeed": 1200,
    "discLiftState": "DOWN",
    "lightEnabled": false,
    "batteryPercent": 78,
    "deviceErrorCode": 0,
    "deviceErrorMessage": "",
    "currentRegionId": "work_region_1",
    "currentRegionRepeatIndex": 1,
    "currentRegionRepeatTotal": 2
  },
  "timestamp": "1782365617370"
}
```

### 7.2 任务开始上报示例

任务开始时通过任务开始 Topic 立即上报一次。

Topic：

```text
rg/cloud/deviceData/002/GRINDER_001/task/start
```

Payload：

```json
{
  "reported": {
    "taskId": "task_20260625_001",
    "taskState": "RUNNING",
    "taskProgress": 0.0,
    "taskMessage": "task started",
    "mapId": "map001",
    "mapName": "车间A区-1楼",
    "mapFrameId": "map_aligned"
  },
  "timestamp": "1782365617370"
}
```

字段说明：

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `reported.taskId` | string | 是 | 开始执行的任务 ID |
| `reported.taskState` | string | 是 | 任务状态，任务开始时为 `RUNNING` |
| `reported.taskProgress` | number | 是 | 任务初始进度百分比，一般为 0 |
| `reported.taskMessage` | string | 否 | 任务开始说明 |
| `reported.mapId` | string | 否 | 任务绑定地图 ID |
| `reported.mapName` | string | 否 | 任务绑定地图名称 |
| `reported.mapFrameId` | string | 否 | 坐标系，默认 `map_aligned` |
| `timestamp` | string | 是 | 任务开始上报时间戳，毫秒 |

### 7.3 任务结果上报示例

任务结束后通过任务结果 Topic 上传任务结果摘要。

Topic：

```text
rg/cloud/deviceData/002/GRINDER_001/task/result
```

Payload：

```json
{
  "reported": {
    "taskId": "task_20260625_001",
    "mapId": "map001",
    "mapName": "车间A区-1楼",
    "taskState": "COMPLETED",
    "taskProgress": 100.0,
    "taskResultAllCompleted": true,
    "taskResultStopReason": "",
    "taskResultFinishedAt": "1782365617000",
    "taskResultRegions": "[{\"region_id\":\"work_region_1\",\"target_repeat\":2,\"executed_repeat\":2,\"completed\":true}]",
    "taskResultImageUrl": "https://example.com/files/task_20260625_001_result.jpg"
  },
  "timestamp": "1782365617370"
}
```

字段说明：

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `reported.taskId` | string | 是 | 结束的任务 ID |
| `reported.mapId` | string | 否 | 任务绑定地图 ID |
| `reported.mapName` | string | 否 | 任务绑定地图名称 |
| `reported.taskState` | string | 是 | 任务最终状态，例如 `COMPLETED`、`STOPPED`、`ERROR` |
| `reported.taskProgress` | number | 是 | 最终进度百分比，完成时为 100 |
| `reported.taskResultAllCompleted` | bool | 是 | 是否所有区域和遍数都执行完成 |
| `reported.taskResultStopReason` | string | 否 | 停止或未完成原因，正常完成时为空字符串 |
| `reported.taskResultFinishedAt` | string | 是 | 任务结束时间戳，毫秒 |
| `reported.taskResultRegions` | string | 否 | 区域执行结果；平台不支持数组时使用 JSON 字符串 |
| `reported.taskResultImageUrl` | string | 否 | 任务结果图 URL；如果平台支持文件 URL，可和 base64 图片字段同时存在或择一使用 |
| `timestamp` | string | 是 | 本次结果上报时间戳，毫秒 |

说明：

- 如果平台属性不支持 object/array，`taskResultRegions` 可以使用 JSON 字符串。
- 如果平台支持 object/array，可直接按平台要求改成结构化字段。

### 7.4 平台请求地图示例

地图数据由平台按需请求。平台向请求 Topic 下发请求，机器人统一返回地图元信息和路径规划预览图。这里的图片不是原始地图底图，而是调度程序生成的路径规划预览图。

Topic：

```text
rg/cloud/deviceCommand/002/GRINDER_001/map/request
```

Payload：

```json
{
  "requestId": "req_20260625_001",
  "requestType": "MAP",
  "mapId": "map001",
  "imageType": "PATH_PREVIEW",
  "imageFormat": "jpg",
  "imageReturnType": "URL",
  "uploadUrl": "https://example.com/api/files/upload",
  "uploadMethod": "POST",
  "uploadHeaders": "{\"Authorization\":\"Bearer xxx\"}",
  "maxEdge": 640,
  "timestamp": "1782365617000"
}
```

字段说明：

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `requestId` | string | 是 | 请求 ID，机器人响应时原样带回，便于平台匹配请求和响应 |
| `requestType` | string | 是 | 请求类型，固定为 `MAP`，表示请求地图元信息和路径规划预览图 |
| `mapId` | string | 否 | 地图 ID；为空时机器人返回当前选中/当前运行地图 |
| `imageType` | string | 否 | 图片类型，默认 `PATH_PREVIEW`；`PATH_PREVIEW` 路径规划预览图，`TASK_RESULT` 任务结果图 |
| `imageFormat` | string | 否 | 图片格式，默认 `jpg`，可选 `jpg`/`png` |
| `imageReturnType` | string | 否 | 图片返回方式，`BASE64` 返回 base64，`URL` 返回图片 URL，`BOTH` 同时返回；默认 `BASE64` |
| `uploadUrl` | string | 条件必填 | 文件服务器上传地址；当 `imageReturnType=URL` 或 `BOTH` 时必填 |
| `uploadMethod` | string | 否 | 文件上传 HTTP 方法，默认 `POST` |
| `uploadHeaders` | string | 否 | 文件上传请求头，JSON 字符串；用于平台鉴权，例如 token |
| `maxEdge` | number | 否 | 图片最大边长，默认按机器人本地配置 |
| `timestamp` | string | 是 | 平台请求时间戳，毫秒 |

机器人响应规则：

- `requestType=MAP`：机器人返回 `rg/cloud/deviceData/002/{devCode}/map/info`。
- 响应中必须同时携带地图元信息和路径规划预览图数据。
- 如果 `mapId` 为空，返回当前选中地图；如果当前无保存地图，可返回 LIVE_MAP。
- 机器人根据请求中的 `imageReturnType` 决定图片字段：`BASE64` 只返回 `pathPreviewImage`，`URL` 只返回 `pathPreviewImageUrl`，`BOTH` 两个字段都返回。
- 当 `imageReturnType=URL` 或 `BOTH` 时，机器人先将图片上传到请求中的 `uploadUrl`，再把文件服务器返回或约定生成的访问地址填入 `pathPreviewImageUrl`。
- 如果请求失败，机器人通过 `rg/cloud/deviceData/002/{devCode}/event` 返回错误事件。

路径规划预览图携带规则：

- 同一个 `map/info` 响应同时定义 `reported.pathPreviewImage` 和 `reported.pathPreviewImageUrl` 两个字段。
- `reported.pathPreviewImage` 用于返回路径规划预览图 base64 图片内容。
- `reported.pathPreviewImageUrl` 用于返回平台可访问的路径规划预览图 URL。
- 机器人按平台请求字段 `imageReturnType` 返回对应字段。
- URL 模式下，`pathPreviewImageUrl` 来源于平台请求中的 `uploadUrl` 上传结果。
- `pathPreviewImage` 和 `pathPreviewImageUrl` 至少有一个不能为空；不能只返回地图元信息而不返回图片数据。

### 7.5 地图响应示例

Topic：

```text
rg/cloud/deviceData/002/GRINDER_001/map/info
```

Payload：

```json
{
  "reported": {
    "requestId": "req_20260625_001",
    "requestType": "MAP",
    "imageType": "PATH_PREVIEW",
    "imageReturnType": "BASE64",
    "mapId": "map001",
    "mapName": "车间A区-1楼",
    "mapVersion": 4,
    "mapFrameId": "map_aligned",
    "mapOriginX": -7.6,
    "mapOriginY": -2.3,
    "mapOriginHeading": 0.0,
    "mapResolution": 0.05,
    "mapWidth": 431,
    "mapHeight": 460,
    "pathPreviewScaleX": 0.888889,
    "pathPreviewScaleY": 0.890995,
    "pathPreviewImage": "<base64>",
    "pathPreviewImageUrl": "https://example.com/files/map001_path_preview.jpg",
    "pathPreviewImageFormat": "jpg"
  },
  "timestamp": "1782365617370"
}
```

字段说明：

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `reported.requestId` | string | 是 | 对应平台请求 ID |
| `reported.requestType` | string | 是 | 对应请求类型，地图响应为 `MAP` |
| `reported.imageType` | string | 是 | 图片类型，例如 `PATH_PREVIEW`、`TASK_RESULT` |
| `reported.imageReturnType` | string | 是 | 实际返回方式，取值 `BASE64`、`URL`、`BOTH` |
| `reported.mapId` | string | 是 | 地图 ID |
| `reported.mapName` | string | 否 | 地图名称 |
| `reported.mapVersion` | number | 否 | 地图版本 |
| `reported.mapFrameId` | string | 否 | 坐标系，默认 `map_aligned` |
| `reported.mapOriginX` | number | 否 | 地图原点 X 坐标，单位 m |
| `reported.mapOriginY` | number | 否 | 地图原点 Y 坐标，单位 m |
| `reported.mapOriginHeading` | number | 否 | 地图原点朝向，单位 deg |
| `reported.mapResolution` | number | 否 | 地图分辨率，单位 m/cell |
| `reported.mapWidth` | number | 否 | 地图宽度，单位 cell 或预览像素，需和平台约定 |
| `reported.mapHeight` | number | 否 | 地图高度，单位 cell 或预览像素，需和平台约定 |
| `reported.pathPreviewScaleX` | number | 否 | 路径规划预览图 X 方向缩放比例 |
| `reported.pathPreviewScaleY` | number | 否 | 路径规划预览图 Y 方向缩放比例 |
| `reported.pathPreviewImage` | string | 否 | 路径规划预览图 base64 内容；与 `pathPreviewImageUrl` 至少返回一个 |
| `reported.pathPreviewImageUrl` | string | 否 | 路径规划预览图访问 URL；与 `pathPreviewImage` 至少返回一个 |
| `reported.pathPreviewImageFormat` | string | 是 | 图片格式，例如 `jpg` 或 `png` |
| `timestamp` | string | 是 | 地图响应时间戳，毫秒 |

### 7.6 路径规划预览图携带方式

路径规划预览图不主动周期上传。平台需要预览图时，请求地图 Topic，机器人通过 `map/info` 响应返回当前最新路径规划预览图。

携带规则：

- `pathPreviewImage` 和 `pathPreviewImageUrl` 同时定义在 `map/info` 响应中。
- 平台通过请求字段 `imageReturnType` 指定返回方式。
- `imageReturnType=BASE64`：机器人返回 `pathPreviewImage`，可不返回 `pathPreviewImageUrl`。
- `imageReturnType=URL`：平台必须在请求中携带 `uploadUrl`，机器人上传图片后返回 `pathPreviewImageUrl`，可不返回 `pathPreviewImage`。
- `imageReturnType=BOTH`：平台必须在请求中携带 `uploadUrl`，机器人同时返回 `pathPreviewImage` 和 `pathPreviewImageUrl`。
- 如果只返回一种，则 `pathPreviewImage` 和 `pathPreviewImageUrl` 至少有一个不能为空。
- 如果返回 URL，需要平台提供文件上传服务或对象存储上传地址；不建议使用机器人本地 HTTP 文件服务器作为公网 URL 来源。
- 第一阶段如果平台暂未提供文件上传接口，可以先返回 base64；如果 MQTT payload 受限，则暂不启用图片字段，只返回地图元信息。

## 8. 上报频率建议

| 数据类型 | 建议频率/触发方式 |
|----------|-------------------|
| 在线状态 | MQTT 连接成功后立即上报 |
| 离线状态 | 程序正常退出时上报；异常断开通过 Last Will 上报 |
| 综合状态 | 1 Hz |
| 定位/速度 | 合并在综合状态中，默认 1 Hz；如平台需要更高频可提高到 2-5 Hz |
| 任务过程状态 | 合并在综合状态中，默认 1 Hz |
| 任务开始/结束事件 | 开始、完成、停止、异常时立即上报 |
| 地图信息 | 不主动周期上报；平台订阅/请求时返回当前最新地图信息 |
| 路径规划预览图 | 不主动周期上报；平台订阅/请求时返回当前最新预览图或预览图 URL |
| 告警 | 发生时立即上报 |


## 9. 配置项

后续可在 `scheduler.yaml` 中增加：

```yaml
mqtt_enabled: false
mqtt_host: 14.18.91.10
mqtt_port: 1883
mqtt_username: "MQTT"
mqtt_access_key_env: "GRINDER_MQTT_ACCESS_KEY"
mqtt_access_secret_env: "GRINDER_MQTT_ACCESS_SECRET"
# 也可以本地部署时直接配置，但不要提交真实密钥：
# mqtt_access_key: ""
# mqtt_access_secret: ""
mqtt_dev_code: "GRINDER_001"
mqtt_client_id: "GRINDER_001"
mqtt_keepalive: 60
mqtt_status_hz: 1.0
mqtt_qos: 0
mqtt_retain: false
mqtt_offline_will_enabled: true
```

## 10. 待平台确认项

与平台联调前需要确认：

1. 当前机器人设备编码 `{devCode}`。
2. MQTT 用户名、accessKey、accessSecret。
3. 平台中各测点的最终属性编码。
4. `reported` 中是否支持 bool/number 原生类型，还是必须全部转成 string。
5. 图片字段使用 base64、URL，还是两者同时返回。
6. 如果使用 base64，需要确认 MQTT payload 最大长度和平台属性字符串最大长度。
7. 如果使用 URL，需要确认平台 HTTP 文件上传接口、鉴权方式和返回 URL 格式。
8. 平台是否支持 JSON 字符串形式的数组/对象字段，例如 `taskResultRegions`。
9. QoS 和 retain 是否有平台侧限制。
