# SL-LinkA Integration Notes

## Current Status

`SL-LinkA` 已经扩展并同步到调度节点所需的任务、地图和视频流消息。

已完成同步：
- `proto/sl_link.proto`
- Python `sdk/python/sl_link/message_gen/sl_link_pb2.py`
- Python `sdk/python/sl_link/message_gen/nanopb_pb2.py`
- Android `sdk/android/src/message_gen/sl_link/SlLink.java`
- Android `sdk/android/src/frame/sl_link/SlMessageBuilder.kt`
- Android `sdk/android/examples/Example.kt`
- Embedded `sdk/embedded/message_gen/sl_link.pb.c`
- Embedded `sdk/embedded/message_gen/sl_link.pb.h`

## New Scheduler Messages

Component:

- `COMP_SCHEDULER = 0x08`

Messages:

- `TaskConfig`
- `TaskCommand`
- `TaskCommandResponse`
- `TaskStatusReport`
- `TaskPathRequest`
- `TaskPathChunk`
- `MapPreviewRequest`
- `MapPreviewResponse`
- `MapEditCommand`
- `MapEditResponse`
- `MapEditStatusReport`
- `VideoStreamInfoRequest`
- `VideoStreamInfoResponse`

## Android Notes

Android 侧现在分成两层：

- `src/message_gen/sl_link/SlLink.java`
  - protobuf 生成模型
- `src/frame/sl_link/SlMessageBuilder.kt`
  - 协议帧打包层

已经补充了这些 builder 方法：
- `buildTaskConfigRaw`
- `buildTaskCommandRaw`
- `buildTaskPathRequestRaw`
- `buildMapPreviewRequestRaw`
- `buildMapEditCommandRaw`
- `buildVideoStreamInfoRequestRaw`

`Example.kt` 也已经补了新消息的发送入口和基础响应处理占位。

## Embedded Notes

Embedded 侧已经有新的 `pb.c/.h`，但示例逻辑需要按业务补路由。

建议至少增加对以下消息的处理：
- `TaskConfig`
- `TaskCommand`
- `TaskPathRequest`
- `MapPreviewRequest`
- `MapEditCommand`
- `VideoStreamInfoRequest`

## Regeneration Reminder

如果后续继续改动 `sl_link.proto`，请务必重新生成：

1. Python `sl_link_pb2.py`
2. Android `SlLink.java`
3. Embedded `sl_link.pb.c/.h`

否则三端会出现字段和枚举不一致。
