# Grinder 与 Android APP 联合复审 · 2026-10-04

当前代码不能保证“切出去再切回来无缝衔接”。同一进程、连接保持、任务显示缓存完整时，设备页已有恢复机制；断网、在途请求取消、进程被回收和遥控锁定等场景仍有缺口。尤其是 APP 为恢复显示而补取路径，会进入机器人端具有执行副作用的规划接口。

审计基线：机器人 HEAD `2c02e72ff4bcd051e10e98869e54a4d37d8e60b3`；APP HEAD `d4a0258e75d43e9f68cb9539a8e2ca7ce4148da7`，仓库位于 `E:/CODE/grindingrobot/grindingrobot`。分析以当前工作区为准。机器人 F04/F05 修改已经暂存，尚未提交；因此直接克隆当前 HEAD 仍不能获得这些修复。此前独立克隆验证使用的是临时候选提交，不能等同于正式交付。

本轮覆盖自研调度/底盘关键链路、地图与视频资源路径、SL-LinkA 双端协议、Android 连接/生命周期/任务地图/遥控/视频/OTA。未对所有外部 ROS 算法和 Android 页面逐行审查。证据分为实际源码离线复现、生产 AST 方法/分支复现、静态确认及待实机验证；这些复现均未连接串口、机器人或启动 ROS 节点。

## 之前问题的当前状态

| 原编号 | 当前复核 | 修改建议与收益 |
| --- | --- | --- |
| F01 P1 停车缓存 | **仍存在，重新复现**。运动后停车新增帧 0，最后已发送轮速 100/100；磨盘开启后停车新增帧同样为 0。 | 停车强制写、局部写使整块缓存失效、重连安全重同步；确保真正下发零输出。 |
| F02 P1 急停被启动覆盖 | **仍存在，已新增阻塞屏障复现**。定位等待中急停，释放后最终 RUNNING，并重新使能磨盘和任务。 | 急停锁存与显式复位、运行代号、每个阻塞阶段及提交输出前校验；确保急停最终优先。 |
| F03 P1 不安全规划回退 | **仍存在，静态复核**。`planner_adapter.py:108` 仍调用 `:1006` 的简化回退。 | 规划不可用应失败并保持停止；调试回退不能直接进入执行。 |
| F04 P1 Super-LIO gitlink | **工作区修复，交付待提交**。索引已是普通源码文件；来源/许可证/本地修改已保留。 | 将现有修复纳入正式提交，再从该提交独立克隆验收。 |
| F05 P1 必需配置被忽略 | **工作区修复，交付待提交**。公开默认配置、可选本地覆盖、预检和仿真关闭外部服务已存在；本次 5 项配置测试通过。 | 提交修复；Linux/ROS 仿真启动仍须完成。此前完整构建受缺少 `livox_ros_driver2` 阻塞。 |
| F06 P1 控制权边界 | **仍存在**。TCP 设备 ID 过滤不构成身份认证，未见控制权租约。 | 认证、会话控制权与权限隔离，防止多个客户端竞争操控。 |
| F07 P2 斜坡超时停车 | **仍存在，重新复现**。留在 80/80；默认斜坡关闭，仅启用时触发。 | 超时/急停绕过斜坡并持续确认零输出。 |
| F08 P2 任务身份 | **仍存在**。`scheduler_node.py:8536` 不校验目标任务，未知命令落入 STOP。 | 验证任务/执行身份及枚举，拒绝过期或非法请求。 |
| F09 P2 地图持久化一致性 | **仍存在**。`map_service.py:922` 直接写 NPY，JSON 单独替换，加载前就修改区域。 | 版本目录/清单与原子指针，完整验证后一次替换内存状态。 |
| F10 P2 回环失败不重试 | **仍存在**。当前文件迁移到 `catkin_ws/src/mapping/cloud_to_occupancy_grid/src/cloud_to_occupancy_grid_node.cpp:403`，失败仍更新 completed revision。 | 有界退避重试同一版本，保留最后成功地图。 |
| F11 P2 视频阻塞调度 | **仍存在**。FFmpeg 写管道持状态锁；调度读取同锁。 | 独立写线程、有界最新帧、截止时间；视频故障不拖住任务周期。 |
| F12 P2 TCP 资源上限 | **仍存在**。连接总数无上限，删除队列无 pending 边界，sendall 无应用截止时间。 | 全局预算、限流和背压，稳定 RSS/线程与控制响应。 |
| F13 P2 测试可信度 | **仍存在，部分重现**。live map 替身缺服务、metrics 替身缺消息 ID；未见 Catkin Python 测试注册。 | 修复替身、隔离 mock、加入安全行为与双端恢复门禁。 |
| F14 P2 依赖闭合 | **部分修复，仍未闭合**。Super-LIO/basic 声明已补；底盘仍缺 geometry_msgs 声明；完整 Linux 构建未完成。 | 明确依赖安装和启动预检；按目标镜像验证。旧 protobuf 环境失败本次没有再次复现。 |
| F15 P2 MQTT 明文凭据 | **实现仍存在，默认暴露已降低**。公开配置默认禁用 MQTT；启用后仍 username/password + 无 TLS。 | 启用外部服务前增加 TLS、证书校验，保持本地秘密覆盖。 |
| F16 P2 串口日志 | **仍存在**。默认开启，逐帧打开/追加文件，无轮转容量。 | 异步有界日志、轮转/采样，降低串口锁内磁盘开销。 |

旧问题详细说明见 [此前报告](E:/CODE/C++/Grinder/artifacts/repository-audit-20260928/AUDIT.md)。F01/F02 应继续列为第一批整改，不能因为 APP UI 显示停止或另有零速发布就关闭问题。

## 新增问题：任务恢复与 APP 连续性

### R01 · P1 · 补取显示路径会改写运行任务状态【生产分支复现 + 静态调用链】

设备页 [loadPath](E:/CODE/grindingrobot/grindingrobot/feature/device/src/main/java/com/sinelynx/grindingrobot/feature/device/viewmodel/DeviceTaskMapController.kt:208) 用 `PathPointPlanRequest(force_replan=false)` 获取显示材料。机器人 [转换请求](E:/CODE/C++/Grinder/catkin_ws/src/grinder_scheduler/src/grinder_scheduler/scheduler_node.py:8657) 进入规划接口，[缓存命中](E:/CODE/C++/Grinder/catkin_ws/src/grinder_scheduler/src/grinder_scheduler/scheduler_node.py:11846) 仍写 `state=READY`；未命中则进入会修改 current_path、规划游标等共享状态的规划流程。

离线执行实际缓存分支得到 `state=READY、exec_active=true`。[路径执行周期](E:/CODE/C++/Grinder/catkin_ws/src/grinder_scheduler/src/grinder_scheduler/scheduler_node.py:6711) 随后提前返回；[磨盘静止保护](E:/CODE/C++/Grinder/catkin_ws/src/grinder_scheduler/src/grinder_scheduler/scheduler_node.py:4118) 也要求 RUNNING。不能将此解读为硬件一定停住：导航可能仍保留已发目标，形成调度状态与实际动作不一致。

另有同一接口修改 `task_config.task_id/global_direction`，离线地图请求暂时替换全局 `self.map_service` 后再恢复（`:11634`）。这使显示查询与实时任务/定时器共享可变上下文。

建议：提供严格只读的“当前执行快照/路径查询”，按 execution_id/path_version 返回现有数据，不规划、不改变任务状态、不临时切换全局地图服务。独立规划请求必须使用局部上下文，并显式限制运行中重规划。

收益：返回 APP、冷启动看任务、多个客户端看状态都不会干扰执行；省去不必要的重规划和导航地图刷新。验收：RUNNING/PAUSED/STOPPED 各状态查询后完全不变；冷 APP 恢复不新增执行目标、使能或磨盘命令，路径版本只在真实重规划时变化。

### R02 · P1 · 锁定遥控可在后台继续发令，并在重连后恢复旧方向【静态确认】

[摇杆](E:/CODE/grindingrobot/grindingrobot/feature/map/src/main/java/com/sinelynx/grindingrobot/feature/map/ui/VirtualDirectionPad.kt:232) 在边缘停留 1 秒可锁定；手势结束/取消时，锁定状态不会回中，清零主要依赖 `onDispose`。后台切换不等于页面退出 Composition。[遥控 ViewModel](E:/CODE/grindingrobot/grindingrobot/feature/main/src/main/java/com/sinelynx/grindingrobot/feature/main/viewmodel/RemoteSettingsViewModel.kt:313) 每 200ms 按 lastPosition 持续发送，未校验生命周期可见性或连接会话；只在回中、部分模式变化和 onCleared 取消。

因此，“先锁定前进→切到其他 APP”存在后台持续发令路径；断线循环也没有清除旧方向，恢复连接后存在继续发送路径。机器人 2 秒手动 watchdog 依赖**没有新命令**，不能保护持续收到旧方向的情况。这里是源码路径判断，未进行平板锁定摇杆实机测试。

建议：手动控制必须绑定前台可见性、有效控制租约和新鲜输入。ON_STOP/失焦/断线/急停清除锁定、取消连续和等待中的控制任务，并提交高优先级零速；重连后必须重新触摸，禁止恢复旧运动意图。机器人端以会话租约/输入时间最终保护。

收益：切后台和断网不会延续遥控运动。自动任务可依产品规则独立运行，恢复页面只恢复观察。验收：锁定摇杆后切出、锁屏、网络断开再恢复，命令记录中不得出现未经新输入的非零命令。

### R03 · P1 · 已排队的旧连接命令能写到新连接【实际 TcpService 方法复现】

[sendData](E:/CODE/grindingrobot/grindingrobot/core/tcp/src/main/java/com/sinelynx/grindingrobot/core/tcp/TcpService.kt:291) 仅检查当前 connected 后 launch，返回 true；协程执行时才读全局 outputStream，没有捕获连接代号或流。连接代号现有校验主要覆盖 connect，不覆盖待发送命令。每次发送独立 launch，也没有业务层单一有界写队列来保证提交顺序和停车优先级。

复现用单线程调度屏障延迟发送，再替换连接输出流：旧流 0 字节，新流收到旧请求的 3 字节标记。测试调用当前编译的 TcpService，没有网络连接；它证明会话隔离缺失，未模拟整个重连握手。BufferedOutputStream 的内部同步不能替代业务命令顺序和连接身份。

建议：每会话一个有界写通道，封装 session_generation、命令类型和 deadline；流与代号在创建发送任务时固定。重连丢弃旧运动/启动命令；STOP/急停取消过期非零命令并优先提交。sendData 的“已排队”与“已写入/设备确认”分开反馈。旧接收/错误回调同样按代号隔离，避免关闭新会话。

收益：旧动作不会穿越连接边界；避免突发 launch 堆积和发送顺序不确定。验收：在发送前插屏障、断开并换流，旧请求不得进入新流；同时覆盖旧错误回调、STOP 与非零速度竞争。

### R04 · P2 · 帧解析器没有在会话切换清零，且零长帧带入旧 payload【实际 Kotlin 源码复现】

[连接回调](E:/CODE/grindingrobot/grindingrobot/core/tcp/src/main/java/com/sinelynx/grindingrobot/core/tcp/TcpManager.kt:221) 仅 clearMapTransfer；[clearMapTransfer](E:/CODE/grindingrobot/grindingrobot/core/tcp/src/main/java/com/sinelynx/grindingrobot/core/tcp/SlLinkManager.kt:139) 未 reset parser、链路统计和路径分片。接收 4096 字节帧的前 20 字节后断线，再接收新连接合法帧，实际 parser 返回 0 帧：它仍在消费旧帧剩余 payload。后续数据也可能暂时被吞掉，直到旧帧消费/CRC 校验结束。

[SlFrameParser](E:/CODE/grindingrobot/grindingrobot/core/sllink/src/frame/sl_link/SlFrameParser.kt:112) 的零 payload 分支没有清空 payloadBuffer。实际复现非空 3 字节帧后再解析零长帧，得到 payload 长度 3；这是帧层缺陷，不等于当前常规状态报文都是零长。

建议：增加统一 resetSession，隔离后重置 parser/monitor/所有组包器，零长帧显式使用空数组；验证版本、地址和合法消息边界。验收覆盖各帧字段处断连、零长帧、CRC 错误、连续/碎片帧，并保证新会话第一帧可解析。

收益：重连后状态和图像不会因残留半包延迟；减少误解析与误报丢包。

### R05 · P2 · 后台取消未释放地图请求，失败项回前台不会自动重试【实际方法与控制器复现】

[页面取消等待](E:/CODE/grindingrobot/grindingrobot/feature/device/src/main/java/com/sinelynx/grindingrobot/feature/device/viewmodel/DeviceTaskMapController.kt:125) 只取消 Flow job；sendMap 回调只得到 Boolean，没有拿到 TcpManager 自动生成的 request_id，所以不能同步 cancelMapSnapshotRequest。[通信层门闩](E:/CODE/grindingrobot/grindingrobot/core/tcp/src/main/java/com/sinelynx/grindingrobot/core/tcp/SlLinkManager.kt:120) 在响应/主动取消前保持占用，20 秒内不允许新地图请求。

实测实际门闩：登记请求 111 后，立即登记 222 返回 false；显式取消 111 才可登记 222。快速切回时如果旧图尚未完整收到，新控制器会把这当作“发送失败”。地图/区域 attempted 和路径 attemptedVersion 在失败后保留；仅反复回前台或收到相同任务/版本上报不会自动重试。实际控制器复现三次失败请求后，隐藏/显示仍是三次请求。

建议：requestMapSnapshot 返回可取消句柄/明确 request_id；UI 取消同步释放归属自己的在途请求。区分 BUSY、OFFLINE、NOT_READY 和错误，按 NOT_READY 的 retry_after_ms 有界退避；连接恢复/前台恢复触发本会话失败项重试，保留成功图层，避免每秒状态上报形成重试风暴。

收益：快速切回不需要手工点重试；避免重复大图传输和长期空白。验收：请求后 0.1/1/10 秒切出切回、失败后断线重连，相同 task/path_version 能补齐缺失材料，且旧响应不能结束新请求。

### R06 · P2 · CONNECTED 不代表业务已同步，进程回收后配置无法完整恢复【静态确认】

[onConnected](E:/CODE/grindingrobot/grindingrobot/core/tcp/src/main/java/com/sinelynx/grindingrobot/core/tcp/TcpManager.kt:221) 没有统一的设置、任务身份、地图版本和当前执行配置同步流程。[HomeViewModel](E:/CODE/grindingrobot/grindingrobot/feature/main/src/main/java/com/sinelynx/grindingrobot/feature/main/viewmodel/HomeViewModel.kt:56) 只在初始化连接成功后读取设置；自动重连不重新执行这段流程。[MainActivity.onResume](E:/CODE/grindingrobot/grindingrobot/app/src/main/java/com/sinelynx/grindingrobot/MainActivity.kt:34) 仅调用 OTA 前台逻辑。网络回调更新 UI 网络类型，不主动校验会话新鲜度。

TaskMapDisplaySnapshot 只在内存；进程重启后补取公共区域和路径无法还原任务临时禁区。当前控制器已明确展示“任务显示信息不完整”，这是一项已有保护，不能当作恢复完整。TaskStatusReport 缺当前执行 map_id/map_revision、execution_id 和完整任务配置查询；它只含 task_id/map_version/path_version 等状态。轨迹只保留本地最近 500 点，设备页未补取后台漏掉的实时轨迹。

建议：连接状态拆为 CONNECTING→CONNECTED→SYNCING→READY；会话恢复只查询设备权威快照，不自动重发 START/RESUME。快照包括设备实例/执行 ID、地图身份、任务配置、路径版本、输出反馈和状态时间；恢复必要设置后再放开控制。轻量界面路由/草稿可持久化，运行权威状态必须重新查询。轨迹按 execution_id/cursor 补取或明确标出缺口。

收益：用户看到的任务、地图和操作权限来自同一次恢复过程；进程回收也能观察正在执行的任务。验收：冷启动/同 task_id 新执行/设备重启，不能将旧地图缓存或旧确认当作当前状态。

Android 官方说明 ViewModel 内存不跨进程死亡保存；后台也可能限制执行和网络，不能依赖 singleton/TCP 永久存活。参考 [保存 UI 状态](https://developer.android.com/topic/libraries/architecture/saving-states)、[进程生命周期](https://developer.android.com/guide/components/activities/process-lifecycle)、[Doze](https://developer.android.com/training/monitoring-device-state/doze-standby)。专用系统平板的电源白名单可能改变触发概率，需要按实际设备验证。

### R07 · P2 · 控制响应没有关联到具体请求，迟到确认可能被误认【静态确认】

TaskCommand/Response 只有 task_id/command 或 result/message/task_id。帧头已有 ack_seq，服务器回显，但 APP [handleTaskCommandResponse](E:/CODE/grindingrobot/grindingrobot/core/tcp/src/main/java/com/sinelynx/grindingrobot/core/tcp/SlLinkManager.kt:1595) 发布业务 payload 时不保留 ackSeq。[切换手动的等待](E:/CODE/grindingrobot/grindingrobot/feature/main/src/main/java/com/sinelynx/grindingrobot/feature/main/viewmodel/RemoteSettingsViewModel.kt:380) 使用 first()，连 task_id 都未匹配；其他页面部分按 task_id 匹配，也无法区分同任务的 START/STOP/PAUSE。

建议：APP 维护按 session_generation+seq+预期消息类型注册的 pending 请求；结果必须匹配 ACK 与任务/执行身份。需要跨重连查询/重试的有副作用操作再添加 operation_id 和设备端幂等结果缓存；不能盲目重放启停。控制执行成功和反馈到达目标状态分开表示。

收益：迟到 START 成功不会成为 STOP 成功；操作超时可以查明是否已执行，避免重复动作。验收：乱序/重复/迟到 ACK、同任务多命令、断线后旧确认均不得完成错误等待。

## 新增问题：资源、效率与协议

### R08 · P2 · 路径分片和部分路径缓存没有容量/生命周期边界【实际 Kotlin 源码复现】

[TaskPathChunkAssembler](E:/CODE/grindingrobot/grindingrobot/core/tcp/src/main/java/com/sinelynx/grindingrobot/core/tcp/TaskPathChunkAssembler.kt:10) 按 request_id 保存未完成片，没有条目数/总字节/总片数上限或 TTL，也无会话清理接口。1000 个不同请求各留一片 4096 字节，实际留下 1000 条目和 4,096,000 字节数据，尚未计容器开销；迟到半包、请求取消和异常响应都可能累积。路径解析在接收回调中同步执行。

[TaskPathStream](E:/CODE/grindingrobot/grindingrobot/core/model/src/main/java/com/sinelynx/grindingrobot/core/model/state/TaskPathStream.kt:58) 按 task_id 保存路径，清理分散在页面行为中；不能证明有全局 LRU/会话上限。完整任务交接缓存已有 active+pending 两份上限，应保留其设计，不重复建立多套缓存。

建议：组包器与查询句柄绑定，设置全局字节、单请求字节、片数、条目和 TTL 上限；完成/取消/换会话均释放。路径缓存按设备/执行身份限定当前项或 LRU，并限制点数。历史数据分页从数据库/设备读取。

收益：后台反复取消、大路径和长期运行不会线性增加 RSS。验收：循环 1000 次缺片/取消/重连，保留条目和内存回到预算范围；满额返回明确错误。

### R09 · P2 · 每包未使用的 Hex 格式化阻塞接收，地图在主线程重复全量解码【静态 + 桌面微基准】

[接收函数](E:/CODE/grindingrobot/grindingrobot/core/tcp/src/main/java/com/sinelynx/grindingrobot/core/tcp/TcpManager.kt:229) 每包执行 joinToString + "%02x".format，日志已经注释，字符串却仍生成。接收循环要等该回调返回才能继续 read。同样的 100×4096 字节桌面探针，当前 parser 约 23ms，未使用 Hex 约 546ms；这只是 JDK/当前桌面的合成测量，不能作为平板实测或承诺提升倍数。

设备控制器在 ViewModel 主协程中 decodeByteArray 仅为取尺寸，再 recycle；[绘制端](E:/CODE/grindingrobot/grindingrobot/feature/device/src/main/java/com/sinelynx/grindingrobot/feature/device/ui/DeviceStatusScreen.kt:244) 又完整解码。协议压缩字节上限 32MiB 不能约束解压后的 Bitmap。重复编码/复制大图、数据类对 imageBytes 做内容比较也有随图大小增长的成本。

建议：立即去掉无用 Hex，必要日志采样并限制字节；尺寸使用 inJustDecodeBounds，解码放后台线程并按显示尺寸采样，限制最大像素和解压预算。保留当前图与版本校验，通过共享不可变图像句柄避免重复 Bitmap/字节副本。CRC 查表/减少复制作为第二阶段优化，先实测剖析。

收益：降低接收背压、GC 与 UI 卡顿，使状态/ACK 更快被处理。验收记录大图传输时读取吞吐、ACK P95/P99、主线程最长帧、Bitmap 峰值，比较同一设备修改前后。

### R10 · P2 · 后台仍做地图轮询/视频播放，返回后播放器缺恢复协调【静态确认】

遥控 ViewModel 初始化就启动 [30 秒地图轮询](E:/CODE/grindingrobot/grindingrobot/feature/main/src/main/java/com/sinelynx/grindingrobot/feature/main/viewmodel/RemoteSettingsViewModel.kt:170)，只检查 TCP，未检查页面可见性。保留该 ViewModel 时切后台或切标签可能继续取大图。[RTSP 播放器](E:/CODE/grindingrobot/grindingrobot/feature/common/src/main/java/com/sinelynx/grindingrobot/feature/common/component/CameraRtspOverlay.kt:198) 以 remember 创建，prepare/play 只随 streamUrl 改变，release 在 onDispose；没有前后台暂停/恢复或会话状态驱动的重新准备。

建议：地图按页面需求与地图版本取，停止隐藏页轮询；视频在 ON_STOP 暂停/释放显示相关资源，ON_START 先校验会话和流地址，再按需准备。播放器状态不要代表机器人在线状态。与 F11 一起将媒体故障隔离到独立资源预算。

收益：减少后台耗电、网络带宽与解码资源；切回不会持续黑屏或等待旧视频会话。验收：后台 5 分钟记录网络/CPU/视频资源；同 URL 断网后回前台可恢复播放。

### R11 · P2 · 大数据和状态/ACK 共用无截止时间发送锁【扩展 F12，静态确认】

[bulk 发送](E:/CODE/C++/Grinder/catkin_ws/src/grinder_scheduler/src/grinder_scheduler/sl_linka_adapter.py:623) 已把路径批量合成约 128KiB，这是减少小写入的有效优化；但 sendall 仍持共享 send_lock，慢客户端可阻塞同连接的状态与 ACK。bulk 满额后又回退同步发送，可能占住有序请求工作线程。APP 后台被冻结或接收端被 Hex/解析拖慢时，这条背压路径更值得关注。

建议：每连接一个有界发送器，状态“只保留最新”、控制确认优先、bulk 按时间/字节切片让出；单次写有截止时间，超时关闭会话并释放资源。不能将 sendall 返回当作对端处理成功；业务确认单独记录。连接总数、删除 pending 和全局规划并发需一起设预算。

收益：大图/大路径不会长期拖住状态确认和线程，慢客户端资源有界。验收：只建连接不读取、低速读取、后台冻结，监测控制应用延迟与 ACK 交付延迟两条指标，RSS/线程不持续增长。

### R12 · P2 · 全局序号被当作连接丢包指标，重连统计失真【实际 Kotlin 源码复现】

服务器 [_next_seq](E:/CODE/C++/Grinder/catkin_ws/src/grinder_scheduler/src/grinder_scheduler/sl_linka_adapter.py:754) 在所有连接共用序号；APP [SlLinkMonitor](E:/CODE/grindingrobot/grindingrobot/core/sllink/src/frame/sl_link/SlLinkMonitor.kt:27) 按来源设备相邻 seq 推断丢包。其他客户端收走的序号也会成为本连接的“丢包”。换会话不 reset，实际 100→0 计为丢 65435 包，同 seq 重复则计 -1，污染累积值。

建议：明确 seq 是会话序号还是设备全局序号；用于连接诊断时每会话独立计数并 reset，区分重复/乱序/重启。TCP 链路质量主要依据 RTT、业务超时、读写背压、断连次数和状态年龄，不能把该序号差直接解释成无线丢包。

收益：故障诊断和优化方向可信，避免用虚假“丢包率”错误调大缓冲/超时。

### R13 · P2 · OTA 安装身份写死，设备灰度与事件归属不独立【静态确认】

[installId()](E:/CODE/grindingrobot/grindingrobot/feature/main/src/main/java/com/sinelynx/grindingrobot/feature/main/ota/ApkOtaManager.kt:79) 仍返回统一测试 ID。README 已明确这是联调值，当前源码仍未改为独立持久化安装身份。

建议：首次生成并持久化，检查/下载刷新/事件使用同一值，明确测试与正式渠道。收益：独立设备灰度、升级失败追踪和补报归属正确。验收：两台设备身份不同，同一安装重启保持一致；不在日志输出密钥/凭据。

## 协议已有优点与需要补齐的契约

两端当前 `sl_link.proto` 规范化内容完全一致，本次没有发现源协议字段漂移；这不等于所有生成 SDK、历史 APK 和固件兼容已经验证。地图协议已有 request_id、snapshot_id、总大小、CRC32 和 NOT_READY/retry_after_ms；APP 有地图片数/字节边界，路径已有 request_id。设备页已使用可见性、generation、缓存交接与路径版本，现有 14 项控制器测试通过。这些应保留并补齐会话/取消的整体契约。

建议协议形成以下最小闭环：

1. **会话握手**：协议能力、设备实例 ID、会话代号、控制权租约；旧客户端兼容行为明确。
2. **只读恢复快照**：执行身份、地图 ID/资产版本、任务配置、状态时间、路径版本、真实输出反馈；返回前按版本避免多次查询拼出混合快照。
3. **请求关联**：同会话 ACK 关联；跨重连副作用操作用 operation_id 去重/查询结果；过期手动输入丢弃。
4. **取消与背压**：APP 请求句柄释放组包；设备取消无用大响应/限制队列；明确 BUSY/OFFLINE/NOT_READY，重试有总截止时间。
5. **数据量优化**：确认现有 compact JSON 路径优化后再评估结构化 protobuf/分段分页、按 path_version 增量查询。逐点重复 JSON 字段与双端全量拼接属于优化候选，需用真实路径测压缩率/解析时间，不凭格式推断固定收益。

## 资源与速度预算：先测量，再调整

| 当前设计/常量 | 实际意义 | 建议优化与收益 |
| --- | --- | --- |
| APP 自动重连等待 5 秒；读/连接超时各 30 秒 | 网络黑洞可能先等待读超时再退避/连接；5 秒不是全部恢复时间。初连 connectAsync 有心跳，自动重连 connect 的心跳调用被注释，两条路径不一致。当前心跳默认单字节 0x00，无业务应答证明。 | 前台/网络恢复触发即时会话探测，统一连接路径；用有效业务状态年龄/请求响应判断新鲜度。有上限退避，避免离线无限固定频率重试。 |
| 设备状态/任务报告每连接 1Hz；scheduler tick 5Hz | 常规状态有约 1 秒周期粒度；部分调度工作为 200ms 粒度，不能把 UI 状态刷新速度等同于停车发送速度。 | 命令后立即权威状态响应、普通状态保持适量频率；批量显示数据不能阻塞控制/状态。 |
| APP 手动命令 5Hz；调度 watchdog 2s、检查 50ms；底盘超时 5s | timeout 只在不再收到命令时有效，锁定后台重复输入会续期；不能据此保证失焦立即停车。 | 生命周期 + 控制租约 + 底盘保护共同覆盖，并按实测通信抖动确定时限。 |
| 19200 baud，轮速读 10Hz、状态读 2Hz且已有合并读取 | 按 8N1 粗算，2 寄存器读请求/响应共 17 字节，线路最低约 8.9ms；不含帧间隔/控制器等待。正常轮询并未证明线路饱和。 | 保留合并轮询，先测 IO 锁等待/P99/重试；不要靠提高轮询频率处理停机缓存或并发问题。 |
| 地图组包上限 32MiB，临时 chunks+输出缓冲+结果字节并存 | 32MiB 是压缩输入边界，不是整个 APP RSS 或 Bitmap 边界。 | 统一字节/像素预算与少拷贝，记录大图峰值和 GC；后台只保留必要缓存。 |
| super_lio_loop 最多 2000 keyframes、每帧最多 8000 点 | 容量上界对应最多 1600 万点，另有图优化/描述子开销；到容量后停止增加关键帧。 | 测点云/图 RSS 与长作业效果；按需选分层子图、降采样、持久化/淘汰，而非简单增加上限。 |
| FFmpeg 本地 15fps、远端 10fps，默认串口原始日志开 | 视频/图像与磁盘负担可挤占安全周期，尤其故障时持锁 IO。 | 最新帧队列/异步轮转日志；以控制 P99 无退化为约束，再调分辨率/fps。 |

建议记录：前台恢复→READY/首个可信状态/地图补齐的 P50/P95/P99；用户停止→驱动实际下行/轮速反馈归零；APP 主线程帧耗时与 RSS/Bitmap峰值；板端 RSS/线程/CPU/磁盘增长；大图/路径吞吐及 ACK 延迟。没有 RK3588/平板/底盘实测，本报告不声称固定百分比的性能提升。

## “切出去再切回来”验收矩阵

| 场景 | 当前判断 | 必须验收的目标 |
| --- | --- | --- |
| 短暂后台，连接存活，显示缓存完整 | **有恢复基础**。设备页生命周期+缓存测试通过；未做实机演练。 | 首个新鲜状态自动接续，查询不改变执行；缓存完整时不重传大图。 |
| 正在拉图/路径时切出再回来 | **不能保证**。地图门闩残留、失败去重、路径残片。 | 取消释放，缺失项自动补齐，新旧响应隔离、内存有界。 |
| 后台断 Wi-Fi，随后恢复 | **只有传输重连，业务恢复不完整**。可能等待超时+退避，未同步 READY。 | 状态校验/设置/执行快照恢复，控制等待 READY，禁止旧命令跨会话。 |
| 长时间后台/系统冻结进程 | **不能保证无缝**，没有业务级前台校验；后台资源仍可能运行。 | 回前台探测新鲜度、必要时重连，只读同步；先显示缓存并标明同步/离线。 |
| 系统回收 APP 后重新打开 | **只能部分恢复**。内存快照/临时禁区和轨迹丢失，恢复路径还有 R01 副作用。 | 从设备执行快照恢复材料，轨迹补缺；不重发 START/RESUME。 |
| 后台期间任务完成/暂停/换图，甚至设备重启 | **缺执行实例/资产身份闭环**。 | 以 execution_id/map_revision 验证缓存，不把旧确认/旧地图用于新执行。 |
| 锁定遥控时切后台/断线重连 | **高优先级风险，见 R02/R03**。 | 后台/失焦/租约失效清零；重连必须新触摸，无自动恢复非零动作。 |
| 视频打开后后台再回来 | **未形成协调恢复**。 | 释放后台显示资源，同 URL 会话变化也可重建播放，视频故障不影响控制。 |

建议在模拟设备服务器和独立 ROS 仿真先执行，覆盖后台 1s/30s/5min、网络关闭/恢复、进程回收、设备实例变化、乱序 ACK、半帧断线和慢读客户端。涉及真实遥控/轮速/磨盘的平板测试需在明确授权及受控硬件场地开展。

## 验证结果与材料

- **实际离线复现**：F01/F07 假传输；F02 完整生产启动/急停方法 AST 屏障；R01 实际缓存分支 + 执行周期早返回；R03 当前 TcpService 的排队发送跨流；R04 当前 Kotlin parser；R05 当前 controller 和 SlLinkManager 门闩；R08 当前 assembler；R12 当前 monitor。
- **APP JVM**：70 项已有测试（TCP 56 + 设备地图控制器 14）及 3 项审计探针，共 **73 项通过**。受 Windows内存/页面文件不足影响，标准 Gradle 测试未跑通；改用当前已编译的 TCP/model 类、重新编译控制器/测试，按声明的 protobuf 3.24.0、Kotlin 2.2.21、coroutines 1.10.1 在小堆 JVM 执行。不能将此报告为完整 APK/Gradle 测试通过。
- **机器人离线测试**：寄存器 6、配置 5、方向 4、平台 1、Super-LIO 管理 11 共 **27 项通过**。metrics 4 项中 1 错误；live map import 缺 `SetSuperLioInitialPose` 替身；重定位测试缺 rospy。底盘 transport Windows 缺 termios，本次未跑通；此前 Linux 8 项通过不算本次通过。
- **静态格式**：当前范围 47 个 Python 文件通过 Python 3.8 AST 检查，16 个 XML/launch 解析通过；双端 proto 内容一致。
- **限制**：没有 Android 前后台/锁屏实机、RK3588 资源、ROS 全链路、真实 serial 停机反馈或完整 APK 验证。WSL 本次启动 E_ABORT；Gradle 原生内存/测试 JVM 内存提交失败是环境问题，不能归为机器人运行时泄漏。

可复现材料：[verification.json](E:/CODE/C++/Grinder/artifacts/repository-app-audit-20261004/verification.json)、[调度屏障探针](E:/CODE/C++/Grinder/artifacts/repository-app-audit-20261004/reproduce_scheduler_races.py)、[Kotlin 帧/分片/统计探针](E:/CODE/C++/Grinder/artifacts/repository-app-audit-20261004/AuditProbe.kt)、[地图恢复探针](E:/CODE/C++/Grinder/artifacts/repository-app-audit-20261004/AuditControllerProbe.kt)、[发送会话探针](E:/CODE/C++/Grinder/artifacts/repository-app-audit-20261004/AuditTransmissionProbe.kt)。运行脚本均在同目录；JVM 环境诊断保存在独立 planning 目录，APP 工作树保持干净。

## 建议整改顺序

1. **安全闭环**：F01/F02/F03 + R01/R02/R03，一起做停车、急停取消、只读恢复和遥控会话隔离；先通过屏障/下行帧测试，再仿真。
2. **无缝恢复**：R04/R05/R06/R07，请求句柄/ACK/恢复快照/READY gate；用上表验证短后台、断网和进程回收。
3. **资源与速度**：先去无用 Hex，随后 R08/R09/R10/R11 + F11/F12/F16；测同设备基线，保障控制 P99 不退化。无需等待大规模 C++ 重写。
4. **可交付与运营**：正式提交 F04/F05，闭合 F13/F14，处理 F06/F15/R12/R13；最后做结构化路径和子图等较大优化。

本轮仅新增审计报告和离线探针，未修改业务源码、协议或 APP 配置，未提交此前修复，保留用户既有改动。
