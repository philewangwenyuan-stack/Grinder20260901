# Grinder 仓库审计（2026-09-28）

基线：根仓库 HEAD `2c02e72` 和当前工作区。审计前 Super-LIO 已存在未提交改动，本次未修改业务源码、配置或已有改动。

结论：首先修复停车输出可靠性和任务并发控制，其次补齐干净克隆、测试与部署闭环。现有单测通过不能证明整条停车链路正确：本次在假传输对象中复现了安全停机不发送帧，以及启用斜坡后超时不能完全归零。

## 范围与证据等级

采用全仓结构盘点、按业务链路深入审查的方法，覆盖自研底盘、调度、地图编辑/持久化、规划适配、SL-LinkA 服务与 Python 帧解析、平台/MQTT、视频、Super-LIO 生命周期、工业回环/二维重投影、构建启动、仿真配置与测试组织。

通用 navigation、nanopb、SLAM/雷达 SDK 等外部代码只检查项目集成边界和相关热点，未逐行审查全部上游代码。Super-LIO gitlink 内的实现、Android/Embedded SDK、MST27 C++ 算法正确性未做完整专项验证。未执行真实硬件、完整 Catkin 编译或 Gazebo 场景测试。本报告不构成全仓无缺陷证明。

- **复现**：用当前源码方法和假对象执行，可重复得到结果。
- **静态确认**：源码或 Git 元数据明确存在该行为；实际损害取决于运行条件。
- **改进建议**：工程结构问题，不等同于已发生生产故障。

P1 为优先处理的安全/交付问题；P2 为可靠性、运维与验证问题；P3 为结构性改善。工作量仅按 S（局部）、M（跨模块）、L（需分阶段）估计，不是排期承诺。

## 优先问题

### F01 · P1 · 安全停机可能被过时缓存跳过【复现，M】

证据：[整块输出缓存判断](E:/CODE/C++/Grinder/catkin_ws/src/grinder_chassis_driver/src/grinder_chassis_driver/chassis_driver_node.py:665)、[安全停机](E:/CODE/C++/Grinder/catkin_ws/src/grinder_chassis_driver/src/grinder_chassis_driver/chassis_driver_node.py:692)、[轮速写入](E:/CODE/C++/Grinder/catkin_ws/src/grinder_chassis_driver/src/grinder_chassis_driver/chassis_driver_node.py:601)。

`_last_written_block` 只在整块写入后更新；轮速写入、磨盘等单寄存器写入均不使它失效。启动写入零输出后，发送非零轮速，再调用 `_issue_safe_stop()`，停机目标恰好等于旧缓存，于是 `_write_all_outputs()` 直接返回。关闭磨盘也有相同情形。

复现输出：停车新增帧 **0**，最后发送轮速仍为 **100/100 RPM**；单独开启磨盘后停车新增帧同样为 **0**。这是驱动方法级缺陷；调度层的其他零速发布可能缓解某些流程，但不能作为驱动停车正确性的保证。

建议：安全停机强制写入；统一寄存器已确认状态，任何局部写入同步更新或使整块缓存失效；重连后重新同步安全输出。加上“启动→运动→停车”“开启磨盘→停车”“重复停车”“通信失败→重连”的控制层测试。

收益：让安全停机确实产生下行指令，防止软件显示停止而下位机仍保留旧输出。验收应核验帧内容和最终轮速/磨盘反馈，而不只检查内存命令值。

### F02 · P1 · 急停可以被正在执行的启动流程覆盖【静态确认，M】

证据：[接收线程立即处理控制指令、工作线程处理有序请求](E:/CODE/C++/Grinder/catkin_ws/src/grinder_scheduler/src/grinder_scheduler/sl_linka_adapter.py:403)、[任务启动](E:/CODE/C++/Grinder/catkin_ws/src/grinder_scheduler/src/grinder_scheduler/scheduler_node.py:5956)、[急停](E:/CODE/C++/Grinder/catkin_ws/src/grinder_scheduler/src/grinder_scheduler/scheduler_node.py:8206)。

启动流程会等待定位服务、TF 或规划。此时急停在另一个线程中将 `_exec_active=false` 并关闭输出；旧启动流程返回后仍继续开启磨盘、设置 `_exec_active=true`、发布 `task_enable=true`。两条路径没有共同的取消代号或急停锁存校验。手动工作线程也可能在急停之后发布已经算好的非零命令。

建议：急停锁存、显式复位；对启动/恢复分配运行代号，每个阻塞阶段返回后及输出提交前检查代号和急停状态；失效的旧请求必须退出。底盘层也校验急停状态并集中序列化输出。不要用“整个启动流程持有一把大锁”修复，否则会让急停等待定位/规划。

收益：保证急停具有最终优先权，避免并发命令恢复运动。验收：在定位服务、规划、底盘写入处设置同步屏障，插入急停，确认屏障释放后没有非零输出和重新使能。

### F03 · P1 · 规划器不可用时自动返回未避障的可执行路径【静态确认，S/M】

证据：[导入失败降级](E:/CODE/C++/Grinder/catkin_ws/src/grinder_scheduler/src/grinder_scheduler/planner_adapter.py:23)、[降级入口](E:/CODE/C++/Grinder/catkin_ws/src/grinder_scheduler/src/grinder_scheduler/planner_adapter.py:103)、[包围盒往返路径](E:/CODE/C++/Grinder/catkin_ws/src/grinder_scheduler/src/grinder_scheduler/planner_adapter.py:1006)。

依赖缺失或模块导入失败后，`_fallback_plan()` 按区域包围盒生成往返端点，没有检查障碍占用、机器人足迹、凹多边形边界及线段连通性。即使先从自由空间提取轮廓，随后取包围盒仍会丢失这些约束。结果按普通 `PlannerPath` 返回；下游局部避障不能证明该覆盖路径有效。

建议：生产模式规划器失败即拒绝规划/启动，返回明确错误；如保留演示降级，应显式标识且禁止真机执行。任何可执行路径都需验证足迹碰撞和工作区约束。

收益：将依赖故障变为可诊断的停止，减少穿越禁区、贴障、反复卡住和错误覆盖估算。验收至少包含凹多边形、中间障碍、不可达区域和缺失规划模块。

### F04 · P1 · Super-LIO 源码无法从根仓库完整恢复【Git 元数据确认，M】

`git ls-files --stage` 显示 `catkin_ws/src/localization/Super-LIO` 为模式 **160000**，提交为 `239984eb8cff0ba5efdf999e9b90d02b2c7d5c5e`；根仓库没有 `.gitmodules`。这与 [AGENTS.md](E:/CODE/C++/Grinder/AGENTS.md:1) 所述“子仓库源码已经吸收到根仓库”不一致。

建议：按当前仓库约定真正将所需源码纳入根仓库；操作前保存现有未提交改动、上游来源、许可证和版本。如果决定继续用子模块，应补齐 `.gitmodules` 及受控拉取流程，并同步文档。

收益：使新机器、CI、交付包可恢复相同源码。验收：在全新目录仅从根仓库克隆，确认 Super-LIO 源码和构建所需文件完整。当前本地目录存在不能替代该验证。

### F05 · P1 · 启动必需配置被忽略，缺少可追踪默认配置【静态确认，S】

证据：[忽略规则](E:/CODE/C++/Grinder/.gitignore:22)、[真实启动强制加载](E:/CODE/C++/Grinder/catkin_ws/src/grinder_scheduler/launch/scheduler.launch:43)、[仿真同样加载](E:/CODE/C++/Grinder/catkin_ws/src/grinder_gazebo/launch/scheduler_sim.launch:9)。

`scheduler.yaml` 在本机存在，但不受 Git 跟踪。未找到对应受控模板或启动时生成流程。干净克隆后，上述 launch 的必需文件缺失；大量非敏感运行参数也不能通过版本审查回溯。

建议：提交不含凭据的默认配置，将设备凭据和部署差异移到可选本地覆盖文件；通过 launch 参数选择覆盖路径，并在启动前校验。保持 YAML、代码回退值、launch 参数一致。

收益：干净克隆可启动仿真，配置变更可审查，凭据仍不入库。验收：无本地秘密文件时能运行仿真，并明确禁用需要凭据的外部服务。

### F06 · P1 · 控制端口缺少身份和控制权边界【静态确认，M】

证据：[默认监听所有接口](E:/CODE/C++/Grinder/catkin_ws/src/grinder_scheduler/src/grinder_scheduler/scheduler_node.py:882)、[仅校验目标设备后分发](E:/CODE/C++/Grinder/catkin_ws/src/grinder_scheduler/src/grinder_scheduler/sl_linka_adapter.py:403)。

SL-LinkA TCP 接口直接执行控制、任务及地图请求，服务端路径中没有会话认证、客户端控制权租约或命令去重。CRC 验证只能检查帧损坏。只要网络可达且能构造协议帧，就能进入这些处理流程；是否暴露到公网取决于部署，本次未检查现场网络。

建议：协议入口认证、单一控制会话租约和重放/重复请求策略；地图读取与危险写操作分权。结合受信网络接口或网络访问控制。认证升级需同步 APP/SDK。

收益：降低未授权控制和多客户端抢占风险，并使命令责任可追溯。验收：未认证、过期会话、重放和第二控制者不能改变运行状态。

## 可靠性与验证问题

### F07 · P2 · 轮速斜坡启用时，超时停车仅减速一次【复现，S】

证据：[超时处理](E:/CODE/C++/Grinder/catkin_ws/src/grinder_chassis_driver/src/grinder_chassis_driver/chassis_driver_node.py:770)、[斜坡限幅](E:/CODE/C++/Grinder/catkin_ws/src/grinder_chassis_driver/src/grinder_chassis_driver/chassis_driver_node.py:931)。

首次超时将目标值改为零，但发送值受步长限制；下一次检查因为目标已是零而跳过写入。复现中由 100 RPM、步长 20 RPM 停在 **80 RPM**。当前受控 YAML 的 `enable_cmd_ramp_limit=false`，所以这是启用该选项时触发的缺陷。

建议：超时/急停绕过正常运动斜坡，强制零速；需要受控减速时由独立发送周期持续推进，以已发送/反馈速度判断完成。同时把驱动安全超时改为单调时钟，避免 ROS 时间暂停或回拨影响判断。

收益：选项切换不破坏停车保证。补充多周期超时、时间回拨和斜坡开关组合测试。

### F08 · P2 · TaskCommand 不验证目标任务身份【静态确认，S】

证据：[任务命令处理](E:/CODE/C++/Grinder/catkin_ws/src/grinder_scheduler/src/grinder_scheduler/scheduler_node.py:8536)。

请求的 `task_id` 只用于响应赋值，启动/暂停/恢复实际操作的是当前任务。旧任务 A 的延迟 START 可以启动当前任务 B，同时响应仍标识 A；未知命令走 `else` 停止，缺少显式参数错误。

建议：对启动、恢复等操作验证任务 ID、计划版本及运行代号；明确 STOP 是否允许全局生效；未知枚举返回非法参数。全局急停仍应保持独立安全语义。

收益：防止重连、重试和多客户端情况下错任务启动，使响应与实际对象一致。

### F09 · P2 · 地图叠加层不是一致性快照【静态确认，M】

证据：[保存](E:/CODE/C++/Grinder/catkin_ws/src/grinder_scheduler/src/grinder_scheduler/map_service.py:912)、[加载](E:/CODE/C++/Grinder/catkin_ws/src/grinder_scheduler/src/grinder_scheduler/map_service.py:929)。

`overlay.npy` 直接覆盖写入，JSON 才使用临时文件替换。进程中断或写盘失败可以留下截断 NPY，或新 NPY 配旧区域元数据。加载先修改区域状态，再读取 NPY；后者失败时也可能留下部分应用的状态。

建议：按版本目录写完整快照，验证并 fsync 后原子切换清单；加载到临时对象全部验证通过再提交；保留上一完整版本。

收益：减少重启丢编辑、区域/栅格不一致和恢复困难。验收：在两个文件写入之间注入异常，重启后只能得到旧版或新版完整快照。

### F10 · P2 · 回环重投影瞬时失败后不重试同一版本【静态确认，M】

证据：[重投影循环与失败分支](E:/CODE/C++/Grinder/catkin_ws/src/mapping/cloud_to_occupancy_grid/src/cloud_to_occupancy_grid_node.cpp:354)。

关键帧服务调用失败后仍把该 revision 写入 `completed_revision_`。循环条件要求 requested 大于 completed，因此即便服务恢复，也不会重试该次回环，直到下一次图版本更新。如果之后没有新回环，旧栅格不会补上这次校正。

建议：区分“成功应用版本”和“失败尝试版本”；有上限、带退避地重试，较新 revision 到来时取消旧任务；通过诊断暴露地图版本落后状态。避免直接删除现有防忙循环逻辑。

收益：瞬时服务错误可自行恢复，降低回环后旧墙、错位栅格持续存在的概率。验收：首次服务失败后恢复，无新 revision 也能最终应用目标版本。

### F11 · P2 · 视频管道阻塞可能拖住调度状态更新【静态确认，M】

证据：[远端推流持锁写 stdin](E:/CODE/C++/Grinder/catkin_ws/src/grinder_scheduler/src/grinder_scheduler/media_streamer.py:38)、[本地推流](E:/CODE/C++/Grinder/catkin_ws/src/grinder_scheduler/src/grinder_scheduler/local_rtsp_server.py:57)、[调度读取视频状态](E:/CODE/C++/Grinder/catkin_ws/src/grinder_scheduler/src/grinder_scheduler/scheduler_node.py:6855)。

视频帧在状态锁内同步写 FFmpeg 管道，没有写入总截止时间。FFmpeg 下游不再消费时，写入可阻塞；`get_state()` 需要同一锁。调度 `_tick()` 调用状态发布，因此一次视频故障可能阻塞该定时回调，连带延后后续任务进度、路径和磨盘保护周期。

建议：独立视频写线程、只保留最新帧的有界队列、写入截止时间和子进程回收；状态快照无需等待 IO 锁。关键安全定时器与媒体状态采集解耦。

收益：视频或网络故障局限于视频链路。验收：假 FFmpeg 停止读取时，调度心跳及停车命令延迟仍满足项目设定的上限。

### F12 · P2 · TCP 连接和部分任务队列缺乏总资源上限【静态确认，M】

证据：[每连接工作线程](E:/CODE/C++/Grinder/catkin_ws/src/grinder_scheduler/src/grinder_scheduler/sl_linka_adapter.py:330)、[删除请求入池](E:/CODE/C++/Grinder/catkin_ws/src/grinder_scheduler/src/grinder_scheduler/sl_linka_adapter.py:440)、[ThreadingMixIn 服务](E:/CODE/C++/Grinder/catkin_ws/src/grinder_scheduler/src/grinder_scheduler/sl_linka_adapter.py:737)。

每条连接创建多个工作线程和状态线程，没有活动连接总数限制、读写截止时间；`request_queue_size=16` 只控制待接受连接积压。大多数队列已有边界，但删除请求的 `ThreadPoolExecutor` 直接 submit，无 pending 槽位。慢读客户端还能把 `sendall` 和发送锁长期占住。

建议：限制活动连接数、会话空闲时间和单次发送时限；删除队列加容量/合并与明确拒绝响应；全局控制预算并测试断连清理。

收益：减少慢客户端和请求突发造成的线程、内存及响应延迟增长。应以连接压力测试实测 RSS、线程数、控制 P99 延迟。

### F13 · P2 · 测试过时、互相污染，未纳入常规构建门禁【复现/静态确认，M】

证据：[直接覆盖 sys.modules](E:/CODE/C++/Grinder/catkin_ws/src/grinder_scheduler/test/test_live_map_save.py:27)、[过时服务替身](E:/CODE/C++/Grinder/catkin_ws/src/grinder_scheduler/test/test_live_map_save.py:72)、[指标测试的协议替身](E:/CODE/C++/Grinder/catkin_ws/src/grinder_scheduler/test/test_metrics.py:85)。

设置正确 PYTHONPATH 后，Windows 聚合发现运行 19 项、5 项错误：包括缺失 `SetSuperLioInitialPose` 和后续用例读到被替换的业务模块。WSL 独立执行也确认指标测试缺少 `MSG_ID_MAP_REQUEST_RESULT`。两个自研包 CMake 未注册这些 Python 测试，根仓库未发现受控 CI 工作流。现有底盘测试主要覆盖协议/模型，没有覆盖本次暴露的节点控制行为。

建议：更新协议/服务替身，限制 mock 生命周期，或统一每文件独立进程；将行为测试注册到 Catkin/CI；增加停车、并发 START/STOP、地图快照损坏、客户端背压测试。使用可重复的 Linux/ROS 环境运行集成测试。

收益：让测试结果可信，且每次修改都能防止已修故障回归。验收：单独执行与聚合执行结果一致，干净环境测试门禁能发现本报告的回归案例。

### F14 · P2 · 依赖声明与部署说明不闭合【环境失败/静态确认，S/M】

证据：[SDK requirements](E:/CODE/C++/Grinder/third_party/sl_linka/sdk/python/requirements.txt:2)、[底盘实际导入 geometry_msgs](E:/CODE/C++/Grinder/catkin_ws/src/grinder_chassis_driver/src/grinder_chassis_driver/chassis_driver_node.py:10)、[底盘依赖声明](E:/CODE/C++/Grinder/catkin_ws/src/grinder_chassis_driver/package.xml:1)。

WSL Ubuntu 20.04 的系统 Protobuf 无法导入 SDK 生成代码所需的 `google.protobuf.internal.builder`。SDK 虽已有最低版本要求，但主 README/平台构建流程未包含这份 Python requirements 的安装/预检。底盘直接使用 `geometry_msgs/Twist` 却没有对应显式包依赖，当前可能因其他依赖传递安装而被掩盖。

建议：整理与 Python 3.8/生成器匹配的受控运行依赖；构建前验证导入与版本；补全直接 ROS 依赖，更新安装路径。修改 package.xml 后做 Catkin 构建。

收益：减少“开发机可用、干净设备失败”的交付问题。当前 Protobuf 失败属于本次环境依赖不足，不能据此断言板端同样失败。

### F15 · P2 · MQTT 凭据通过未启用 TLS 的连接发送【静态确认，M】

证据：[username/password 后直接 connect_async](E:/CODE/C++/Grinder/catkin_ws/src/grinder_scheduler/src/grinder_scheduler/platform_integration.py:814)、[默认端口 1883](E:/CODE/C++/Grinder/catkin_ws/src/grinder_scheduler/src/grinder_scheduler/scheduler_node.py:922)。

代码未调用 TLS 配置接口。如果没有部署层 VPN/加密隧道，凭据和任务遥测在传输中缺少机密性保护。本次没有验证外部 Broker 的配置。

建议：支持 CA 校验的 TLS 连接及可配置端点，按设备授权 topic；如采用隧道明确部署前提并预检。凭据继续留在本地秘密配置中。

收益：降低网络窃听造成的凭据及生产数据泄露风险。验收：错误证书拒绝连接，合法设备只能访问授权 topic。

### F16 · P2 · 高频串口日志无容量上限且同步写盘【静态确认，S/M】

证据：[受控配置默认开启](E:/CODE/C++/Grinder/catkin_ws/src/grinder_chassis_driver/config/chassis_driver.yaml:61)、[逐条打开追加](E:/CODE/C++/Grinder/catkin_ws/src/grinder_chassis_driver/src/grinder_chassis_driver/modbus_transport.py:377)。

文件按启动时间命名，但代码中没有大小轮转、保留数量或容量清理；高频 RX/TX 及运动事件持续追加。同步 IO 也增加控制链路抖动，吞掉写日志异常只能避免报错传播，不能限制磁盘占用和 IO 等待。

建议：采用有界异步日志队列、轮转与磁盘预算；正式配置默认采样或关闭全帧日志，仅故障窗口临时开启。

收益：避免长时间运行耗尽存储，降低串口回调写盘开销；保留足够诊断证据。验收：持续压力运行后磁盘占用不超过预算，比较开启日志前后控制延迟。

## 结构改进建议（P3）

1. **按故障边界拆分调度器（L）**：`scheduler_node.py` 当前 12,057 行；符号统计包含 311 个方法及 1 个类。优先抽取任务状态转换、控制仲裁、地图存储和协议 DTO 校验。先建立行为测试，再分阶段迁移。收益是缩小并发共享状态、降低单次改动影响范围；不要把单纯减少行数当成验收目标。
2. **修正文档与实际目录（S）**：AGENTS.md 中 `cloud_to_occupancy_grid`、`super_lio_loop`、`rf2o_laser_odometry` 位置已落后于 `mapping/`、`localization/`；README 使用特定机器绝对链接。收益是减少定位、部署和维护误导。本次不直接改导航文件，避免将审计建议混为已实施修复。
3. **补齐长时间建图策略（M/L）**：工业回环达到 `max_keyframes=2000` 后返回 `CAPACITY_REACHED` 并停止新增关键帧。已有容量保护是好事；建议明确任务规模、容量告警、继续建图/分段保存策略，并测试达到上限后的地图完整性。不能仅凭该上限断言已经内存泄漏或建图崩溃。

已有值得保留的设计：Modbus CRC/响应地址校验、左右轮单帧写入、多数 SL-LinkA 队列有界、手动命令单调时钟看门狗、地图资产版本号，以及 Super-LIO 地图包 fsync/原子提交。整改应沿用这些机制。

## 验证结果

| 检查 | 结果与解释 |
|---|---|
| 底盘离线单测，WSL Ubuntu 20.04 | 8/8 通过；不连接硬件 |
| 假传输安全链路复现 | 3 个场景全部复现；WSL 正常退出 |
| Python 3.8 AST 语法 | 45 个选定自研/集成 Python 文件通过；不证明依赖及运行时 API 兼容 |
| XML 解析 | 16 个选定 launch/package 文件通过；不等于 roslaunch 能启动 |
| Bash 语法 | 6 个工作区构建/启动/依赖脚本通过 |
| 调度独立进程测试，WSL | planning_direction 4/4、platform_integration 1/1、super_lio_mode_manager 11/11 通过；metrics 4 项中 1 错误；两个文件被 Protobuf 依赖阻止导入 |
| 调度聚合测试，    | 设置 PYTHONPATH 后 19 项中 5 错误；含过时替身和模块污染 |
| Catkin/C++/Gazebo/硬件 | 未执行；无完整 ROS/真机集成结论 |

复现脚本：[reproduce_safety_cache.py](E:/CODE/C++/Grinder/artifacts/repository-audit-20260928/reproduce_safety_cache.py)。脚本从当前驱动 AST 提取原方法，使用假对象，无 ROS 启动、串口和网络操作；它断言当前缺陷存在，修复后应转为正式回归测试。

验证脚本：[verify.py](E:/CODE/C++/Grinder/artifacts/repository-audit-20260928/verify.py)。原始结果：[verification.json](E:/CODE/C++/Grinder/artifacts/repository-audit-20260928/verification.json)。

## 建议实施顺序与收益验收

| 顺序 | 内容 | 主要收益 | 验收依据 |
|---|---|---|---|
| 1 | F01/F02/F03/F07/F08 | 停车可靠、急停优先、错误路径/错误任务不能执行 | 帧级测试、并发屏障测试、路径约束测试 |
| 2 | F04/F05/F13/F14 | 源码/配置/依赖可恢复，测试结果可信 | 干净克隆后构建、仿真启动、测试门禁 |
| 3 | F06/F12/F15 | 控制会话可信、网络故障资源有界、遥测加密 | 会话/权限测试、慢客户端压力、证书校验 |
| 4 | F09/F10/F11/F16 | 断电/服务/视频/日志故障可恢复且不拖累主链路 | 故障注入、长时间运行、延迟/RSS/磁盘预算 |
| 5 | 调度模块拆分与文档整理 | 降低后续维护成本 | 每阶段保持行为测试通过，明确模块状态所有者 |

收益以可验证行为描述，不虚构性能百分比或节省工时。建议整改前后记录停车延迟 P95/P99、串口失败后的恢复耗时、干净环境部署成功率、长时间 RSS/磁盘曲线。
