# Grinder 全局复审：正确性、效率、规划与 SL-LinkA

审计日期：2026-10-05。结论基于当前工作区源码与离线探针，**本轮没有修改业务源码，也没有回滚**。用户已回复“没有改动就算了”。机器人 HEAD 为 `2c02e72ff4bcd051e10e98869e54a4d37d8e60b3`；Android HEAD 为 `d4a0258e75d43e9f68cb9539a8e2ca7ce4148da7`。F04/F05 的既有授权修复仍暂存，APP 工作树干净。

主要结论：先修停车/急停、规划越过约束、读路径改变执行状态和会话串命令，再优化整图重复处理、规划阻塞、主线程解码、发送背压与视频/磁盘 IO。当前已有 C++ 内核、地图缓存、批量发送及部分有界队列；性能问题不能概括成“Python 太慢”，也不应通过统一提高频率解决。

本报告将此前 F01–F16、联合审计 R01–R13 合并到全局链路，避免把同一问题重复计数。新增规划、缓存、协议过载、导航确认及资源问题直接列在下面。P1 表示优先阻断危险动作或错误任务；P2 表示可靠性、效率和交付问题；P3 为结构优化。S/M/L 表示预计修改范围小/跨模块/较大，不是工期承诺。

## 覆盖范围与证据边界

| 链路 | 核查范围 | 证据/限制 |
| --- | --- | --- |
| 调度与底盘 | 启动、停止、急停、手动控制、任务执行、输出缓存、Modbus、超时与日志 | 生产方法 AST、假传输复现；没有真实下位机反馈 |
| 覆盖规划 | planner_adapter、mst27 Python/C++、Hybrid 连接、区域选择/重复、缓存、执行适配 | 本轮重点深入；纯方法探针；没有把 Windows `.so` 当 Linux 内核执行 |
| 导航 | 实际 teb/rpp launch、RPP 跟踪与固定距离停车、costmap 参数和 footprint | 静态核查项目配置及相关控制分支；没有逐行重审全部上游 ROS navigation |
| SL-LinkA | proto、Python/Embedded 帧、机器人分发/发送/队列、Android 收发/组包/请求关联 | 双端 proto 一致；帧和过载离线复现；没有真实 Wi-Fi 吞吐测量 |
| 地图/定位 | overlay 合成/持久化、预览/资产版本、生命周期/健康检查、回环/重投影、栅格发布 | 静态核查；有界点云容量和全图开销已定位；需要 Linux 长时间验证 |
| APP | TCP 会话、状态流、任务地图、遥控、图片、视频、缓存、后台/重连/进程恢复、OTA | 结合上轮当前源码 JVM 验证；没有平板生命周期/帧率实机验证 |
| 外部服务/交付 | 上传/MQTT、日志/指标、依赖声明、构建启动、配置与源码恢复 | 未读取或输出本地秘密配置；未连接平台和硬件 |

“全局”指覆盖系统各主要数据与控制链路，不等于已经逐行证明所有第三方源码无缺陷。构建/产物/运行目录不作为源码递归审查对象。

## 第一优先级：正确停止与正确执行

### G01 · P1 · 驱动停车仍会被旧整块缓存跳过（F01/F07，M）

[整块写缓存](E:/CODE/C++/Grinder/catkin_ws/src/grinder_chassis_driver/src/grinder_chassis_driver/chassis_driver_node.py:665) 没有被轮速/单寄存器写失效，[安全停机](E:/CODE/C++/Grinder/catkin_ws/src/grinder_chassis_driver/src/grinder_chassis_driver/chassis_driver_node.py:692) 又未强制写。本轮重跑生产方法探针：启动零输出→发送 100/100→停车，**新增帧 0**；单独开启磨盘后停车同样 0。启用轮速斜坡时超时仅减到 80/80，默认斜坡关闭。

建议：安全输出强制写；局部写同步已确认寄存器状态或失效缓存；超时/急停绕过斜坡，重连先同步安全输出。收益是停机真正发帧，不是只改内存。验收需要帧内容、通信失败/重连屏障及最终零速/磨盘反馈；方法探针不能替代实机反馈。

### G02 · P1 · 急停会被旧启动/手动线程覆盖（F02，M）

[启动](E:/CODE/C++/Grinder/catkin_ws/src/grinder_scheduler/src/grinder_scheduler/scheduler_node.py:5956) 在定位等阻塞步骤之后继续提交输出，[急停](E:/CODE/C++/Grinder/catkin_ws/src/grinder_scheduler/src/grinder_scheduler/scheduler_node.py:8206) 缺共享取消代号和锁存。本轮在实际生产方法中插入定位屏障：屏障内急停，释放后最终 **RUNNING，并再次 task_enable=true、disc_enable=true**。

建议：急停锁存、显式复位、execution generation；每个阻塞返回和输出提交前检查，底盘也拒绝失效执行的非零输出。不要让整个定位/规划持有阻塞急停的大锁。收益是停止拥有最终优先权；验收在定位、规划、输出处分别插入急停并核验释放后不重新使能。

### G03 · P1 · 规划连接失败后会主动降低避障约束（扩展 F03，M）

此前只发现导入失败后的简化规划。本轮确认：即使 mst27 正常加载，[区域连接](E:/CODE/C++/Grinder/catkin_ws/src/grinder_scheduler/src/grinder_scheduler/planner_adapter.py:538) 也依次尝试 Hybrid、膨胀图 A*、**未膨胀图 A***，最终 [直线回退](E:/CODE/C++/Grinder/catkin_ws/src/grinder_scheduler/src/grinder_scheduler/planner_adapter.py:581)。连接头尾还有直线补齐；[Hybrid](E:/CODE/C++/Grinder/catkin_ws/src/grinder_scheduler/src/grinder_scheduler/planner_adapter.py:848) 输入主要是手工障碍多边形，未把原始占据格/裁剪图作为完整碰撞输入。

mst27 内部的最终碰撞检查只验证单区输出，之后适配器拼接的整条路径没有同等最终复检。本轮调用生产直线方法连接被贯穿墙隔开的两点，再调用生产碰撞检查，结果 **collision_free=false**。这是直线方法级复现；完整任务路径风险还由上述生产调用链静态确认，不声称已经跑过整机。

建议：不可达应明确失败，禁止可执行路径自动降级净距或直线穿越；组合路径统一对有效地图、裁剪边界及车辆 footprint 校验，首尾衔接也校验。收益是拒绝错误路线，减少碰撞和无效导航重试。验收包含不连通区域、窄通道、栅格独有障碍、裁剪外连接、端点吸附后空隙及所有回退分支。

### G04 · P1 · 查询显示路径会修改运行中的任务（R01，M）

APP 的显示补取进入 [路径规划接口](E:/CODE/C++/Grinder/catkin_ws/src/grinder_scheduler/src/grinder_scheduler/scheduler_node.py:8657)，[缓存命中](E:/CODE/C++/Grinder/catkin_ws/src/grinder_scheduler/src/grinder_scheduler/scheduler_node.py:11846) 也设置 READY；未命中则改共享路径/游标。本轮重跑实际分支得到 **READY、exec_active=true，执行周期提前返回**。离线地图请求还临时替换全局 map_service。

建议：查询当前执行快照完全只读，按 execution_id/path_version/map_revision 获取；独立规划使用局部不可变上下文，运行中重规划显式受控。收益同时覆盖正常看路径、多客户端、历史地图查询、APP 返回和重连，且避免不必要的重新规划。验收在 RUNNING 查询路径/离线地图，执行状态、目标和游标都保持一致。

### G05 · P1 · 遥控后台续发与旧会话排队命令可恢复非零动作（R02/R03，M）

APP [锁定方向盘](E:/CODE/grindingrobot/grindingrobot/feature/map/src/main/java/com/sinelynx/grindingrobot/feature/map/ui/VirtualDirectionPad.kt:232) 在锁定时不随手势结束归零；[遥控循环](E:/CODE/grindingrobot/grindingrobot/feature/main/src/main/java/com/sinelynx/grindingrobot/feature/main/viewmodel/RemoteSettingsViewModel.kt:313) 缺 ON_STOP/控制会话约束。[TcpService 排队发送](E:/CODE/grindingrobot/grindingrobot/core/tcp/src/main/java/com/sinelynx/grindingrobot/core/tcp/TcpService.kt:291) 执行时读取全局 outputStream。上轮实际 JVM 探针已证明旧排队字节被写入新连接；本轮 APP 代码未变化。锁定后台风险为静态确认，未做平板遥控测试。

建议：失焦/后台清遥控意图；单一有序发送器捕获 session generation，断线取消旧命令，新会话必须重新取得控制权并由新操作产生运动。收益是重连恢复数据时不会顺便恢复旧动作。

### G06 · P1 · 控制权无认证和租约，多客户端可竞争任务（F06，M）

服务器默认监听 0.0.0.0，仅校验 dst_id，未见控制身份认证、独占控制租约和查看/操控权限隔离。CRC 与设备 ID 不能证明操作者身份。

建议：会话认证、带过期时间的控制租约、命令权限和主动撤销；普通查看端不获得运动权限。收益是避免多个 APP 相互覆盖状态和输出，并为重连/请求去重提供会话边界。验收双客户端争用、租约到期、重启与旧会话重放。

### G07 · P1 · 规划缓存遗漏安全参数和原始地图修订（新增，M）

[缓存键](E:/CODE/C++/Grinder/catkin_ws/src/grinder_scheduler/src/grinder_scheduler/scheduler_node.py:5035) 含区域与 map_version，但不含 inflation_radius、vehicle geometry、path_spacing 等完整规划参数，也不含 raw map 的 stamp/seq/content revision。[SettingsWrite](E:/CODE/C++/Grinder/catkin_ws/src/grinder_scheduler/src/grinder_scheduler/scheduler_node.py:7992) 改这些参数后未清该键；纯参数更新不一定产生地图编辑版本变化。[set_raw_map](E:/CODE/C++/Grinder/catkin_ws/src/grinder_scheduler/src/grinder_scheduler/map_service.py:208) 更新原图也不增加 overlay 的 map_version。

生产缓存键探针将膨胀半径 0.1→1.5、间距 0.7→2.0、原图 stamp/seq 改变，得到 **相同键**。同任务/区域/位姿下 force_replan=false 可命中旧路径。TaskConfig 路径已有显式失效，这个问题主要是 SettingsWrite 和原图更新等旁路，不能把所有配置写入都说成无失效。

建议：不可变 PlanningContext，完整算法参数/算法版本/地图内容修订作为键；统一失效规则。区分导航实时障碍和静态规划地图的版本策略，避免拿时间戳简单替代内容版本造成每秒强制重算。收益是既不错误复用，也不无谓重算。验收逐项改变安全参数/地图内容，并验证需要失效及允许复用的两类情况。

### G08 · P1 · 非法区域选择会静默回退到其他区域（新增，S）

[调度区域选择](E:/CODE/C++/Grinder/catkin_ws/src/grinder_scheduler/src/grinder_scheduler/scheduler_node.py:4487) 对非空显式选择全部无效时回退 active region，若 active 也无效则返回全部有效区域；[适配器](E:/CODE/C++/Grinder/catkin_ws/src/grinder_scheduler/src/grinder_scheduler/planner_adapter.py:444) 又有回退所有已准备区域的分支。生产方法探针指定 missing，在无 active 时得到 A/B，有 active=A 时得到 A，均不是请求的区域。过期 ID、串地图或拼写问题因此可能换区/扩大研磨范围。

建议：非空显式选择有无效 ID 时返回明确错误，只有明确“全部区域”请求允许扩大；反馈缺失 ID。收益是任务边界可预测，避免错误加工。验收全不匹配、部分不匹配、重复 ID 及地图切换。

## 第二优先级：规划效率与可解释性

### G09 · P2 · 多圈规划复用点对象，圈次标签被最后一圈覆盖（新增，S）

[重复展开](E:/CODE/C++/Grinder/catkin_ws/src/grinder_scheduler/src/grinder_scheduler/planner_adapter.py:469) 只复制 list，随后写每个对象的 path_type。三圈的九个引用只对应三个对象，最后全部标为 lap3。探针验证的是生产写法的 Python 引用语义；[圈次进度](E:/CODE/C++/Grinder/catkin_ws/src/grinder_scheduler/src/grinder_scheduler/scheduler_node.py:7252) 实际解析这些标签，完成计数和展示会受影响。几何点数仍展开，不能据此说只走了一圈。

建议：新建轻量不可变点记录，或将 lap/region 元数据存于段对象，避免改底层共享点。收益是正确圈次/结果统计，也可减少完整点对象复制。验收 1/2/3 圈、倒序圈、进度与最终计数。

### G10 · P2 · 每个区域重复拷贝和预处理整幅地图（新增，M，效率高收益）

[每区调用](E:/CODE/C++/Grinder/catkin_ws/src/grinder_scheduler/src/grinder_scheduler/planner_adapter.py:332) 传 grid_map.copy；mst27 再 [复制、膨胀和构造角度规划图](E:/CODE/C++/Grinder/third_party/path_planner/mst27/mst27.py:2211)，并转换 int32 网格/全图区域 mask 给 C++。R 个区域、N 个格的重复预处理约为 O(RN)，即使每个区域很小也扫描全图。默认 3000×3000 网格，一个 int32 全图就是 36 MB；多份数组并存，不能把这一数值当总 RSS。

建议：按有效地图修订缓存原始障碍图/距离场；同角度共用预处理；对区域加安全边界做 ROI，坐标转换集中处理；C++ 输入尽量一次转换，使用紧凑 occupancy 类型。收益是减少重复全图遍历、临时数组和 GC/分配峰值。10 个同图同角度区域的可共享预处理可由 10 次降到 1 次，这是操作次数目标，**不是宣称总耗时降低 90%**。验收不同角度/净距的缓存隔离，路径几何与碰撞约束一致，板端 P95/RSS 对比。

### G11 · P2 · C++ 内核计算仍持有 Python GIL（新增，M，响应性高收益）

[plan_core](E:/CODE/C++/Grinder/third_party/path_planner/mst27/cpp/mst27_cpp.cpp:1595) 解析 Python 输入后直接调用纯 C++ plan_all_stages/densify_path，未见 gil_scoped_release 或等效处理。pybind 默认持 GIL；长计算期间，同进程 Python 收发/定时器/急停处理不能正常并行执行 Python 代码。此为源码确认，阻塞多久尚未在板端测量。

建议：取得独立 C++ 输入后仅对纯计算段释放 GIL，恢复后再创建 Python 结果；更进一步将长规划置于隔离进程，带取消代号和截止时间。不要对访问 py::list/tuple 的整函数盲目释放 GIL。收益主要是控制响应与调度抖动改善，不是自动加快单次算法。释放 GIL 后共享状态并发会更明显，必须与 G02/G04/G07 的状态所有权一起处理。

### G12 · P2 · A* 启发式不匹配八邻域，并重复展开过期堆项（新增，S/M）

[Python A*](E:/CODE/C++/Grinder/third_party/path_planner/mst27/mst27.py:1651) 和 [C++ A*](E:/CODE/C++/Grinder/third_party/path_planner/mst27/cpp/mst27_cpp.cpp:235) 允许 1.414 代价的对角移动，却用 Manhattan 启发式；没有对已过期优先队列项作标准跳过。启发式会高估，首次到达目标不能保证最短路径；固定 20×20、seed0 栅格中生产 Python 返回 33.898，Dijkstra 32.726，**该样例多 3.58%**。未实测 C++ 相同地图的输出。

建议：Octile 启发式、显式维护 stale/closed 规则和可取消搜索预算，保持不穿角；Python/C++ 用同一组黄金地图验证。收益是更合理的连接长度、可控搜索开销；更准确启发式并不保证每幅地图搜索更快，需分别记录展开数/耗时/路径长。

### G13 · P2 · 小请求可展开成超大规划结果，缺统一预算（新增，S/M）

[repeat](E:/CODE/C++/Grinder/third_party/sl_linka/proto/sl_link.proto:507) 是 uint32；[接收](E:/CODE/C++/Grinder/catkin_ws/src/grinder_scheduler/src/grinder_scheduler/scheduler_node.py:8482) 与规划器只做 max(1, repeat)，没有业务上限，随后 range 展开。65 KB 的单帧限制无法限制圈数导致的输出膨胀。部分浮点/几何参数仅做最小值限制，也缺统一有限值、顶点数、路径点数/字节与规划时间约束。

建议：解析后先校验 finite、业务范围、区域/顶点/圈数、预估输出上限，计算中检查取消与预算；返回可理解的拒绝原因。收益是防止错误配置耗尽 CPU/RAM，让正常任务不被拖住。验收最大 uint32 圈数、NaN/Inf、极小点间距、大量区域，不允许先分配巨大输出再检查。

## SL-LinkA：吞吐、时延与事务语义

帧格式的 15 字节头、CRC16、尾字节、小端序在 Python/Embedded/Android 主要实现中一致；双端 proto 当前一致。地图请求已有 request_id、结果与分片，路径已有版本/分片；历史接口已有 execution_id，地图资产已有 map_revision。需要把这些身份贯穿运行态，不能误报成协议完全没有这些字段。CRC 防传输损坏，不提供鉴权；目前的“加密标志”也不代表有加密实现。

### G14 · P2 · “最新优先”队列会丢失配置/任务事务（新增，M）

[有序 worker](E:/CODE/C++/Grinder/catkin_ws/src/grinder_scheduler/src/grinder_scheduler/sl_linka_adapter.py:16) 满时 popleft，[默认分发](E:/CODE/C++/Grinder/catkin_ws/src/grinder_scheduler/src/grinder_scheduler/sl_linka_adapter.py:448) 把一般请求放入它。丢弃只日志记录，没有关联的 BUSY/取消响应。生产类屏障探针：积压 TaskConfig、START 后再加入 settings，**TaskConfig 被丢弃，START 仍执行**。测试用容量 2 强制触发同一规则，实际队列容量 32。

建议：只对可合并的状态/预览查询按键 latest-wins；配置、任务命令、地图修改用有界可靠队列，超限返回 BUSY 和原请求身份。收益是保持事务顺序，避免 APP 超时重试放大负载与 START 使用旧配置。

### G15 · P2 · 大数据与控制 ACK 共用阻塞发送，连接总预算不足（F12/R11，M，效率高收益）

[bulk TX](E:/CODE/C++/Grinder/catkin_ws/src/grinder_scheduler/src/grinder_scheduler/sl_linka_adapter.py:623) 持 send_lock 合并约 128 KiB 后 sendall；常规 ACK/状态也争同一锁，没有应用层发送截止时间。bulk slots 已限为 2，但满时有同步发送回退。每连接有多个 worker/状态线程；全局连接、删除任务 pending 等仍缺上限。

建议：单一发送调度器，紧急控制结果/状态优先，大数据有界分片交错、写截止时间与每连接/全局字节预算；过慢客户端主动断开；查询合并，共用预序列化状态。收益是大图期间 ACK 的 P99 可控、避免慢客户端牵连，CPU/线程/RSS 随客户端数受控。必须测慢读客户端下“收到命令→实际安全输出”和“收到命令→ACK”两个时延，不能只看 ACK。

4 KiB payload 的纯帧开销约 18/4096=0.44%；因此首要问题是排队、锁、重复编码及客户端处理，不能把分片头当主要带宽瓶颈。更大的 TCP write 可以提高吞吐，但需要同时限制控制等待。

### G16 · P2 · 帧解析缺版本/标志校验和半帧截止时间（新增，S/M；合并 R04）

[Python parser](E:/CODE/C++/Grinder/third_party/sl_linka/sdk/python/sl_link/frame/frame.py:93) 和 [Embedded parser](E:/CODE/C++/Grinder/third_party/sl_linka/sdk/embedded/src/sl_packet.c:59) 解头后未拒绝不支持的 version/flags；服务器只过滤目的 ID。离线生产 parser 接受 version=99、encrypted flag=2 的普通明文帧。伪头声明 65535 payload 后接一个有效帧，得到 0 个帧并继续等剩余字节，没有解析截止时间。它不是无限分配，而是会话内恢复/资源占用问题。Android 另有零长帧残留及断线 parser 未复位，见既有 R04 探针。

建议：限制版本/标志/消息类型可用长度，按会话复位 parser，半帧时间和字节预算，失败后明确重同步策略；跨 SDK 统一空载荷/损坏/断线向量。收益是断线和损坏流可恢复，避免把未知格式解释成有效控制。

### G17 · P2 · ACK/任务身份未形成请求到执行的闭环（F08/R07，M）

服务器会回 ack_seq，但 APP 部分业务响应丢掉该值，订阅“第一个响应”或只按 task_id 匹配。TaskCommand 会回显请求 task_id，却操作当前任务，未验证其目标，未知枚举落 STOP。详情与实际方法位置见 [既有联合报告](E:/CODE/C++/Grinder/artifacts/repository-app-audit-20261004/AUDIT.md:100)。

建议：session_id+operation_id/request_id 关联；任务命令携执行身份和预期状态/版本，重试幂等、重复 ACK 可识别；非法枚举明确拒绝。收益是迟到响应不误认、重试不重复执行。传输发送成功、请求接收、操作完成、硬件确认必须是不同语义。

### G18 · P2 · 地图并发版本字段未执行，持久化不是一致快照（新增 + F09，M）

[expected_map_version](E:/CODE/C++/Grinder/third_party/sl_linka/proto/sl_link.proto:663) 已定义，但 [MapEdit handler](E:/CODE/C++/Grinder/catkin_ws/src/grinder_scheduler/src/grinder_scheduler/scheduler_node.py:11335) 未读取校验。map_id 检查不能阻止两客户端基于同图旧版本互相覆盖。edit_id 也未形成明确去重闭环。overlay.npy 与 JSON 又分开落盘，不能保证掉电后一致。

建议：在同一编辑锁内校验并提交版本，冲突返回当前版本；edit_id 幂等记录；文件以完整版本目录/manifest+原子指针切换，加载验证完再替换内存。收益是地图编辑可追踪、减少覆盖和坏状态重建。验收并发旧版本、重复编辑、两文件写入间断电、损坏 NPY。

### G19 · P2 · 路径组包/缓存只有局部上限，未完成请求可长期积累（R08，S/M）

Android TaskPathChunkAssembler 按 request_id 保留未完成组包，没有 TTL/总条目/总字节/会话 clear。既有生产 Kotlin 探针：1000 个各 4096 字节不完整请求保留约 4.096 MB payload，尚不含对象。TaskPathStream 按任务保留缓存也缺统一全局预算。

建议：统一 session+task+version 身份；TTL、LRU、全局字节/条目预算，终态和会话切换清理；分片重传只补缺失。收益是长期操作 RSS 不持续增长，并减少重连下载。

## 地图、图像、媒体和 IO 效率

### G20 · P2 · APP 接收有无用 Hex 开销，图片重复全量解码（R09，S/M，低成本高收益）

TcpManager 收包仍执行逐字节 Hex 格式化，实际日志已注释；尺寸检查先全量 Bitmap 解码再回收，展示又解码，部分在主线程。上轮桌面 100×4096 字节探针约 parser 23 ms、无用 Hex 546 ms；**只说明桌面样例的无用工作显著，不代表平板加速倍数**。

建议：先删掉/惰性开启无用格式化；用 inJustDecodeBounds 检查尺寸，像素预算，后台按目标尺寸解码，Bitmap 按版本复用，严格清理。收益是立即减少接收 CPU、界面卡顿和内存峰值。32 MiB 压缩输入上限不能代替解码像素上限；验收接收吞吐、UI 帧耗时、Bitmap 峰值/GC 和损坏图片。

### G21 · P2 · 默认全图快照在锁内反复扫描，调度再重复合成（新增，M，效率高收益）

[地图配置](E:/CODE/C++/Grinder/catkin_ws/src/mapping/cloud_to_occupancy_grid/config/mid360.yaml:18) 150×150 m、0.05 m 格为 3000×3000=900 万格。log_odds float+observed byte 基础数组约 **45 MB**。[occupancySnapshot](E:/CODE/C++/Grinder/catkin_ws/src/mapping/cloud_to_occupancy_grid/src/cloud_to_occupancy_grid_node.cpp:532) 在 map_mutex 内全图转换，1 Hz map 与 0.2 Hz tablet 分别调用；每次输出 9 MB 格值，buildMapMessage 还复制 data。调度 [compose](E:/CODE/C++/Grinder/catkin_ws/src/grinder_scheduler/src/grinder_scheduler/map_service.py:272) 又构建 int16 原图/overlay 副本、mask 并重新填区域。

建议：以栅格内容修订共用不可变 occupancy 快照、缓存 composed grid/多边形 mask；记录 dirty bounds，分块更新；缩短地图锁持有时间。导航坐标/范围保持稳定，APP 展示另用降采样或裁剪快照，不能为省资源随意改导航 origin。收益是减少内存带宽、锁等待和重复发布编码。9 MB 是未压缩 ROS 栅格，**不是 APP 的 PNG 流量**。测相同地图、少量 dirty cells、满图更新的 CPU/锁 P99 与订阅者 RSS。

### G22 · P2 · 视频阻塞可能影响调度，后台消费者仍耗资源（F11/R10，M）

FFmpeg stdin 写/flush 持状态锁；调度状态读取同锁。视频管道卡住时可以拖住状态周期。APP 地图 30 秒轮询主要按 connected 判断；播放器缺统一前后台/网络会话恢复协调，见既有联合报告。

建议：视频独立 writer、有界 latest frame、超时重启；状态读取不等视频 IO。按实际消费者控制抓帧/编码/轮询，后台保留必要控制状态；返回后按会话重建播放器。收益是降低空耗、电量/带宽，并确保视频故障不影响控制。验收冻结 FFmpeg/服务端、无人观看、后台/同 URL 重连。

### G23 · P2 · 串口诊断日志逐帧同步落盘且无轮转（F16，S）

[raw logging](E:/CODE/C++/Grinder/catkin_ws/src/grinder_chassis_driver/src/grinder_chassis_driver/modbus_transport.py:377) 每帧 open/write，默认配置启用，位于串口 IO 调用路径。建议有界异步队列、容量/时间轮转、常规采样，错误附近保留完整窗口；收益是降低 SD 卡写放大、长期磁盘占用和控制锁等待。验收磁盘慢/满也不拖住安全指令，明确丢日志计数。

### G24 · P2 · 平台上传完整读入文件后再次拼接（新增，S/M）

[_multipart_upload](E:/CODE/C++/Grinder/catkin_ws/src/grinder_scheduler/src/grinder_scheduler/platform_integration.py:660) handle.read 后 b''.join，大小 F 的文件在拼接期间至少约 2F payload 并存，另有 HTTP/响应开销。平台队列已有 maxsize=8、代号去旧任务和删除退避，不能说所有上传无界。

建议：流式 multipart/分段上传，文件大小/总时限限制，与规划分配板端内存预算；弱网可取消、断点或有界重试。收益是大 PCD 上传峰值不随 2F 增长，降低 OOM 和与规划争资源。验收大文件+慢网络+同时规划，确认上传失败不会扰动控制。

### G25 · P2 · 回环重投影失败即标完成，容量满后冻结（F10 + 长运行核查，M/L）

[重投影失败](E:/CODE/C++/Grinder/catkin_ws/src/mapping/cloud_to_occupancy_grid/src/cloud_to_occupancy_grid_node.cpp:403) 仍更新 completed revision，同一版本不再重试。[loop 上限](E:/CODE/C++/Grinder/catkin_ws/src/localization/super_lio_loop/src/super_lio_loop_node.cpp:377) 到 2000 keyframes 后冻结；每帧最多 8000 点，上限约 1600 万点，不能仅凭点数给出精确总 RSS。

建议：失败同版本有界退避，保留最后成功地图；长作业采用子图/关键帧降采样、落盘和滚动策略，清楚报告容量降级。收益是地图更一致、长运行内存与处理时间受控。不要单纯抬高上限。验收瞬时服务失败、同 revision 重试、容量满后导航/地图版本的行为。

## 导航、状态恢复与交付

### G26 · P2 · RPP 所称“连续确认”实际可以跨空白扫描累计（新增，S）

[候选计数](E:/CODE/C++/Grinder/catkin_ws/src/2-dnavigation-package/2dnavigation/regulated_pure_pursuit_controller/src/regulated_pure_pursuit_controller.cpp:747) 加一；非候选时调用 clearCollisionLatchIfSafe，但该方法在 [未锁存时直接返回](E:/CODE/C++/Grinder/catkin_ws/src/2-dnavigation-package/2dnavigation/regulated_pure_pursuit_controller/src/regulated_pure_pursuit_controller.cpp:893)，没有清 confirm_count。因此“命中→空白→命中”可能逐步累计到 15，不等于 15 连续帧。

建议：未锁存的非候选新扫描显式清候选连续数；锁存解除保留独立计数，不重复计算同一 seq。收益是减少间歇噪声导致的无故停走，改善任务用时。验收交替命中、14 次命中后清空、15 次连续命中、同一扫描重复控制周期。

### G27 · P2 · 停车确认时间与制动净距未形成可验证预算（新增，待动态验证，M）

RPP 当前预测碰撞关闭，固定距离 footprint 检测开启；配置 confirm_scans=15、front clearance=0.25 m、max speed=0.15 m/s，robot_diff_drive controller=10 Hz。按控制周期最多消费一份新扫描，理想连续采样从首次到第 15 次确认约 14/f：有效 10 Hz 时约 1.4 s，0.15 m/s 可前进约 0.21 m，**尚未加传输、制动、抖动和第一帧相位**。实际任务默认速度更低、实际参数可能被 runtime settings 覆盖，不能据此认定现场已发生碰撞。

建议：用实测有效扫描/控制频率、制动距离和最坏时延反推净距/确认时限；近危险区即时停止，远区滤波；评估预测检查覆盖转弯扫掠范围。收益是响应和误停间的明确取舍，避免靠盲目增速缩短作业时间。验收最低扫描率/CPU 高负载、最大允许速度、细障碍、转向及失效数据；这是硬件/仿真待验项。

### G28 · P2 · APP 尚不能保证任意后台/断网/进程回收后无缝恢复（R05/R06，M）

取消 UI 等待未释放地图请求 gate；失败 attempted/pathAttemptVersion 会阻止后续重试；重连没有统一 SYNCING→READY 的业务同步流程。部分任务显示缓存只在内存，状态报告缺运行执行身份与地图资产身份的完整对应，已有历史 execution_id 尚未贯穿实时控制。旧底盘设置缓存也可能残留。

建议：机器人拥有权威执行状态；APP 前台/重连先验证设备实例、执行/地图/路径版本，再补差量；持久化显示草稿和必要缓存，取消/失败释放请求门闩、可重试。收益是不串任务/地图，并减少重复全量下载。无缝恢复指恢复正确显示与查询，不能自动恢复遥控非零意图。

| 场景 | 当前判断 | 验收 |
| --- | --- | --- |
| 短后台、进程和连接仍在 | 有缓存恢复基础，受路径查询副作用影响 | 回来显示一致且机器人执行状态不变 |
| 在途请求取消后再回来 | gate/attempted 可能挡重试 | 可取消、无僵死等待、只补缺数据 |
| 断网/设备重启 | 缺统一会话业务同步 | 新可信状态后才允许操作，不重放旧命令 |
| Android 进程回收 | 内存显示状态不能完整恢复 | 先恢复必要草稿，再与机器人执行快照校对 |
| 锁定遥控/视频后切后台 | 控制与消费者生命周期存在缺口 | 遥控清零，视频资源收敛，回前台显式恢复 |

### G29 · P2 · 链路“丢包率”混合全局序号和连接生命周期（R12，S）

服务器 seq 全局递增，不同客户端交错；APP monitor 按设备统计且缺会话复位。既有 Kotlin 探针 100→0 计 65435 丢失，重复 0→0 得负计数。TCP 有序字节流下 seq 缺口也不能直接称无线丢包。

建议：每会话统计、处理 wrap/duplicate/reset，分开记录 parser CRC 错误、应用主动丢任务、重连与超时；收益是性能诊断可信，不把错误指标当网络问题调参。

### G30 · P2 · 构建/测试仍缺干净目标环境的闭环（F13/F14，M）

底盘使用 geometry_msgs 但依赖声明不完整；部分 scheduler 测试 mock 缺新服务/消息 ID；Python 测试未见统一 Catkin 门禁。先前完整 Linux 构建因 livox_ros_driver2 等环境依赖未完成。建议固定目标镜像和依赖安装，独立 mock，加入安全屏障/规划不变量/双端协议黄金向量；CI 在干净 clone 验证构建、配置预检和仿真。

收益是减少板端首次部署和回归故障；不能把 compileall/AST 或 JVM 单测当完整 ROS/Android 发布验证。

### G31 · P2 · 外部服务身份和安全传输未闭合（F15/R13，S/M）

MQTT 实现仍 username/password 无 tls_set，公开默认禁用已降低默认暴露；OTA 安装身份写死影响灰度和事件归属。建议启用服务前 TLS/证书校验、每安装生成持久身份，保留秘密本地覆盖。收益是账户传输和版本归属可追踪，不把多设备误认成一台。

## F04/F05 的交付状态

F04：索引已从 160000 gitlink 变为普通 Super-LIO 源码，来源/许可证/原修改备份已保留。F05：公开默认配置、可选本地覆盖、启动预检与仿真禁用外部服务已存在。**两项均尚未正式提交，克隆当前 HEAD 仍拿不到修复**；此前候选提交的独立克隆验证不能替代正式提交交付。按用户本轮要求不继续提交或修改，保留现状。后续验收需正式修复提交的干净克隆 + 目标 Linux 构建/无秘密配置仿真启动。

## 效率整改的收益排序

| 顺序 | 动作 | 收益对象 | 收益如何证明 |
| --- | --- | --- | --- |
| 1 | G04/G07：只读执行查询 + 正确缓存 | 避免重复规划和错用旧结果，同时保护执行 | 返回/重复查询规划次数为 0；参数/地图改变正确失效 |
| 2 | G20：移除无用 Hex、尺寸探测与异步复用 Bitmap | 低成本减少接收 CPU、主线程卡顿、解码峰值 | 平板帧时间、接收速率、CPU/Bitmap 峰值 |
| 3 | G10：共享预处理、ROI、紧凑数组 | 多区域规划耗时、内存带宽和峰值 RSS | 1/10/50 区域、同角度/混合角度 P50/P95，校验同等避障 |
| 4 | G11：释放纯计算段 GIL/隔离规划 | 控制响应、任务周期抖动 | 规划中停止接收→驱动零帧 P99；tick missed deadline |
| 5 | G15/G14：事务队列/发送优先级/背压 | 多客户端大图期间 ACK 和内存稳定 | 慢读/请求洪峰下控制 P99、队列字节、RSS/线程 |
| 6 | G21：复用地图快照/dirty 更新 | 周期 CPU、地图锁等待和复制 | 静态地图/局部变化/满图变化分别测 |
| 7 | G22/G23/G24：媒体、日志、上传 IO 隔离 | 长期运行稳定、磁盘、电量与峰值内存 | 慢盘/满盘/冻结编码器/弱网上传故障注入 |
| 8 | G12/G09/G26：连接质量、圈次与误停修正 | 总作业路径、有效加工统计和停走次数 | 路径长、加工覆盖率、重复次数、误停与任务总用时 |

效率要分三层：运算耗时、操作响应、整项作业时长。C++ 加速不能解决无效查询重规划；提高车速不能弥补连接路径错误；降低图像质量不能解决发送锁饿死 ACK。先消除浪费/阻塞，再按目标硬件测算法与速度参数。

19200 baud、8N1 下，2 寄存器一次读取请求+响应约 17 字节，纯线时约 8.9 ms，未含帧间隔/设备等待。正常 10 Hz 轮速、2 Hz 合并状态轮询尚无证据表明线路饱和。建议先看串口锁等待、超时/重试、停车实际发帧，**不优先提高 baud/轮询频率**。

## 已有机制与不应误报的事项

- Python CRC 已优先使用 binascii.crc_hqx，不是逐位 Python CRC 热点。
- MapCatalog handler 已在序列化后按 65000 字节删缩略图/拒绝超大元数据，不能仅看到 512000 thumbnail budget 就认定发超长帧；旧资产 hash 也会回填缓存/持久化。
- 平台上传队列有 8 项边界、旧任务代号和删除退避；metrics 有界事件 ring、异步固定快照覆盖；进度历史 deque 有上限。
- mst27 单区 Python/C++ 输出已有最终碰撞复检，A* 也禁止对角穿角；问题在适配器后续拼接/降级及完整 footprint/约束闭环，不能说全部规划毫无避障。
- RPP 过期激光会停，碰撞停车有锁存和独立清除；问题是确认计数和实际安全预算，不是完全缺碰撞保护。
- Room 当前构建未见 fallbackToDestructiveMigration；不能在没有证据时声称升级必然删数据。

## 验证与后续验收

本轮新增 [规划探针](E:/CODE/C++/Grinder/artifacts/global-audit-20261005/planner_probe.py) / [结果](E:/CODE/C++/Grinder/artifacts/global-audit-20261005/planner_probe.json)：生产 AST A* 对比 Dijkstra、生产直线穿墙、圈次引用语义、生产缓存键遗漏；[协议探针](E:/CODE/C++/Grinder/artifacts/global-audit-20261005/protocol_probe.py) / [结果](E:/CODE/C++/Grinder/artifacts/global-audit-20261005/protocol_probe.json)：版本/标志接受、半帧阻塞、生产队列丢配置。F01/F07、F02、R01 生产方法探针本轮重新通过，证明问题仍存在。

[静态验证](E:/CODE/C++/Grinder/artifacts/global-audit-20261005/static_verification.json)：38 个相关 Python 文件通过 Python 3.8 AST，8 个相关 launch 解析通过；机器人/APP proto 内容 SHA256 一致。此计数是这次脚本范围，不与上次 47/16 简单相加。

上轮同业务基线已有 APP 70 项测试+3 探针的小堆 JVM 通过，以及机器人 27 项离线测试通过，细节/失败在 [verification.json](E:/CODE/C++/Grinder/artifacts/repository-app-audit-20261004/verification.json)。本轮没有不必要地重跑全部既有单测，不能称完整 Gradle/APK/ROS 测试通过。Windows 原生库进程异常、WSL/Gradle 页面文件与内存提交问题属于审计环境限制，未归因为板端泄漏。

尚未验证：RK3588 全链路 CPU/RSS/温度/降频、真实 WLAN、Android 前后台/锁屏/进程回收、完整目标构建、底盘制动/寄存器反馈。建议在离线与仿真先完成下面矩阵，再做受控硬件验收：

| 组 | 核验用例 | 记录 |
| --- | --- | --- |
| 停止 | 定位/规划/写入屏障插急停；运动/磨盘后重复停车；失败→重连 | 安全帧、最终输出、没有重新使能 |
| 规划正确性 | 不连通/窄通道/薄障碍/裁剪/任意角度/空选择/错误选择/重复圈 | 路径全段 footprint、覆盖率、圈次、失败语义 |
| 缓存 | 每项参数变化、原图内容变化、overlay 变化、不变重复查询 | 路径版本、复用/失效、无执行副作用 |
| 规划资源 | 1/10/50 区域，3000² 与小地图，Python/C++，上传/视频并发 | P50/P95、RSS 峰值、展开数、控制 P99 |
| 协议压力 | 慢读、多客户端、超过 pending、重复请求、迟到 ACK、半帧重连 | 丢弃类型、响应身份、超时、总字节/线程预算 |
| 地图定位 | 回环同版本失败重试、容量满、保存中断、地图版本冲突、定位质量下降 | 资产/栅格版本一致、降级状态、正确安全输出 |
| APP | 后台 1s/30s/5min、断网、进程回收、设备重启、遥控锁定、视频恢复 | 可信状态/首图时间、不会重放运动、帧时间/RSS |

P3 建议：把 scheduler 的协议处理、只读查询、规划上下文、执行提交、持久化拆成职责明确的模块；建立端到端 trace_id 和分阶段耗时。现有指标可复用，补 planning preprocess/search/compose、TX lock/queue、实际驱动帧、地图锁、APP decode/READY 的观测，而不是另建一套无限日志。

整改顺序：G01/G02 → G03/G04/G05/G06/G07/G08 → G09/G14/G16/G17/G18 与规划不变量测试 → 高收益效率组 → 长运行/定位/交付闭环。前三批允许交叉实施，但效率改动必须保持安全与路径约束。
