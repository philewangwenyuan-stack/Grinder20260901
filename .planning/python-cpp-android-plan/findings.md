# Findings

## 已知设备端基线

- SL-LinkA 当前接收调用已经是 32KB，并有有界队列和单发送锁；原方案的 4096B 接收缓冲判断已过时。
- 地图路径需要重新核对各响应的 chunk 上限，不能把某一历史 512B 限制泛化到全部地图消息。
- 三维累计地图完整复制、未认证控制面、定位质量门控和底盘停机闭环比纯 Python CPU 更高优先级。

## 待核对

- 安卓 SlFrameParser、TcpManager、SlLinkManager 的线程、缓存、重组、校验和超时行为。
- 设备端地图与大响应具体分片上限、批量发送、背压和取消实现。
- Super-LIO mode manager 的 operation 状态、启动就绪和停止失败语义。

## 2026-09-21 Android 初步核对
- Android TcpService 当前接收缓冲为 4096B，每次读取会 copyOf；解析器支持 TCP 分片/粘包，因此这是性能项，不是正确性缺陷。
- TcpService 普通发送、异步发送、心跳分别启动协程并直接 write+flush 同一 OutputStream，缺少统一单写者队列；地图传输并发时存在帧交错、顺序不确定和过度 flush 风险。
- 设备端 sl_linka_adapter 已使用 recv(32*1024)、send_lock、控制/批量并发限制和有界队列，因此原方案中“设备端 4096B 接收缓冲”“没有基本队列隔离”等描述需要更新。
- 报告应把 Kotlin 单写者 Channel、控制/事务/大数据优先级、流式地图落盘与校验放在 Python/C++ 改造之前。

## 2026-09-21 代码核对结论
- 设备端 MapRequest 默认/上限均为 4096B，原方案“当前被压回512B”已过时；16KB需协议兼容压测后再开放。
- 设备端 path bulk TX 已将完整协议帧合并为约128KiB写入，但 MapRequest 仍在 dispatch 中逐片同步发送；build_map_chunks 还先创建完整 outputs 列表，造成编码数据+所有分片对象并存。
- 设备端地图请求每次从 OccupancyGrid 复制为 NumPy、生成三通道图、flip/resize、PNG编码；没有按 map_id/map_version/参数缓存。
- Android TCP 普通发送、suspend发送、心跳分别直接 write+flush；必须改成单写者有界优先级队列。TcpManager 还会把完整帧逐字节转十六进制字符串用于成功日志，应生产禁用或采样。
- Android地图组包只以 uint32 map_id 为键；MapChunk没有 transfer_id/total_bytes/whole checksum。组包没有超时、总大小/分片数上限，接收完使用 ByteArrayOutputStream 再复制成最终 ByteArray，内存峰值高。
- Android SlFrameParser逐字节状态机可正确处理分片/粘包，但CRC时再次拼接header+payload，完成帧又copy payload；可先用Kotlin减少复制，只有基准证明它占CPU热点才考虑JNI/C++。
- Android测试覆盖若干协议builder/parser调用及路径分片，但未看到TCP单写者、地图乱序/重复/缺片/超时/恶意长度、断线重连的专门测试。
- 当前 mid360_mapping.launch 包含的是 Super-LIO节点launch，不是Livox驱动；base脚本才启动livox_ros_driver2。因此“Livox必然重复启动”不符合当前代码。仍需把驱动所有权写成唯一入口并用启动检查防回归。
- Super-LIO停止逻辑已有残留mapping节点kill兜底，但kill后未再次确认退出/返回TIMEOUT；定位READY仍是收到initialpose后任意/lio/odom即可，缺少时间戳、地图版本、配准质量和连续稳定帧门槛。

## 最终结论
- 原方案约70%方向成立，但设备端32KB接收、4KB地图分片、有界队列等已落地，不能重复实施。
- 第一优先级：Android单写者、地图transfer事务/流式组包、生命周期无假成功、安全与可观测性。
- C++只在路线一后经三轮基准确认帧处理持续占单核10%–15%以上时进入；不整体改写scheduler，不先引入Android JNI。
- 已生成并逐页QA 10页Word方案，路径 artifacts/Grinder设备端与安卓端Python与C++加速实施方案.docx。
