# 证据记录

机器人既有 F04/F05 已修复并暂存，尚无提交；旧审计位于 artifacts/repository-audit-20260928。APP 根目录 E:/CODE/grindingrobot/grindingrobot，初始 git status 干净。README 描述 TCP → SLLink → 全局 Stream → ViewModel/Compose，需核验源码。

- 当前 F01/F07 假传输复现仍成立：停机新增帧 0，轮速 100/100；磨盘停车新增帧 0；斜坡超时留 80/80。F02 启动流程仍无取消代号/锁存。
- 两端 sl_link.proto diff 无差异；新地图协议已有 request_id/snapshot_id/CRC32，不可沿用旧注释声称协议没有关联 ID。
- APP DeviceTaskMapController 有可见性取消/代号和部分缓存恢复；路径补取进入设备 handle_path_plan_request，缓存命中也设置 state=READY，需验证运行任务副作用。
- APP TcpManager 连接/断线只 clearMapTransfer，不 reset 帧解析器和路径分片；TaskPathChunkAssembler 按 request_id 保存未完成字节，无 TTL/容量上限。
- APP RemoteSettingsViewModel 200ms 发令循环、30s 地图拉取无生命周期 gate；摇杆有边缘 1s 锁定，onDispose 才清零，Activity 后台未必销毁 Composition。
- 首轮 Gradle 测试 daemon 原生内存分配失败，已改为单 worker、768MiB 堆和进程内 Kotlin 重试；WSL 启动 E_ABORT，改 Windows Python，复现成功。
- 旧 F10 路径已迁移至 catkin_ws/src/mapping/cloud_to_occupancy_grid，初次旧路径搜索失败已纠正。

- 73 个 JVM 单测/探针通过（原 TCP 56 + 新编译设备地图 controller 14 + 审计 3）。当前 TcpService 排队旧会话发送在换输出流后写入新流；MapGate取消未释放、失败加载前台不自动重试均实际方法复现。
- 当前 parser 零长帧带旧 3 字节 payload；旧连接 4096 字节半帧后新合法帧返回 0 帧。路径 assembler 1000 个缺片请求保留 4,096,000 字节，暂无 TTL 上限。
- 当前 monitor 100→0 记 65435 丢包，重复 seq 记 -1；服务器 seq 跨连接共享，不能代表连接真实丢包。
- 最终桌面微基准100×4096：parser 23.151ms，无用 Hex 546.4235ms；仅桌面合成，不推导平板提升比例。
- 机器人 27 项纯测试通过；metrics 替身缺消息常量、live map 缺服务替身、relocalization 缺 rospy。47 Python3.8 AST+16XML通过；双端 proto 完全一致。
- APP 工作树恢复干净，审计产生的崩溃日志移动到 planning/environment-diagnostics 保存。机器人原有 staged F04/F05 和用户删除 Office 临时文档均保留。
