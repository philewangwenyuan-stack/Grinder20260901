# Findings

## 已知高风险基线

- Super-LIO 发布未归一化四元数，TF 可拒绝整条定位链。
- 重定位地图加载返回值未检查，空 target 仍进入配准。
- 重定位 ICP 只看 `hasConverged()`，没有质量门控。
- 点云去畸变有尾迭代器越界和零时间间隔风险。
- ESKF 使用 float、直接矩阵求逆且缺少 SO(3)/协方差健康检查。
- 模式管理器只凭初始位姿后收到任意 odom 就判定定位 ready。

## 待补充

- 调度/协议/地图/规划链路风险。
- 底盘安全、导航、构建部署、测试覆盖与运维风险。

## 规模与测试基线

- 关键自研/集成链路约 5 万行以上：scheduler 约 1.76 万行、Super-LIO 约 1.60 万行、SL-LinkA 约 0.84 万行、MST27 约 0.45 万行。
- `scheduler_node.py` 单文件约 1.12 万行，是明显的变更耦合和回归风险中心。
- 现有明确测试主要集中在 scheduler 5 个 Python 测试、chassis 2 个 Python 测试、SL-LinkA embedded 2 个 C 测试；Super-LIO、二维建图、回环、导航 launch 基本无自动化测试。

## 安全与远程控制面

- `catkin_ws/src/grinder_scheduler/config/scheduler.yaml` 保存了疑似生产平台和 MQTT 明文凭据，并使用明文 HTTP/MQTT；存在凭据泄漏、重放和命令篡改风险。
- SL-LinkA 配置监听 `0.0.0.0`；已检查的服务端接收/分发路径未看到认证会话门禁，却可分发任务、运动、地图和控制帧。CRC 只能发现传输错误，不能证明发送者身份。
- SL-LinkA 已有有界 control/map/bulk 工作队列，这是可靠性基础；但周期上报异常被吞掉，应改为可计数、限频告警。

## 底盘安全链路

- 底盘代码具备启动零输出、命令超时停机、急停、通信异常停机和 shutdown 双次停机等基础保护。
- 生产 YAML 将 `command_timeout` 配为 5.0 秒（代码默认 0.5 秒），并将 `enable_safe_stop_on_comm_error` 设为 false；这会显著放大上游失联或串口故障后的持续运动窗口。
- scheduler 的 `_safe_stop_motion()` 只发布零轮速并撤销使能，没有端到端回读确认、超时升级或独立硬件安全通道。ROS/进程/串口故障可能使“已请求停机”不等于“执行器已停机”。
- 原始 RS485 日志在生产 YAML 默认开启，需轮转/容量上限，否则长期运行会占满磁盘并影响实时性。

## 状态持久化

- scheduler 多个 JSON 状态文件使用临时文件加 `os.replace()`，具备单文件原子替换基础。
- 写文件没有显式 flush/fsync，多个 registry/overlay/state 文件也不是一个事务；掉电时可能出现跨文件版本不一致。加载失败通常只告警并继续，缺少 last-known-good、校验和和恢复状态上报。

## 规划、地图与并发

- MST27 已提供 C++ 后端、Python 回退及最终碰撞复检，是现有优点；但 scheduler 同步调用规划，没有任务级截止时间、取消令牌或地图尺寸/区域点数资源预算。异常或超大请求可能长期占用协议工作线程和大量内存。
- 规划会生成多份全图数组（栅格转换、原图副本、膨胀图、各阶段 planning grid、C++ contiguous copy）；大地图/多区域时峰值内存按 `地图格数 × 副本数` 增长。
- MapService 在同一互斥锁内完成整图组合、旋转、绘制和预览编码；慢预览会阻塞地图更新与编辑。每次 compose 又从 ROS 列表重建 NumPy 整图并复制 overlay。
- 二维增量建图本体内存是有界固定栅格，150m×150m@0.05m 约 900 万格：核心 log-odds+observed 约 45MB；发布快照/回环重投影会再产生一到多份整图副本。其主要优化点是减少整图扫描/复制和发布重复构造，而不是更换为 gmapping。
- PGM/YAML 各自使用临时文件，但保存时先删除正式文件再 rename，且两文件不是原子资产包；断电可能缺文件或版本不匹配。
- Super-LIO 三维建图采用 `*point_map_ += *world_pc_` 持续累计；当前 MID-360 配置 `save_interval: -1`，只做运动关键帧准入但没有总点数/空间块上限。
- 每次接受关键帧后 `get3DCloud()` 又执行 `*cloud_map_ = *point_map_`，持续复制完整累计地图；导出时再创建 voxel 结果，峰值会同时存在多份全图。这是 200㎡接近 5GB 和随地图扩大而越来越卡的主要代码级原因。
- 回环工业后端已有 pending 队列上限、每关键帧点数上限和 `max_keyframes=2000`，比主 LIO 地图更有界；但达到上限后直接 `CAPACITY_REACHED` 停止接收新关键帧，长任务需要滑窗/磁盘分块或明确的任务容量告警，而不是静默失去后段回环能力。

## 构建、部署与可维护性

- `build_grinder_platform.sh` 未默认传 `CMAKE_BUILD_TYPE=Release`；`full` profile 也不包含 Super-LIO/loop/2D mapper，实际“全量”构建可能漏掉定位建图链。
- g2o 构建失败被 `|| true` 吞掉，容易形成“构建成功但运行降级/缺功能”的假成功。
- 未发现仓库级 CI workflow；定位、建图、回环、导航 launch 基本无自动测试。
- 启动脚本只做进程存活和 SL-Link TCP 端口检查，缺少雷达数据、TF 连通性、定位质量、地图、move_base、底盘回读等功能就绪门槛；主进程后续死亡也缺少统一监督/重启策略。
- scheduler 单文件约 1.12 万行、至少 127 处广义 `except Exception`，协议、状态机、地图、规划、媒体、平台上传和设备控制高度耦合，故障隔离及回归定位成本高。
- `_set_chassis_enabled()` 已变成 no-op，但 `chassis_power` 协议仍返回 `RESULT_SUCCESS`，形成接口语义与真实执行不一致；在控制/安全接口上不可接受。
