# Progress

- 2026-09-19: 创建本次独立规划，确认范围包含机器人设置/RPP 端到端链路和当前地图删除顺序修复。
- 2026-09-19: 已读取仓库级导航说明、旧 LIVE_MAP 修复规划和 planning-with-files 规则。
- 2026-09-19: 核对 Gazebo URDF：驱动轮轴为 `base_link` 的 x=0、轮距 0.70 m；发现仿真、costmap、TEB 存在不同 footprint 数值，需在实现前选择单一来源。
- 2026-09-19: 确认 `MapDeleteRequest` 当前同步等待 `PlatformFileSync.delete_map()`，远端删除成功后才执行 `shutil.rmtree()`、registry 和本地状态清理；决定改为本地优先、远端异步可重试。
- 2026-09-19: 确认 RPP 可走 dynamic_reconfigure；base-to-laser 仍由 static TF launch 生成，因此板端设置接口需要明确“保存后重启/下次模式切换生效”而不是伪装成即时 TF 更新。
- 2026-09-19: 扩展 SL-Link 源协议：MapSettings 增加 footprint/base-laser 字段，新增 RppSettings 和临时应用/保存标志，MapDeleteResponse 增加本地/远端/待重试状态。
- 2026-09-19: PlatformFileSync 增加可持久化远端地图删除待办、指数退避和后台 worker 处理，并已接入 scheduler 的本地优先删除顺序。
- 2026-09-19: 生成 embedded/Python/Android SL-Link SDK；增加 map delete 回归测试，目标测试通过；完整 discovery 仍受现有 ROS 测试 stub 互相污染影响。
- 2026-09-19: Python 语法检查、协议序列化检查、`test_live_map_save`（6 tests）和 `test_platform_integration`（1 test）通过；未运行真实 ROS、move_base、dynamic_reconfigure 或硬件联调。
- 2026-09-20: 修复 Android 重定位编辑态：实时板端 pose 轮询不再覆盖用户正在调整的本地点位；标记改为小绿点加细箭头并移除外圈。
- 2026-09-20: SL-Link `MapSettings` 增加六个路径规划可选字段，重新生成 Python/Android/embedded SDK；nanopb 生成和 Python protobuf 往返检查通过。
- 2026-09-20: Android 设置页新增“路径规划”页面，提供七个核心参数的读取、校验和写入；导航页保留核心卡片并去掉高级参数提示。
- 2026-09-20: scheduler、TaskConfigModel、planner_adapter 与 MST27 默认值已接入七个路径规划参数；Python 静态编译检查通过，未执行 Android/ROS 编译。
