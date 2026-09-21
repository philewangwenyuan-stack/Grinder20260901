# Findings

- Android 当前 `RobotSettingsScreen.kt` / `RobotSettingsViewModel.kt` 只有 `vehicle_width` / `vehicle_length`，协议 `MapSettings` 没有 footprint 顶点、base-to-laser TF 或 RPP 字段。
- 板端 RPP 已注册 `dynamic_reconfigure::Server<RegulatedPurePursuitConfig>`；scheduler 当前只动态修改 `desired_linear_vel`。
- footprint 来自 costmap YAML，`base_link -> base_laser_link` 来自启动参数；两者需要板端运行时服务或明确的“保存后重启”接口。
- 当前地图删除链路疑似为：停止重定位 → 远端删除 → 本地删除；远端超时会让本地删除和 registry 删除不执行。
- 当前仓库的 Gazebo URDF 将 `drive_axle_x=0.00`、轮距 `0.70 m`，因此 `base_link` 的运动中心应在两驱动轮之间；URDF 注释和仿真覆盖使用的简化 footprint 是 `x=[-0.30,1.00]`、`y=±0.47`，而导航 diff-drive 配置存在 `x=[-0.60,1.00]`、`y=±0.47` 与 TEB 的 `x=[-0.60,1.25]`、`y=±0.60` 两套值，后续必须避免混写。
- 相关文件：`catkin_ws/src/grinder_gazebo/urdf/x920_grinder.urdf.xacro`、`catkin_ws/src/grinder_gazebo/config/navigation_sim_overrides.yaml`、`catkin_ws/src/2-dnavigation-package/2dnavigation/teb_local_planner_tutorials/cfg/diff_drive/costmap_common_params.yaml`、同目录 `teb_local_planner_params.yaml`。
- `PlatformFileSync` 已有后台上传 worker 和 `_operation_lock`，但 `delete_map()` 是同步调用；可复用同一 worker 增加可持久化的远端删除待办，不让 `MapDeleteRequest` 等待 HTTP。
- `handle_map_delete_request()` 当前在 `shutil.rmtree()` 和 registry 清理前同步调用 `platform_file_sync.delete_map()`，这正是平台超时导致本地删除失败的顺序问题。
- `MapDeleteResponse` 目前只有 `deleted`，需要增加本地删除、远端已删除、远端待重试三个状态，兼容旧 APP 时仍保持 `deleted=true/result=SUCCESS` 表示本地删除成功。
- `base_link -> base_laser_link` 的静态 TF 由 launch 参数生成，不能仅靠 scheduler 的参数服务器写入实现立即变更；本轮协议可先返回“已保存/需重启”语义，RPP 则可通过 dynamic_reconfigure 临时应用。
- 已完成板端协议和 handler：RPP 核心速度、前视、曲率、避障、转向和到达参数可读写；临时应用走 dynamic_reconfigure，保存默认同步 RPP YAML。
- 已完成 geometry handler：footprint 动态更新两个 costmap 并写入 costmap YAML；base-to-laser 写入 Super-LIO mode manager 的下一次重载参数，并在响应中标记需重启/重载。
- 已完成地图删除顺序修复：本地 bundle、registry 和关联状态先清理；远端删除进入持久化队列并指数退避重试。
- 2026-09-20 重定位与路径规划页面：
  - 重定位弹窗的 pose 状态不能把实时 `robotPose` 放进 `remember` key，否则状态轮询会把用户手动调整的位置拉回板端位置；现改为弹窗打开/地图切换时取一次初始值。
  - 重定位标记按参考图收敛为小绿点和细箭头，不再绘制外圈。
  - 路径规划参数通过 `MapSettings` 字段 6、16～21 传输；新增字段使用 optional，旧版只写机器人外形时不会覆盖规划参数。
  - Android 设置页新增“路径规划”菜单和三组参数卡片；默认值为 `0.6/2.0/0.3/0.7/3.0/45.0/0.0`。
