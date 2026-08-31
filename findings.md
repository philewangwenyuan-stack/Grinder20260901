# Findings: X920 Gazebo 全链路仿真

## Existing environment

- WSL2 `Ubuntu 20.04.3 LTS`，ROS Noetic、Gazebo 11.15.1、`gazebo_ros`、`stage_ros` 和 WSLg 已就绪。
- 现有仓库只有 Stage 差速仿真资源，没有 URDF/Xacro/SDF Gazebo 机器人模型。
- 调度器监听 `0.0.0.0:8002`，APP 协议可保持不变。

## Existing control/data interfaces

- 自动导航：`move_base -> /cmd_vel`；调度器通过 `/chassis/task_enable` 门控。
- 手动/设备控制：`/chassis/wheel_speed_cmd`、`disc_speed_cmd`、`disc_enable_cmd`、`work_mode_cmd`、`disc_lift_cmd`、`light_cmd`、`emergency_stop`。
- 反馈：`/chassis/status`、`/chassis/wheel_speed_state`、`/odom_wheel`。
- Aurora 输入：配置化的 map、odom、左右图像、深度图话题；地图模式/同步/清理/重定位使用 slamware 自定义消息与服务。
- 仿真必须使用独立配置禁用真实平台同步和 MQTT。
- 仿真覆盖项应至少设 `file_upload_on_map_save: false`、`mqtt_enabled: false`、`stream_enabled: false`；本地 RTSP 可独立保留。`PlatformFileSync` 的真正开关就是 `file_upload_on_map_save`。

## Geometry/config facts

- 2026-08-25 用户最终布局要求：不保留任何独立万向轮；前置直径 `0.90m` 磨盘承担唯一全向支撑，车辆前向定义为 `+X`，磨盘正旋转轴定义为 `+Z`。实现采用磨盘 `+Z` 转向连续关节和隐藏球形支撑 `+Y` 滚动连续关节，接触 `mu1=mu2=0`；驱动轮继续保持牵引摩擦。

- 2026-08-25 方向/站位修正：方盒车体后缘为 `x=-0.60m`；轮子与雷达中心改为 `x=0.00m`，满足距后缘 `0.60m`。协议文档明确摇杆上推为 `remote_y=-1`；因此仿真桥保留实车 `drive_left/right_sign=-1`，Gazebo 轮轴恢复 `+Y`，自动 `/cmd_vel` 不额外取反，避免 APP 上推被重复翻转。

- 方向/姿态验证：APP 摇杆上推对应负轮速；最新独立测试中车体 `x` 从约 `-0.165m` 到 `+0.070m`，明确朝磨盘 `+X` 前方。自动 `/cmd_vel +0.15` 也沿 `+X` 移动；停止后 `base_footprint` pitch 约 `-0.003rad`，未保持翘头。

- 2026-08-25 质量修正：用户确认整车约 `800kg` 且质量中心在磨盘。Xacro 现在把车体、磨盘、轮子、支撑球、雷达和相机合计配平到 `800kg`，并将综合 x COM 设为 `0.55m`；主车体惯性 origin x 由公式得到约 `0.568m`。
- 2026-08-25 翘头修正：整车综合 z COM 也配平到磨盘盘面 `z=0.04m`；隐藏球形支撑仍作为磨盘万向组件，但改由 `base_link` 承载并相对盘心前置 `0.15m`，避免磨盘旋转导致支撑点绕到质心后方。独立实测静止 pitch `0.00027rad`，自动前进约 `0.497m` 后 pitch `-0.00037rad`，APP 上推等效轮速再前进约 `1.21m` 后 pitch `-0.00020rad`。
- 质量矩公式已纳入前置支撑偏移、驱动轮轴高、雷达/相机安装高；Xacro 展开严格断言整车质量 `800.000kg`、COM=`(0.550, 0.040)m`。修正后完整导航启动再次通过：`move_base`、调度器、仿真底盘、slamware 桥存活，地图 `1000x1000@0.05m`，静止 pitch `0.00026rad`。
- 按用户要求缩短车体后部 `0.30m`：后缘由 `x=-0.60m` 收到 `x=-0.30m`，前缘保持 `x=1.00m`；同步更新仿真盒体、scheduler 尺寸和 global/local costmap footprint，800kg 质量矩与磨盘重心保持不变。
- 缩短后动态验证：自动 `/cmd_vel=+0.15m/s` 前进约 `0.46m` 时 pitch 约 `-0.00005rad`，APP 等效负轮速继续沿 `+X` 前进约 `1.17m`，未出现翘头；完整导航加载新 footprint 和 50m 地图成功。

- STEP 文件存在，大小 `70,719,045` bytes，AP203，由 SolidWorks 2018 导出；坐标量级为数百，按机械 CAD 常规判断单位是 mm，仍需由 CAD 内核读取边界确认。
- STEP 是总装而非现成三角网格；源文件只作为输入，不直接让 Gazebo 加载。
- 底盘配置：轮距 `0.70 m`、轮半径 `0.1475 m`、减速比 `60.0`。
- 调度规划尺寸：车宽 `1.0 m`、车长 `1.5 m`；应作为碰撞 footprint 的初始依据，并用 STEP 边界复核。
- 当前 Windows/WSL 均未发现 FreeCAD、Assimp、MeshLab、Blender、Gmsh 命令；需安装或使用其它 OpenCascade 转换方式。
- 第一次 WSL Python 模块探测因嵌套引号导致 `SyntaxError`，未得到模块结论；下一次使用无歧义脚本重试，不重复原命令。
- 重试确认 WSL 没有 FreeCAD/OCP/OCC/cadquery/trimesh/meshio Python 模块；Ubuntu 20.04 apt 可安装 `freecad-python3 0.18.4`。
- WSL 根分区约 944 GiB 可用，且 `sudo -n` 可用；安装 CAD 转换依赖不会受空间或交互式密码阻塞。
- 已在 WSL 安装 `freecad-python3 0.18.4` 及 OpenCascade 7.3 运行库，约占 255 MB；仅用于开发期 STEP 转换。
- FreeCAD headless 入口为 `/usr/bin/freecadcmd`，版本 0.18.4；`FreeCAD`、`Part`、`Mesh` 模块可由该解释器加载。
- OpenCascade 精确读取结果：CAD 原生单位 mm，边界 `1962.325 x 1932.746 x 900.000 mm`；Z 轴明确为竖直方向，最低/最高 Z 为 `373/1273 mm`。
- CAD 包围盒中心 `(158.889, 487.058, 823.000) mm`；306 个 solids、14,549 faces。完整总装约 2 m x 1.93 m，明显大于调度中 1.0 x 1.5 m 的旧 footprint，不能用调度尺寸裁剪视觉模型。
- 实体总体积约 `0.092998 m^3`，但未包含材料密度，不能据此可靠推导整机质量；仿真质量保持可配置默认值。
- 导入与精确拓扑统计约 98 秒；后续不重复解析做普通审计，只在 mesh 生成或源模型变化时运行。
- 已用 OpenCascade/FreeCAD 以 2.0 mm 线性偏差导出 `meshes/x920_frame.stl`；视觉 mesh 保持 STEP 原坐标和毫米单位，Xacro 负责 `0.001` 缩放与中心/落地变换。
- 导出 STL 为 41.91 MB、838,198 facets、416,287 points。只适合作为 visual，不可用作 collision；初版保留高精度，若 Gazebo 实测帧率不足再做二次 decimation。
- STEP 产品树包含 306 个实体和复用的 X850/标准轴承/紧固件等子装配名称，说明这是细节完整的机械总装，视觉复杂度不是单一外壳造成的。
- 机器人坐标初始约定：CAD Z 保持向上，CAD X 暂作机器人前向；mesh 按包围盒中心对齐 X/Y，按最低 Z 落地。通过 Gazebo 截图验证后再决定是否绕 Z 旋转 90°。
- WSL 中确认存在 `libgazebo_ros_diff_drive.so`、`libgazebo_ros_laser.so`、`libgazebo_ros_camera.so`。
- Xacro 在显式加入新包的 `ROS_PACKAGE_PATH` 后可展开出完整 link/joint/plugin 树；系统未安装 `check_urdf`，后续用 XML/robot_description/Gazebo spawn 多层验证。

## Simulation frame/topic constraints

- 导航使用 `map`、`base_link`、`/scan`；TEB 读取 `/odom`，RPP 当前读取 `/odom_wheel`。
- Gazebo 驱动应发布 `odom -> base_footprint`，机器人模型提供 `base_footprint -> base_link` 与传感器 TF。
- slamware 模拟层需要兼容 ClearMap/SyncMap/SetMapUpdate/SetMapLocalization 消息，以及 `SyncGetStcm`、`SyncSetStcm`、`RelocalizationRequest` 服务。
- `grinder_system.launch` 固定包含真实 slamware 驱动，因此仿真应新增独立 launch，而不是给现有 launch 堆叠大量条件。
- 调度器已经允许 map/odom/左右图像通过 launch 参数重映射，但深度图话题当前未由 `scheduler.launch` 暴露；仿真可直接发布到其默认 slamware 命名空间。
- `AuroraBridge` 在 `use_depth_colorized_image=true` 时不会订阅左右相机；为了让 APP 的双相机分块与 RTSP 都可用，仿真覆盖必须设为 `false`。
- 仓库实际包含可执行的 `tools/mediamtx/mediamtx` x86-64 二进制（另有 aarch64 版本），因此 WSL 仿真可保留本地 RTSP 8554。
- 仿真包可直接依赖现有 `grinder_chassis_driver` 与 `slamware_ros_sdk` 生成的消息/服务，避免复制接口定义。
- 调度器对 `SyncGetStcm` 的硬要求是服务成功且目标文件非空；仿真桥可写带明确 `GRINDER_SIM_STCM_V1` 标识的占位文件，从而覆盖 APP 地图保存/目录/切换业务，但不会冒充真实 Aurora STCM。
- 调度器只把雷达系统状态当作不透明字符串；重定位聚合状态要求精确值 `RelocalizationRunning/Succeed/Failed/Canceled`。
- 隔离 devel 环境中，不能再把 `grinder_chassis_driver/src` 放到生成消息路径之前；其普通 Python 包会遮蔽同名的 Catkin `msg/srv` 生成包。正常全量 Catkin 工作空间会把两者合并到 devel 空间，该问题仅由本次手工隔离冒烟环境的路径顺序引起。
- 当前工作区的 `catkin_ws/build/CMakeCache.txt` 记录 `CMAKE_HOME_DIRECTORY=/home/neardi/work/Grinder/catkin_ws/src`，`devel/.catkin`也记录同一旧源路径；在 WSL 当前路径 `/mnt/e/CODE/C++/Grinder/catkin_ws` 下不可复用。
- 用户已进入 WSL shell，因此不应在 Ubuntu 内再执行 Windows 命令 `wsl -d Ubuntu-20.04`；直接 `cd` 到工作空间即可。
