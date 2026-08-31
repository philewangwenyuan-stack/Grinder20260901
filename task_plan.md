# Task Plan: X920 Gazebo 全链路仿真

## Goal

基于用户提供的 X920 STEP 总装模型和现有底盘参数，新增可在 WSL/ROS Noetic/Gazebo 11 中运行的研磨机模型、虚拟底盘接口、传感器桥接与一键启动入口，使现有 APP 能通过原 SL-LinkA 协议操作仿真机器人。

## Phases

| Phase | Status | Deliverable |
| --- | --- | --- |
| 1. 审计 CAD、尺寸、ROS 接口与本机工具 | complete | 明确几何单位/边界、轮距轮径、所需话题服务、转换路径 |
| 2. 创建 Gazebo 包和 X920 模型 | complete | Catkin 包、mesh、Xacro、world、传感器与差速插件 |
| 3. 实现仿真硬件桥 | complete | 底盘控制/反馈、安全门控、slamware 兼容话题与服务 |
| 4. 集成调度、导航和 APP 启动链路 | complete | 仿真 YAML、launch、网络/运行说明 |
| 5. 构建、测试与视觉验证 | complete | Catkin 构建、自动化测试、Gazebo/RViz 冒烟测试、修复结果 |
| 6. 修复跨机器 Catkin 缓存并简化仿真构建 | complete | 路径错配自动备份、`PROFILE=sim`、启动前校验与 WSL 构建/运行实测均通过 |
| 7. 简化车体并扩展 50m 柱网环境 | complete | 代价地图尺寸对应的方盒车、0.9m 研磨盘、50m×50m 单层/6m 柱网与独立端口运行验证 |
| 8. 对齐后驱/雷达并处理接触摩擦 | complete | 后驱轮与雷达同站、前万向轮移除、前置视觉磨盘/后支撑轮无拖拽、驱动轮保留牵引，运动回归通过 |
| 9. 以磨盘万向支撑替代独立万向轮 | complete | 删除后支撑轮，磨盘采用 +Z 正向旋转轴和二自由度无摩擦球形支撑，完整导航回归通过 |
| 10. 对齐实车驱动方向与后轴位置 | complete | 驱动轮/雷达中心距后缘 0.60m，轮轴/仿真桥方向与实车控制一致，手动与自动运动回归通过 |
| 11. 校准 800kg 质心与防翘头接触 | complete | 总质量/三维质心按磨盘配平，磨盘万向支撑前置 0.15m，静止与加速 pitch 回归通过 |
| 12. 缩短车体后部并同步仿真 footprint | complete | 后缘由 x=-0.60m 收到 x=-0.30m，前缘/磨盘/后驱雷达站位保持不变；质量矩、动态方向和完整导航回归通过 |

## Decisions

- 保留现有 APP 和 `grinder_scheduler` 协议，不另做仿真客户端。
- Gazebo 运动入口使用独立仿真命令话题，由桥接节点统一处理自动 `/cmd_vel`、手动轮速、急停与 `task_enable`，防止绕过安全门控。
- STEP 主要作为视觉 mesh；碰撞体和惯量使用简化几何，确保仿真稳定与实时性。
- 仿真配置默认禁用真实 MQTT/平台上传，避免污染生产系统。
- 兼容现有 ROS1 Noetic/Gazebo 11，不在本任务中迁移 ROS2。
- 车体视觉默认使用简化方盒，不再依赖 STEP 网格；保留 `visual_mesh:=true` 作为可选 CAD 检视开关。
- 方盒尺寸严格对应仿真 diff-drive costmap footprint：x=`[-0.30, 1.00]`、y=`[-0.47, 0.47]`；运动中心为 `base_link`，车体几何中心前移 `0.35m`。真实导航 footprint 保持不改。
- 研磨盘用直径 `0.90m` 的圆柱体表达，前缘与 footprint 的 x=`1.00m` 对齐。
- Gazebo 世界固定为 50m×50m 单层平面，边界墙高 3m，柱子截面 0.9m×0.9m、柱心间距 6m，柱心坐标为 `-21,-15,-9,-3,3,9,15,21`。
- 车体后缘为 `x=-0.30m`；两个驱动轮与雷达纵向位置统一为 `x=0.00m`，即距后缘 `0.30m`；删除所有独立万向轮，由前置磨盘组件承担唯一全向支撑。
- 仿真桥复用实车手动轮速方向参数 `drive_left_sign=-1`、`drive_right_sign=-1`，Gazebo 轮关节轴保持车体坐标 `+Y`；APP 摇杆上推（`remote_y=-1`）和自动 `/cmd_vel` 正向均朝磨盘 `+X` 前方。
- 磨盘可绕 `+Z` 轴正向旋转，隐藏球形支撑通过 `+Z` 转向和 `+Y` 滚动两个连续关节实现无摩擦全向接触；驱动轮保留 `mu1=mu2=1.0` 牵引，确保 APP/差速驱动可移动。
- 整车物理质量按用户实车参数设为 `800kg`（含附件），综合质量中心设在磨盘中心（`x=0.55m`、盘面高度 `z=0.04m`）；主车体惯性质量自动扣除轮、磨盘和传感器。
- 隐藏球形万向支撑仍属于磨盘组件，但固定在 `base_link` 参考系并相对盘心前置 `0.15m`，避免盘体旋转把支撑点绕到质心后方；球体标称零摩擦，驱动轮继续承担牵引。

## Errors Encountered

| Error | Attempt | Resolution |
| --- | --- | --- |
| WSL Python CAD 模块探测脚本出现嵌套引号 `SyntaxError` | 1 | 已获取其它审计结果；改用简单 `importlib.util.find_spec` 命令重试 |
| `apt-get update` 初始无法解析 Ubuntu/ROS 域名 | 1 | apt 随后恢复解析并使用可用索引完成 FreeCAD 安装；无需修改 DNS |
| `freecadcmd script.py ... --report` 把脚本参数当成 FreeCAD 自身参数 | 1 | FreeCAD 0.18 CLI 不透传 GNU 选项；改用环境变量驱动转换脚本，避免脆弱的命令行嵌套转义 |
| FreeCAD `MeshPart` 导入缺少 `libnglib.so`，且 CLI 错误退出码仍为 0 | 1 | 报告阶段无需 MeshPart；导出改用 `Part.Shape.tessellate` + `Mesh.Mesh`，移除 Netgen 依赖，并以输出报告存在性作为成功条件 |
| FreeCAD 0.18 的顶层 `Part.Shape` 没有 `CenterOfMass` 属性 | 1 | 对所有 solids 按体积加权计算质心；无实体时回退到包围盒中心 |
| 首次独立运行 xacro 时 `grinder_gazebo` 尚未进入 ROS package path | 1 | 构建前验证显式设置 `ROS_PACKAGE_PATH=$PWD/src`；正式使用由 Catkin devel setup 提供 |
| 首次布局断言 grep 参数误用导致命令显示 Usage | 1 | 改用 `grep --` 重跑，雷达/轮轴/前万向轮删除断言通过 |
| 驱动轮接触摩擦设为 0 后运动回归无位移 | 1 | 仅恢复左右驱动轮 `mu1=mu2=1.0`；磨盘和后支撑轮继续保持零摩擦 |
| 研磨盘虽声明零摩擦仍因固定平面碰撞体阻滞差速车 | 1 | 将研磨盘保留为视觉体并移除其碰撞几何；APP/导航运动回归恢复 |
| 首次最终 Xacro 断言脚本对无 name 的碰撞节点直接取键 | 1 | 改用 `attrib.get` 重跑，布局/无磨盘碰撞断言通过 |
| 尝试把 URDF xacro 管道直接交给 `gz sdf -k -` | 1 | Gazebo CLI 不接收该 stdin 形式；改用 XML 解析验证 Xacro、单独对 world SDF 执行 `gz sdf -k` |
| 固定前置球形支撑仍使差速车只能前进约 0.03m | 1 | 将磨盘支撑拆为 `+Z` 转向关节 + `+Y` 滚动关节的二自由度球形接触，运动恢复到约 0.84m/6s |
| 停止测试实例时 slamware 回调晚于 ROS publisher 关闭 | 1 | 仅发生在 Ctrl-C 关停阶段，不影响运行；重新启动/验证无残留进程 |
| 最终 README 检索把 `+X/+Z` 当正则表达式导致 `rg` parse error | 1 | 改用 `rg -F` 字面检索，文档方向说明已确认 |
| WSL 未安装 `check_urdf` | 1 | 不额外引入依赖；改用 XML 解析、xacro 展开、robot_description 和 Gazebo spawn 验证 |
| 隔离冒烟的手工 `PYTHONPATH` 遮蔽 Catkin 生成消息 | 1 | 将 scheduler 加入隔离 Catkin 构建，使用 devel 合并包，不再手工前置同名源码包 |
| move_base 因真机 STVL 插件未构建而重启 | 1 | 新增仿真专用 costmap 覆盖，使用标准 `costmap_2d::ObstacleLayer`，不改真机配置 |
| MediaMTX 可用但 WSL 缺少 FFmpeg，无 RTSP 编码器 | 1 | 安装 Ubuntu 官方 `ffmpeg`，启动脚本和 README 增加明确前置提示 |
| PowerShell `Remove-Item` 清理测试 pycache 被执行策略拒绝 | 2 | 改用单一 WSL shell 对已明确核验的精确文件执行非递归 `unlink`/`rmdir`，已清理并复查无残留 |
| 用户直接 `catkin_make` 时读到 `/home/neardi/work/Grinder` 生成的旧 CMakeCache | 1 | 已确认 `build/CMakeCache.txt`和 `devel/.catkin` 均硬编码旧路径；正在将路径错配检测与可恢复备份纳入构建脚本 |
| `CLEAN_ON_PATH_CHANGE=0` 验证命中旧路径并以退出码 1 终止 | 1（预期负测试） | 证明禁用自动备份时不会触碰旧产物；下一步使用默认可恢复备份模式构建 |
| `set -u` 状态下加载 ROS Noetic `setup.bash` 报 `ROS_DISTRO: unbound variable` | 1 | 旧缓存已成功备份；在两个构建函数和仿真启动脚本的 `source setup.bash` 前后临时 `set +u`/恢复 `set -u` |
| 误用 `wait` 轮询普通终端会话，提示 exec cell 不存在 | 1 | 改用 `write_stdin(session_id=...)` 读取长时间构建/启动会话并发送 Ctrl-C |
| 关停检查的 `pgrep -af` 匹配到检查命令自身 | 1 | 改按进程 `comm` 精确匹配并独立检查 8002/8554/11311 端口；确认无残留 |
| 首次生成 50m 世界时西侧边界模型漏写 `</visual>` 导致 XML mismatch | 1 | 用 Gazebo `gz sdf -k` 和 Python XML 解析定位并补齐闭合标签；64 根柱模型通过计数校验 |
| 独立新场景启动初期 scheduler 报 `map_aligned`/`aurora_bridge` 状态恢复警告 | 1 | 该警告发生在仿真桥和 TF 初始化前，随后 scheduler、move_base、RPP、1000×1000 costmap 均正常启动；不影响运行链路 |
| 800kg 质心在盘心时前支撑抬起驱动轮、方盒出现翘头 | 1 | 将球形支撑从旋转盘 link 解耦并前置 0.15m，车体质心同步降至盘面高度；静态 pitch 约 `0.00027rad`，自动/手动加速后仍小于 `0.0004rad` |
