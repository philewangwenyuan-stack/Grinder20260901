# X920 Gazebo 仿真

该包默认使用与 X920 尺寸相近的简化方盒车体和研磨盘，并保持现有 Grinder APP、调度、导航和底盘 ROS 接口不变。STEP 网格仍可按需打开。

## 模型参数

- 默认视觉模型：`1.30 x 0.94 x 0.30 m` 方盒体，运动中心为 `base_link`，车体中心相对运动中心前移 `0.35 m`；后部较上一版缩短 `0.30 m`，前缘保持不变。
- 物理质量：整车总质量 `800 kg`（含磨盘、驱动轮、支撑球、雷达和相机）；质量加权重心位于磨盘中心（`x=0.55m`、盘面高度 `z=0.04m`），用于避免加速时翘头。
- 研磨盘：直径 `0.90 m`，中心 `x=0.55 m`，前缘与代价地图 footprint 的 `x=1.00 m` 对齐。
- 两个驱动轮和雷达中心位于 `x=0.00m`，即相对车体后缘 `x=-0.30m` 向前 `0.30m`；删除全部独立万向轮，前置磨盘作为唯一万向支撑。
- 仿真桥复用实车手动轮速方向参数 `drive_left_sign=-1`、`drive_right_sign=-1`；Gazebo 轮关节轴保持车体坐标 `+Y`，自动 `/cmd_vel` 不再额外翻转。APP 摇杆上推（协议 `remote_y=-1`）和导航正向均朝磨盘 `+X` 前方。
- 磨盘视觉圆盘朝车体 `+X` 前方，旋转正轴为 `+Z`；其下方使用隐藏球形接触体实现无摩擦全向支撑。支撑固定在 `base_link` 参考系并相对盘心前置 `0.15m`，避免磨盘旋转时支撑点绕到质心后方而抬头；水平盘面本身不参与碰撞。
- 驱动轮保留 `mu1=mu2=1.0` 的必要牵引力，否则差速驱动无法移动。
- STEP 边界：`1.9623 x 1.9327 x 0.9000 m`，详见 `meshes/x920_frame_report.json`。
- 驱动轮距：`0.70 m`。
- 驱动轮半径：`0.1475 m`。
- 减速比：`60.0`。
- 简化物理碰撞体：`1.30 x 0.94 x 0.30 m`，与仿真代价地图 footprint `[-0.3, 1.0] x [-0.47, 0.47]` 一致；真实导航参数不改。
- 环境：`50m x 50m` 单层平面，边界墙高 `3m`，`0.9m x 0.9m` 柱子按 `6m` 柱心间距布置，共 `8 x 8 = 64` 根。

默认不加载 STEP，Gazebo collision 和 inertia 使用简化几何，以保证姿态稳定和实时性。仿真启动会用 `navigation_sim_overrides.yaml` 将 global/local costmap footprint 对齐到上述方盒尺寸；真实导航参数保持不改。需要查看 CAD 外观时运行 `./start_grinder_sim.sh visual_mesh:=true`。

## WSL 启动

```bash
cd /mnt/e/CODE/C++/Grinder/catkin_ws
source /opt/ros/noetic/setup.bash
sudo apt-get install -y ffmpeg python3-scipy
PROFILE=sim ./build_grinder_platform.sh
./start_grinder_sim.sh
```

如果工作区从其它 Linux 路径拷贝而来，构建脚本会自动识别旧
`CMakeCache.txt`，将 `build/devel/install` 改名为带时间戳的 `.bak.path-mismatch.*`
备份后重建，不会直接删除旧产物。

无 GUI 冒烟测试：

```bash
./start_grinder_sim.sh gui:=false visual_mesh:=false start_navigation:=false local_rtsp_enabled:=false
```

常用参数：

- `gui:=true|false`：Gazebo 图形界面。
- `visual_mesh:=true|false`：真实 STEP visual 或轻量盒体。
- `start_navigation:=true|false`：启动 move_base + RPP/TEB。
- `navigation_local_planner:=rpp|teb`。
- `local_rtsp_enabled:=true|false`。
- `sl_linka_port:=8002`。

## APP 连接

- Windows 本机 APP：`127.0.0.1:8002`。
- 局域网手机 APP：连接 Windows 电脑局域网 IP 的 `8002` 端口。
- 左右 RTSP：`rtsp://<Windows-IP>:8554/left` 和 `/right`。

RTSP 编码依赖 WSL 内的 `ffmpeg`；启动脚本会在缺失时给出警告。
MST27 完整规划依赖 `python3-scipy`；缺失时调度器会自动退回备用规划器。

Windows 10 + WSL2 NAT 下，以管理员 PowerShell 运行以下脚本设置手机访问转发；WSL 重启、IP 改变后重新执行：

```powershell
& "E:\CODE\C++\Grinder\catkin_ws\src\grinder_gazebo\tools\update_wsl_portproxy.ps1"
```

## 仿真边界

- `SyncGetStcm` 生成带 `GRINDER_SIM_STCM_V1` 标识的仿真占位文件，只用于验证 APP 地图业务；它不是真实 Aurora STCM。
- 深度彩色图在仿真中复用左相机画面，不提供度量深度。
- RS485 电气时序、真实磨削地面效果和 Aurora 内部定位算法不在 Gazebo 模型内。
- 仿真覆盖配置默认关闭平台文件上传、MQTT 和云端视频，避免测试数据进入生产系统。

## STEP 重新导出

安装 FreeCAD Python3 后运行：

```bash
cd /mnt/e/CODE/C++/Grinder
export GRINDER_STEP_INPUT="/path/to/source.STEP"
export GRINDER_STEP_OUTPUT_STL="catkin_ws/src/grinder_gazebo/meshes/x920_frame.stl"
export GRINDER_STEP_REPORT="catkin_ws/src/grinder_gazebo/meshes/x920_frame_report.json"
freecadcmd catkin_ws/src/grinder_gazebo/tools/step_to_mesh.py
```
