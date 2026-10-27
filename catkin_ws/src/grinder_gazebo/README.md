# X920 Gazebo 仿真

以当前 main 的调度、MST27 路径规划和导航为基础，使用 Gazebo 虚拟 MID-360S 和 IMU 驱动真实 Super-LIO 建图/定位链路。仅传感器和底盘由仿真替代；默认不启动旧版 `slamware_sim_bridge`。STEP 网格仍可按需打开。

## 模型参数

- 默认视觉模型：`1.30 x 0.94 x 0.30 m` 方盒体，运动中心为 `base_link`，车体中心相对运动中心前移 `0.35 m`；后部较上一版缩短 `0.30 m`，前缘保持不变。
- 物理质量：整车总质量 `800 kg`（含磨盘、驱动轮、支撑球、雷达和相机）；质量加权重心位于磨盘中心（`x=0.55m`、盘面高度 `z=0.04m`），用于避免加速时翘头。
- 研磨盘：直径 `0.90 m`，中心 `x=0.55 m`，前缘与代价地图 footprint 的 `x=1.00 m` 对齐。
- 两个后驱动轮中点为 `base_link` 运动参考，雷达在其前方 `0.20m`，横向 `0m`，高于轮轴 `0.55m`。支撑球与滚动轴同心，避免滚动时支撑高度变化而翘头。
- 仿真桥复用实车手动轮速方向参数 `drive_left_sign=-1`、`drive_right_sign=-1`；Gazebo 轮关节轴保持车体坐标 `+Y`，自动 `/cmd_vel` 不再额外翻转。APP 摇杆上推（协议 `remote_y=-1`）和导航正向均朝磨盘 `+X` 前方。
- 磨盘视觉圆盘朝车体 `+X` 前方，旋转正轴为 `+Z`；其下方使用隐藏球形接触体实现无摩擦全向支撑。支撑固定在 `base_link` 参考系并相对盘心前置 `0.15m`，避免磨盘旋转时支撑点绕到质心后方而抬头；水平盘面本身不参与碰撞。
- 驱动轮保留 `mu1=mu2=1.0` 的必要牵引力，否则差速驱动无法移动。
- STEP 边界：`1.9623 x 1.9327 x 0.9000 m`，详见 `meshes/x920_frame_report.json`。
- 驱动轮距：`0.70 m`。
- 驱动轮半径：`0.1475 m`。
- 减速比：`60.0`。
- 简化物理碰撞体：`1.30 x 0.94 x 0.30 m`，与仿真代价地图 footprint `[-0.3, 1.0] x [-0.47, 0.47]` 一致；真实导航参数不改。
- 环境：来自 `S_B1混凝土结构模型_V1.0.rvt` 的梁、柱、墙、板精确三角网格，来源、原点与构件清单见 `meshes/b1_structure/source_report.json`。保留连续支撑地面；二维地图分配范围每轴 `[-60,+60]m`，裁剪外扩不能突破这个分配范围。

默认不加载 STEP，Gazebo collision 和 inertia 使用简化几何，以保证姿态稳定和实时性。仿真启动会用 `navigation_sim_overrides.yaml` 将 global/local costmap footprint 对齐到上述方盒尺寸；真实导航参数保持不改。需要查看 CAD 外观时运行 `./start_grinder_sim.sh visual_mesh:=true`。

## WSL 启动

完整链路：虚拟 MID-360S `/livox/lidar`（CustomMsg，10 Hz）和 `/livox/imu`（200 Hz）
→ `super_lio_mode_manager` 托管的 Super-LIO → `/lio/odom`、`/scan`、`/map`
→ main 的调度、规划、导航 → 仿真底盘。Gazebo 真值单独发布在
`/grinder/sim/ground_truth/odom`，不参与定位或发布导航 TF。

启动后在 APP 点击开始建图，并保持车辆静止等待 IMU 初始化；保存地图后进入定位流程。
地图写入仓库下 `maps/sim_mid360`，仿真状态写入 `temp/grinder_scheduler_mid360_sim_state`，
与生产地图、任务状态隔离。请重新建图，不直接使用旧版模拟器生成的占位地图。

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
正常测试：
```bash
cd /mnt/e/CODE/C++/Grinder/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
./start_grinder_sim.sh gui:=false
```
```bash
cd /mnt/e/CODE/C++/Grinder/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
./start_grinder_sim.sh gui:=false 2>&1 | \
grep --line-buffered -v -E \
'makePlan is called|CarrotPlanner external active segment unavailable or unmatched|External active segment plan is stale'
```

常用参数：

- `gui:=true|false`：Gazebo 图形界面。
- `visual_mesh:=true|false`：真实 STEP visual 或轻量盒体。
- `start_navigation:=true|false`：启动 move_base + RPP/TEB。
- `navigation_local_planner:=rpp|teb`。
- `local_rtsp_enabled:=true|false`。
- `sl_linka_port:=8002`。
- `lidar_x:=0.20 lidar_y:=0.0 lidar_z:=0.55`：雷达相对运动中心的位置；同时透传模型与 LIO 外参。
- `sensor_noise_config:=/absolute/path/noise.yaml`：雷达测距噪声、IMU 噪声和偏置漂移；默认见 `config/sensor_noise.yaml`，数值用于仿真，非实测标定。

更新 C++、ROS 消息或传感器插件后运行 `build` 再重启；Python、launch、YAML 修改后同步并重启相关节点。
`build` 包含独立 MST27 C++ 扩展，不能仅复制 `.py` 而继续使用旧 `.so`。
当前 main 业务源码不会从旧 Gazebo 分支回退。

仿真覆盖中 `collision_self_filter_enabled=false`，因为未配置独立的车体自身回波多边形。
只修改该自滤波开关，距离、车体外扩和代价地图碰撞检测保持各自原有配置。若启用该过滤但
`collision_self_filter_polygon=[]`，RPP 会以 `self_filter_invalid` 停车并在 APP 显示“避障配置异常”。
该参数在 RPP 初始化时读取，修改后须重启导航节点或整套仿真，单改 ROS 参数不会立即生效。

在独立仿真实例中验证传感器及建图→保存→定位就绪：

```bash
source /opt/ros/noetic/setup.bash
source catkin_ws/devel/setup.bash
python3 catkin_ws/src/grinder_gazebo/test/check_mid360_chain.py --map-cycle
```

`--map-cycle` 会切换仿真实例的建图/定位模式并新建测试地图，请勿在已有仿真任务执行中运行。

## APP 连接

- Windows 本机 APP：`127.0.0.1:8002`。
- 本机模拟器的主机网关地址取决于模拟器网络设置；远端 WSL + MuMu 使用下文 SSH 隧道与 ADB 映射，APP 输入 `127.0.0.1:8002`。
- 局域网手机 APP：连接 Windows 电脑局域网 IP 的 `8002` 端口。
- 左右 RTSP：`rtsp://<Windows-IP>:8554/left` 和 `/right`。

RTSP 编码依赖 WSL 内的 `ffmpeg`；启动脚本会在缺失时给出警告。
MST27 完整规划依赖 `python3-scipy`；缺失时调度器会自动退回备用规划器。

Windows 10 + WSL2 NAT 下，以管理员 PowerShell 运行以下脚本设置手机访问转发；WSL 重启、IP 改变后重新执行：

```powershell
& "E:\CODE\C++\Grinder\catkin_ws\src\grinder_gazebo\tools\update_wsl_portproxy.ps1"
```

## 远端 WSL 与本机 MuMu

工具默认连接 `dev@172.20.22.114:2222`，远端目录为
`/home/dev/work/Grinder-remote-sim`，SSH 私钥为本机
`%USERPROFILE%\.ssh\grinder_remote_sim`。
以下命令在 Windows PowerShell 执行，不要求当前目录在仓库内。

```powershell
# 同步源码；保留远端地图、运行数据、本地配置和远端独有文件
powershell -ExecutionPolicy Bypass -File E:\CODE\C++\Grinder\catkin_ws\scripts\remote_sim.ps1 sync
# 同步并编译 ROS 仿真包，以及 MST27 路径规划 C++ 扩展
powershell -ExecutionPolicy Bypass -File E:\CODE\C++\Grinder\catkin_ws\scripts\remote_sim.ps1 build
# 启动远端仿真，保持窗口打开；Ctrl+C 停止
powershell -ExecutionPolicy Bypass -File E:\CODE\C++\Grinder\catkin_ws\scripts\remote_sim.ps1 start
```

另外打开 PowerShell 建立隧道，保持这个窗口打开：

```powershell
powershell -ExecutionPolicy Bypass -File E:\CODE\C++\Grinder\catkin_ws\scripts\remote_sim.ps1 tunnel
```

再开一个 PowerShell 设置 MuMu 端口映射：

```powershell
powershell -ExecutionPolicy Bypass -File E:\CODE\C++\Grinder\catkin_ws\scripts\remote_sim.ps1 app
```

APP 输入 `127.0.0.1:8002`。隧道将 Windows 的 `18002/18554`
转发至远端 WSL 的 `8002/8554`，`app` 再将 MuMu 的 `8002/8554`
映射至 Windows 端口。MuMu 重启后重新运行 `app`。
隧道连接成功后没有输出是正常现象；需要远端仿真已启动才能连接服务。
本地 WSL 默认发行版为 `Ubuntu-20.04`，供同步、编译、启动及 RViz 使用。
`watch` 每隔约三秒同步，需手动编译和重启；`rviz` 通过 SSH X11 转发显示在本机。

## 仿真边界

- 当前主链路由 Super-LIO 生成真实点云地图与二维栅格；保留的旧 `slamware_sim_bridge.py` 不作为默认入口。
- MID-360S 使用 CPU 射线近似，支持测距噪声和 IMU 偏置漂移，不模拟真实非重复扫描花纹、逐点运动畸变或 UDP 丢包。
- 相机为彩色图，不提供度量深度。
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
