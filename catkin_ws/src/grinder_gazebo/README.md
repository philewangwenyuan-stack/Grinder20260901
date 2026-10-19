# X920 Gazebo 仿真

该包默认使用与 X920 尺寸相近的简化方盒车体和研磨盘，并保持现有 Grinder APP、调度、导航和底盘 ROS 接口不变。STEP 网格仍可按需打开。

默认链路为虚拟 MID-360S 三维射线 + IMU → Super-LIO → 二维栅格地图 →
APP/调度/规划 → move_base + RPP/TEB → 仿真底盘 → Gazebo。APP 的建图、保存、
加载地图、提交初始位姿和定位就绪门控均使用实车的 `super_lio_mode_manager`。
启动后先在 APP 开始建图，静止等待初始化，再遥控探索。保存后建图停止；
选择保存的地图、设置初始位姿并等待定位 READY 后可开始作业。

`/map` 来自实际点云建图，`/lio/odom` 来自 Super-LIO；不再启动生成工厂真值地图
的 `slamware_sim_bridge.py`。Gazebo 真值仅发布到 `/grinder/sim/ground_truth/odom`
供仿真底盘反馈和对照，`/odom_wheel` 保留轮速反馈用途。
真值位姿的参考系为 `sim_world`；LIO 的原点由初始化确定，比较轨迹误差前需对齐初始位姿。
`odom -> base_link` 仅由 Super-LIO 发布；URDF 以 `base_link` 为根。

## 模型参数

- 默认视觉模型：`1.30 x 0.94 x 0.30 m` 方盒体，运动中心为 `base_link`，车体中心相对运动中心前移 `0.35 m`；后部较上一版缩短 `0.30 m`，前缘保持不变。
- 物理质量：整车总质量 `800 kg`（含磨盘、驱动轮、支撑球、雷达和相机）；质量加权重心位于磨盘中心（`x=0.55m`、盘面高度 `z=0.04m`），用于避免加速时翘头。
- 研磨盘：直径 `0.90 m`，中心 `x=0.55 m`，前缘与代价地图 footprint 的 `x=1.00 m` 对齐。
- 两个后驱动轮轴线中点为 `base_link`，`x=0.00m`；雷达位于其前方 `0.20m`、左右居中，即 `base_link -> laser_link = (0.20, 0, 0.55)m`，零旋转。`+X` 朝磨盘方向。`0.55m` 是沿用的轴心上方高度，约离地 `0.6975m`，不是已测安装高度。URDF 和托管 LIO 使用相同 launch 参数。前置磨盘作为唯一万向支撑。
- 仿真桥复用实车手动轮速方向参数 `drive_left_sign=-1`、`drive_right_sign=-1`；Gazebo 轮关节轴保持车体坐标 `+Y`，自动 `/cmd_vel` 不再额外翻转。APP 摇杆上推（协议 `remote_y=-1`）和导航正向均朝磨盘 `+X` 前方。
- 磨盘视觉圆盘朝车体 `+X` 前方，旋转正轴为 `+Z`；其下方使用隐藏球形接触体实现无摩擦全向支撑。支撑固定在 `base_link` 参考系并相对盘心前置 `0.15m`，避免磨盘旋转时支撑点绕到质心后方而抬头；水平盘面本身不参与碰撞。
- 驱动轮保留 `mu1=mu2=1.0` 的必要牵引力，否则差速驱动无法移动。
- 接触参数为 `kp=1000000`、`kd=10000`，用于抑制重车接触求解的数值振荡；传感器噪声另由 `sensor_noise.yaml` 注入。
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
sudo apt-get install -y ffmpeg python3-scipy python3-pip libgoogle-glog-dev libgflags-dev libapr1-dev
python3 -m pip install --user 'protobuf==4.25.1'
PROFILE=sim CATKIN_JOBS=2 CATKIN_LOAD=2 ./build_grinder_platform.sh
./start_grinder_sim.sh
```

构建复用 `catkin_ws/src/drivers/ws_livox/src/Livox_SDK2` 和 `livox_ros_driver2`，
在仓库的 `temp/grinder_sim_livox/` 中构建 SDK，ROS 驱动随主工作区显式按 ROS1 编译；
不执行供应商会清理目录的 `build.sh`，不修改其源码，也不运行物理雷达驱动。`PROFILE=sim` 同时编译
`super_lio`、`cloud_to_occupancy_grid` 和 Gazebo 传感器插件。
新克隆若未携带这些本地驱动源文件，需先按上述路径放入兼容的 SDK2/ROS1 驱动。

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
- `lidar_x:=0.20 lidar_y:=0.0 lidar_z:=0.55`：相对后轮轴中点的雷达安装位置，单位 m；改变位置后需重新建图。
- `lidar_horizontal_samples:=360 lidar_vertical_samples:=20`：轻量默认每帧最多 7200 条射线，10 Hz；无回波不输出点。高性能机器可用 `720` / `28`（每帧 20160 条射线，约 20 万射线/s）。规则射线采样不等于真实扫描轨迹。
- `lidar_max_range:=45.0`：虚拟雷达最大量程，单位 m，默认 45 m；必须大于最小量程 0.10 m。重启仿真后生效，仅缩短仿真射线范围，不修改实车参数；CPU 是否能维持 10 Hz 仍需实测。
- `sensor_noise_config:=/absolute/path/noise.yaml`：测距与 IMU 噪声参数。
- `super_lio_map_root:=/absolute/path/maps`：默认仓库 `maps/sim_mid360/`，与实机地图隔离；状态目录为 `temp/grinder_scheduler_mid360_sim_state/`。

APP 修改外参会改变定位估计参数，不会移动 Gazebo 模型。调整物理安装位置应使用
`lidar_x/y/z` 重启仿真，并重新建图。

## 传感器与误差模型

虚拟雷达覆盖水平 360°、垂直 -7°～52°；仿真默认量程 0.1～45 m，可通过 `lidar_max_range` 调整。视场参数参考
[Livox MID-360S 官方规格](https://www.livoxtech.com/mid-360s/specs)。
CPU 多层射线是规则角度网格近似，不复现真实非重复扫描轨迹、反射率、强光、
玻璃、多径和入射角造成的距离误差。反射强度使用固定兼容值。

点云使用 `livox_ros_driver2/CustomMsg` 发布到 `/livox/lidar`，内置 IMU 以
200 Hz 名义频率发布 `/livox/imu`，世界物理步长为 1 ms。两者由同一个 Gazebo ROS 节点发布，满足管理器的
单一传感器发布者检查。导航 `/scan` 由 Super-LIO 从该点云切片产生。
每帧射线在同一仿真时刻采样，`header.stamp/timebase` 为该时刻，所有点
`offset_time=0`；不伪造逐点时间，因此不能用于验证真实扫描运动畸变及去畸变性能。
IMU 相对雷达偏移 `(0.011, 0.02329, -0.04412)m`，匹配现有 LIO 的
`T_imu_laser` 配置；这是仿真坐标约定，并非新测得的 MID-360S 内部外参。
世界重力采用 `9.7946m/s²`，与现有 LIO 配置一致。

[`config/sensor_noise.yaml`](config/sensor_noise.yaml) 配置下列误差：

| 项目 | 默认示例值 | 模型 |
| --- | --- | --- |
| 测距抖动 | σ=0.01 m | 独立高斯噪声 |
| 陀螺仪白噪声 | σ=0.001 rad/s | 三轴独立 |
| 加速度计白噪声 | σ=0.02 m/s² | 三轴独立 |
| 陀螺仪/加速度计初始零偏 | σ=0.0002 rad/s、0.01 m/s² | 启动时每轴随机 |
| 陀螺仪/加速度计缓慢零偏漂移 | 0.0001、0.001，相关时间 1000 s | Gazebo 一阶 Gauss–Markov |

这些是功能验证参数，未按实机数据标定；白噪声参数是每次采样的标准差，
不是 Super-LIO 内的连续时间噪声密度。将各标准差设为 0 可对比理想传感器。
运动与地面接触会产生物理加速度，但当前平地模型没有额外施加磨盘/电机机械振动。
IMU 总波动还包含接触求解的数值加速度，设置的白噪声 σ 不能直接当作最终输出的总标准差。
时间戳抖动、网络延迟、丢包、温漂和安装松动也未注入。
要逼近实机，应使用静止和行驶实测数据估计噪声/零偏，并另外测量振动谱及通信时序。

仿真运行后可在第二个已 source 工作区的终端检查传感器；加 `--map-cycle` 会执行
一次建图→保存新测试地图→重定位的服务回归（请在没有 APP 作业的独立仿真实例中运行）：

```bash
python3 src/grinder_gazebo/test/check_mid360_chain.py
python3 src/grinder_gazebo/test/check_mid360_chain.py --map-cycle
```

## APP 连接

- Windows 本机 APP：`127.0.0.1:8002`。
- 模拟器 本机 APP：`10.0.2.2:8002。
- 局域网手机 APP：连接 Windows 电脑局域网 IP 的 `8002` 端口。
- 左右 RTSP：`rtsp://<Windows-IP>:8554/left` 和 `/right`。

RTSP 编码依赖 WSL 内的 `ffmpeg`；启动脚本会在缺失时给出警告。
MST27 完整规划依赖 `python3-scipy`；缺失时调度器会自动退回备用规划器。

Windows 10 + WSL2 NAT 下，以管理员 PowerShell 运行以下脚本设置手机访问转发；WSL 重启、IP 改变后重新执行：

```powershell
& "E:\CODE\C++\Grinder\catkin_ws\src\grinder_gazebo\tools\update_wsl_portproxy.ps1"
```

## 仿真边界

- 不提供 Aurora STCM 及其模拟服务；地图保存为实际 Super-LIO PCD 与二维 YAML/PGM 资产包。
- 左右视频来自 Gazebo 相机；没有启用深度彩色图。
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
