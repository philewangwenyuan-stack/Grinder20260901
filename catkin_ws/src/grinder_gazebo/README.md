# X920 Gazebo 仿真

## 同一场景运行 2～3 台车

入口 `catkin_ws/start_grinder_fleet.sh`。一个 Gazebo server 加载当前场景并生成
`robot1`、`robot2`、可选 `robot3`，车之间与梁柱墙板共享碰撞物理。
每台车有独立 ROS Master、Super-LIO、栅格地图、调度器、RPP 导航和 APP 双通道。
桥接只传该车的虚拟雷达、IMU、轮子关节状态、Gazebo 真值及控制输出；
不跨车转发 `/tf`、`/map`、任务或急停状态，因此各车保留原有 `map/odom/base_link` 名称。
Gazebo 真值只供仿真反馈和诊断，不替代 Super-LIO 定位。

原单车入口 `start_grinder_sim.sh` 保留。多车默认使用另一组端口，不会自动关闭已运行的单车。
同时运行两套场景会额外占用资源，正式多车测试前可在原单车启动终端按 Ctrl+C。
多车隔离并不包含共享地图、协同调度或会车协调，也不修复已有的定位质量问题。

### 编译与启动（远端 WSL）

在仓库根目录修改源码后，按本 README 的远端同步流程同步本次多车文件；
不要用较旧 main 的整个 Gazebo 目录覆盖远端定制场景。修改 C++ 插件后必须编译：

```bash
cd /home/dev/work/Grinder-remote-sim/catkin_ws
PROFILE=sim CATKIN_JOBS=4 CATKIN_LOAD=6 bash ./build_grinder_platform.sh
# 查看将采用的场景、出生位置和端口，不启动任何节点
bash ./start_grinder_fleet.sh --robots 2 --dry-run
# 两台车；第三台改成 --robots 3
bash ./start_grinder_fleet.sh --robots 2
# 不显示 Gazebo 窗口，仍保留 GPU 雷达渲染
bash ./start_grinder_fleet.sh --robots 2 --gui false
```

在启动终端 Ctrl+C 停止本次 fleet。脚本只管理自身启动的进程，不执行全局 `pkill`。
若在后台启动，可在同一工作区执行 `bash ./start_grinder_fleet.sh --stop`；
脚本核对 PID 的启动命令和工作区后再停止，不会关闭原单车实例。
机器人建图仍由各自 APP 的扫描操作启动，启动后先保持车辆静止完成 IMU 初始化。
桥接中断或控制包积压超期时，对应 Gazebo 输入会在实际时间约 0.5 秒内归零；
桥接进程退出时启动器停止本次 fleet，不持续运行失去控制桥的车辆。

配置为 `config/fleet.yaml`，也可用 `--config /absolute/path/fleet.local.yaml`。
空的 `world` 自动沿用设备本地 `catkin_ws/temp/sim_world_path`，出生基准沿用
`catkin_ws/temp/sim_spawn_pose`，各车 `spawn_offset: [x,y,yaw]` 是世界坐标方向的增量。
也可为每台车填写绝对 `spawn: [x,y,yaw]`；初始位置至少间隔 2m，且需自己选在可行驶区域。
`--world /absolute/path/scene.world`、`--sim-speed 1` 可覆盖场景及目标倍速。
默认关闭相机、RTSP、MQTT 和平台上传，避免多台模拟设备共用真实设备身份。

| 用途 | 共享世界 | robot1 | robot2 | robot3 |
| --- | --- | --- | --- | --- |
| ROS Master | 11411 | 11421 | 11422 | 11423 |
| Gazebo Master | 11445（所有车共享） | — | — | — |
| APP 数据端口 | — | 8102 | 8202 | 8302 |
| APP 控制端口 | — | 8103 | 8203 | 8303 |
| 默认出生偏移 | — | `[0,0,0]` | `[0,3,0]` | `[0,6,0]` |

地图分别存于仓库 `maps/sim_fleet/robotN/`；任务/草稿/历史、导航图、RPP/footprint
可写配置、预览及日志分别存于 `catkin_ws/temp/sim_fleet/robotN/`。
启动器首次复制 RPP 和 costmap 参数，此后保留各车在 APP 中修改的副本，不回写共享源码。
更新源码默认参数后，若要更新某车已存在的副本，请先停 fleet，再有选择地更新该车的
`rpp.yaml` / `costmap.yaml`。不要删除整车数据目录来更新参数。
运行清单在 `catkin_ws/temp/sim_fleet/manifest.json`，启动/桥接/调度日志也在该目录下。

### 本地 MuMu 的多个 APP 分别连接

**当前 APP 控制端口固定为 8003，不能只填写远端 8102/8202 就认为双通道已连接。**
每个 MuMu 实例均在 APP 填 `127.0.0.1:8002`，使用下面的脚本把两个通道一起映射到对应车辆。
一个 MuMu 实例同一时间连接一台车；同时操作两台车需要两个 MuMu 实例，分别安装 APP。

本机 Windows PowerShell 打开并保留一个隧道终端：

```powershell
powershell -ExecutionPolicy Bypass -File E:\CODE\C++\Grinder\catkin_ws\scripts\fleet_app.ps1 tunnel -Robots 2
# 三台车使用 -Robots 3
```

在另一终端用 MuMu 的 adb 查看实际实例序列号，再分别映射：

```powershell
& 'D:\Program Files\Netease\MuMu Player 12\nx_main\adb.exe' devices
powershell -ExecutionPolicy Bypass -File E:\CODE\C++\Grinder\catkin_ws\scripts\fleet_app.ps1 app -Robot 1 -AdbSerial 127.0.0.1:16384
# 下行的序列号要替换成 adb devices 列出的第二个 MuMu 实例
powershell -ExecutionPolicy Bypass -File E:\CODE\C++\Grinder\catkin_ws\scripts\fleet_app.ps1 app -Robot 2 -AdbSerial <第二个实例序列号>
```

映射为本机 `18102/18103 → 远端8102/8103`、`18202/18203 → 8202/8203`、
`18302/18303 → 8302/8303`。如用同一 MuMu 切车，重跑 `app -Robot N` 后在 APP 断开再重连。
MuMu 重启后需重设 adb reverse。物理平板也需要等价的双端口映射，或后续在 APP 支持可配置控制端口；
本次没有修改独立 APP 仓库。

### 每台车的 RViz 与验证

在带 X11 转发的 MobaXterm SSH 终端中：

```bash
cd /home/dev/work/Grinder-remote-sim/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
ROS_MASTER_URI=http://127.0.0.1:11421 ROS_HOSTNAME=127.0.0.1 rviz -d src/grinder_gazebo/config/remote_sim.rviz
# robot2 改为 11422，robot3 改为 11423
# 若原来使用 --gui false，可只给共享世界补开一个 Gazebo 窗口
GAZEBO_MASTER_URI=http://127.0.0.1:11445 gzclient
```

默认只读检查，确认虚拟传感器、APP 端口、Master 和存储隔离：

```bash
python3 src/grinder_gazebo/test/check_fleet.py
# 仅在无人操作、各车均未建图的测试 fleet 中：
# 同时建图/停止，再经实际 APP 双通道协议逐车短距离行走，检查其他车没有被带动
python3 src/grinder_gazebo/test/check_fleet.py --map-cycle --drive
# 同样仅限闲置测试 fleet：暂停共享物理时钟，验证控制桥的实际时间超时归零
python3 src/grinder_gazebo/test/check_fleet.py --drive --watchdog
python3 -m unittest discover -s src/grinder_gazebo/test -p test_fleet.py
```

目标倍速不等于实际倍速；多台雷达和多套 Super-LIO 都消耗资源。先以 1 倍测试，
并用各车 `/super_lio/localization_status`、`/lio/odom` 和共享 Gazebo 真值检查定位质量。

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

仿真底盘提供 `/chassis/reset_emergency_stop`：急停锁存后只有复位服务能解除，
复位会清除旧行走命令并关闭磨盘，保持停车直到收到新的控制命令。
离线安全回归见 `test/test_sim_emergency_reset.py`。

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

仿真默认不连接外部平台。需要让仿真上报 MQTT 和地图时，在 `scheduler.local.yaml`
配置 broker、设备码和凭据、平台账号后显式开启：

```bash
GRINDER_SCHEDULER_LOCAL_CONFIG="$PWD/src/grinder_scheduler/config/scheduler.local.yaml" \
  ./start_grinder_sim.sh sim_external_reporting:=true
```

MQTT 状态会周期上报；地图文件在 APP 保存地图时上传。已保存的旧地图不会在启动时自动补传，
需要在开启上报后通过 APP 再保存一次。若平台账号关联多个项目，还要填写 `platform_project_id`。

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

- `world_name:=/absolute/path/scene.world`：选择 Gazebo 场景；默认使用 B1。
- `gui:=true|false`：Gazebo 图形界面。
- `sim_speed:=2`：目标2倍仿真速度，默认`1.0`；支持正数，例如`0.5`、`2`、`4`。实际速度受算力限制。
- `visual_mesh:=true|false`：真实 STEP visual 或轻量盒体。
- `start_navigation:=true|false`：启动 move_base + RPP/TEB。
- `navigation_local_planner:=rpp|teb`。
- `local_rtsp_enabled:=true|false`。
- `sim_external_reporting:=true|false`：默认关闭仿真对外 MQTT 和平台地图上传。设为 `true` 后，需要通过 `scheduler.local.yaml` 配置 MQTT broker/设备码/凭据及平台账号；地图在 APP 保存时上传。用于仿真时请确认本地配置指向预期的测试服务。
- `sl_linka_port:=8002`。
- `lidar_x:=0.20 lidar_y:=0.0 lidar_z:=0.55`：雷达相对运动中心的位置；同时透传模型与 LIO 外参。
- `sensor_noise_config:=/absolute/path/noise.yaml`：雷达测距噪声、IMU 噪声和偏置漂移；默认见 `config/sensor_noise.yaml`，数值用于仿真，非实测标定。

外部场景可在 `catkin_ws/temp/sim_world_path` 保存世界文件的绝对路径，后续直接
运行 `bash ./start_grinder_sim.sh` 即加载该场景。该文件不进入 Git，远端源码同步也会保留。
环境变量 `GRINDER_SIM_WORLD` 可覆盖此选择，命令行 `world_name:=...` 优先级最高。
脚本自动将世界文件旁的 `../models` 目录加入 `GAZEBO_MODEL_PATH`，并在启动日志打印实际世界路径。
例如已部署花都模型的远端 WSL：

```bash
mkdir -p temp
printf '%s\n' /home/dev/work/huadu_factory_gazebo/worlds/huadu_factory.world > temp/sim_world_path
DISPLAY=:0 bash ./start_grinder_sim.sh
```

更换场景前先停止已有仿真实例；世界文件只会在 Gazebo 启动时加载。

启动命令可直接指定目标倍速，无需修改场景world文件：

```bash
./start_grinder_sim.sh gui:=false sim_speed:=2
```

远程启动同样透传：`remote_sim.ps1 start sim_speed:=2`。同步修改后的源码及脚本后再使用；本功能不会自动改变已运行的仿真。

`spawn_x920.launch`启动一次性的`set_sim_speed.py`，等待Gazebo物理服务和动态配置初始化完成后，读取当前场景的物理时间步长，按“更新频率=目标倍速÷时间步长”设置上限；重力和ODE配置原样保留，不修改原始world或机器人行驶速度。仿真时钟整体加快；实际倍速可用`gz stats`观察。设置后按墙钟读回确认至少3秒，启动期间被动态配置覆盖会重试；持续覆盖、无效参数或服务失败会在启动日志报错。业务实现及离线回归分别见`src/grinder_gazebo/sim_speed.py`与`test/test_sim_speed.py`。
可在 `catkin_ws/temp/sim_spawn_pose` 保存一行 `x y yaw`（米、弧度），
作为该设备的机器人出生点；命令行 `x:=... y:=... yaw:=...` 可分别覆盖。
切换场景时应同步调整或移除该出生点文件。首层 `z=0` 对应的车体出生高度仍为 `0.1675m`。
花都场景保持首层地面 `z=0`，可沿用默认机器人出生点。其完整范围约 `192 x 295m`，
当前默认建图范围 `[-60,+60]m` 只能覆盖原点附近区域；切换后需重新建图，不复用 B1 定位地图。

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


### GPU MID-360S 扫描后端

默认 `lidar_backend:=gpu`，CPU 回退为 `lidar_backend:=cpu`。两种后端均输出
`/livox/lidar` 的 `livox_ros_driver2/CustomMsg`，保持 `laser_link`、仿真时间戳、
10 Hz、360×20 扫描网格、水平360°、垂直-7°～52°和 `sensor_noise.yaml` 测距噪声。
IMU保持独立的原有200 Hz模拟。仍是瞬时规则网格近似，点内时间偏移为0，不是实机非重复扫描轨迹。

```bash
./start_grinder_sim.sh sim_speed:=4 lidar_backend:=gpu
# 无图形窗口也需要可用的GPU渲染上下文，例如远端WSLg的 DISPLAY=:0
./start_grinder_sim.sh gui:=false sim_speed:=4 lidar_backend:=gpu
```

GPU使用Gazebo 11的 `GpuRaySensor`，内部三相机拼接全周扫描，避免CPU ODE射线扫描占用物理碰撞锁。
它查询可视几何，CPU射线查询碰撞几何；场景两者应一致。GPU失败不会静默切回CPU，应检查
Gazebo/Ogre日志和 `/livox/lidar`，或显式指定CPU。WSLg需确认 `glxinfo -B` 显示硬件加速。

当前验证环境是WSL/WSLg、Mesa D3D12、RTX4090。该环境的Classic Ogre点栅格读回存在一行一列偏移，
默认 `gpu_ray_sample_offset:=1` 通过下边界保护行列补偿，剔除未填充尾部纹素，保持原有视场和点数。
其他渲染后端需先跑下面的几何回归；若无偏移可指定 `gpu_ray_sample_offset:=0`，不要只凭画面判断。
本参数经顶层launch、spawn launch、URDF透传给插件，CPU忽略该补偿。

配置回归：`python3 -m unittest discover -s src/grinder_gazebo/test -p test_gpu_lidar_config.py`（catkin_ws内）。
几何/噪声回归仅在独立终端、独立ROS/Gazebo主端口执行，避免干扰APP实例：

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11381 GAZEBO_MASTER_URI=http://127.0.0.1:11382 DISPLAY=:0
python3 src/grinder_gazebo/test/check_gpu_lidar.py --write-world /tmp/gpu_room.world
roslaunch grinder_gazebo spawn_x920.launch gui:=false world_name:=/tmp/gpu_room.world lidar_backend:=gpu
# 在采用相同环境变量的另一终端执行，完成后Ctrl+C关闭这个独立launch
python3 src/grinder_gazebo/test/check_gpu_lidar.py --check
```

回归验证已知四面墙、地面和顶板的点云几何、完整视场、噪声、点数、帧/时间戳和IMU。
性能以实际 `gz stats` 为准；倍速是目标上限，不保证达到指定倍数。

2026-11-04远端实测：RTX4090/WSLg、花都厂房约541万三角面、目标4倍、保留原场景和相机，CPU雷达约1.18倍，GPU雷达约3.7倍。解析房间输出7200点/帧、10 Hz（仿真时间），平墙误差中位数5.8 mm、99分位22.8 mm、测距样本标准差11.9 mm；独立厂房实例完成建图→保存→定位READY。性能数值仅代表该次测试条件。
