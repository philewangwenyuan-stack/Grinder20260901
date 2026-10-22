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
- 环境：从用户提供的 `S_B1混凝土结构模型_V1.0.rvt` 提取梁、柱、墙、板，替换原来的规则柱网、边墙与示例梁板。共有 **767 根梁、199 根柱、191 面墙、557 块板**，平面范围约 `104.45m x 82.57m`，高度范围约 `-0.55m～7.20m`（相对 B1 地面）。
- 构件保留 Revit 原始面三角化后的轮廓、位置关系、斜面和洞口；不以包围盒或凸包代替构件。四类构件共 `34,530` 个三角面，按类别合并成静态 visual/collision 网格，参与虚拟雷达射线和物理碰撞。28 个没有可提取几何的元素（3 根梁、25 块板）未生成网格，几何提取错误为 0。
- Revit 内部英尺按 `0.3048` 转成米，保持模型朝向，仅整体平移。源坐标 `(51.22561494, 21.95509716, -6.0)m` 对应 Gazebo 原点；`B1` 标高 `-6m` 对齐仿真 `z=0`。原点选在构件净空范围，机器人原有出生位姿参数保持不变。原有 `ground_plane` 继续提供连续地面支撑；这是仿真支撑面，不是从 RVT 新提取的构件。

默认不加载 STEP，Gazebo collision 和 inertia 使用简化几何，以保证姿态稳定和实时性。仿真启动会用 `navigation_sim_overrides.yaml` 将 global/local costmap footprint 对齐到上述方盒尺寸；真实导航参数保持不改。需要查看 CAD 外观时运行 `./start_grinder_sim.sh visual_mesh:=true`。

楼层场景位于 [`worlds/factory_floor.world`](worlds/factory_floor.world)，其中
`b1_beam`、`b1_column`、`b1_wall`、`b1_slab` 分别加载
[`meshes/b1_structure/`](meshes/b1_structure/) 内的四类构件。
[`source_report.json`](meshes/b1_structure/source_report.json) 保存源文件 SHA-256、单位换算、
坐标平移、构件 ID、数量和网格校验值；原始 RVT 不随仓库分发，也未被保存或修改。
`spawn_x920.launch` 将包目录加入 `GAZEBO_RESOURCE_PATH`，用于解析可移植网格 URI。
只修改 world 无需重新编译；远端使用时先 `sync`，停止旧仿真后重新 `start` 才会加载新几何，
重新建图后再验证定位，避免旧三维地图与新的楼层结构不一致。
顶部楼板为不透明实体，Gazebo 从楼顶俯视会被遮挡，可将相机移入楼层观察；
RViz 的机器人模型显示不包含 world 梁板，应通过建图点云查看扫描到的楼层结构。

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

本节是仿真运行在本机 WSL 时的连接方式；**另一台电脑运行仿真、本机 MuMu 显示 APP**
请按下一节的 `tunnel` + `app` 操作，不直接套用本节 IP。

- Windows 本机 APP：`127.0.0.1:8002`。
- Android 模拟器：宿主机地址取决于模拟器网络配置；MuMu 推荐使用下一节的 ADB reverse，映射后连接 `127.0.0.1:8002`。
- 局域网手机 APP：连接 Windows 电脑局域网 IP 的 `8002` 端口。
- 左右 RTSP：`rtsp://<Windows-IP>:8554/left` 和 `/right`。

RTSP 编码依赖 WSL 内的 `ffmpeg`；启动脚本会在缺失时给出警告。
MST27 完整规划依赖 `python3-scipy`；缺失时调度器会自动退回备用规划器。

Windows 10 + WSL2 NAT 下，以管理员 PowerShell 运行以下脚本设置手机访问转发；WSL 重启、IP 改变后重新执行：

```powershell
& "E:\CODE\C++\Grinder\catkin_ws\src\grinder_gazebo\tools\update_wsl_portproxy.ps1"
```

## 本地源码 + 远端 WSL 运行

### 1. 两台电脑分别做什么

本机 Windows 编辑源码、运行 MuMu；本机 WSL 提供同步客户端和 WSLg 显示。
另一台电脑的 WSL 负责 Linux 编译、Gazebo、Super-LIO、调度、导航和 RViz 计算。
RViz 窗口通过 SSH X11 转发显示在本机，不需要在本机再启动一套 ROS 节点。

| 内容 | 当前路径或地址 |
| --- | --- |
| 本机唯一开发仓库 | `E:\CODE\C++\Grinder`，本机 WSL 中为 `/mnt/e/CODE/C++/Grinder` |
| 远端 Windows 局域网地址 | `172.20.22.114` |
| SSH 入口 | `dev@172.20.22.114:2222`，转发到远端 WSL 的 22 端口 |
| 远端 WSL 内网地址示例 | `172.27.203.246`，重启后可能变化，不作为本机 SSH 目标 |
| 远端源码副本 | `/home/dev/work/Grinder-remote-sim` |
| 远端 Catkin 产物 | 上述目录下 `catkin_ws/build`、`catkin_ws/devel` |
| 远端规划器 C++ 构建目录 | 上述目录下 `temp/remote_mst27` |
| 远端仿真地图 | 上述目录下 `maps/sim_mid360` |
| 远端调度持久化状态 | 上述目录下 `temp/grinder_scheduler_mid360_sim_state` |

远端有完整源码副本和远端编译产物。日常只编辑本机这一份，使用 `rsync` 单向增量同步，
未提交的源码也会同步，不需要先提交或推送 Git。远端镜像不携带 `.git`，不要在镜像内手工改源码。

### 2. 首次准备环境与 SSH

**已经配置成功的电脑跳过本小节，从第 3 步开始。** 以下 Linux 命令在注明的 WSL 中执行，
不要粘贴到 Windows CMD；本机运行脚本所需的默认发行版名为 `Ubuntu-20.04`。

本机 WSL 安装客户端依赖，并确认 Windows 可通过 WSLg 显示窗口：

```bash
sudo apt-get update
sudo apt-get install -y openssh-client rsync xauth
echo "$DISPLAY"
```

远端 WSL 使用 Ubuntu 20.04，先安装 SSH 和同步所需的基础工具：

```bash
sudo apt-get update
sudo apt-get install -y openssh-server rsync xauth
sudo service ssh start
hostname -I
sudo ss -lntp 'sport = :22'
```

若使用 WSL NAT，在**远端电脑的管理员 PowerShell** 配置 SSH 转发；
下面的 `wsl` 查询默认发行版，若目标不是默认发行版，应先用 `wsl -l -v` 确认并增加 `-d <发行版名>`。

```powershell
$simWslIp = ((wsl -- hostname -I) -join ' ').Trim().Split(' ')[0]
netsh interface portproxy add v4tov4 listenaddress=172.20.22.114 listenport=2222 connectaddress=$simWslIp connectport=22
New-NetFirewallRule -DisplayName 'Grinder remote WSL SSH 2222' -Direction Inbound -Action Allow -Protocol TCP -LocalAddress 172.20.22.114 -LocalPort 2222 -RemoteAddress LocalSubnet -Profile Private
netsh interface portproxy show v4tov4
```

防火墙规则只需创建一次；以上示例要求局域网连接使用 Windows“专用”网络配置。
WSL IP 变化后重新执行地址查询和 `portproxy add` 更新目标。Windows 局域网 IP 变化时也应更新监听地址和防火墙规则。

本机 PowerShell 查看专用 SSH 公钥；尚未生成密钥时才执行 `ssh-keygen`，不要覆盖已有私钥：

```powershell
# 仅首次、文件不存在时生成
ssh-keygen -t ed25519 -f "$env:USERPROFILE\.ssh\grinder_remote_sim" -C grinder-remote-sim
Get-Content "$env:USERPROFILE\.ssh\grinder_remote_sim.pub"
```

把输出的**公钥整行**追加到远端 `dev` 用户的 `~/.ssh/authorized_keys`，保留已有内容：

```bash
# 在远端 WSL 的 dev 终端执行，编辑器中追加公钥
mkdir -p ~/.ssh
chmod 700 ~/.ssh
nano ~/.ssh/authorized_keys
chmod 600 ~/.ssh/authorized_keys
```

RViz 转发还要求远端 SSH 配置启用 `X11Forwarding yes`；修改后执行 `sudo service ssh restart`。
本机私钥不上传仓库或远端，`watch` 需要密钥能非交互登录；若使用带口令的密钥，需预先配置可用的 SSH agent。

### 3. 首次同步、安装依赖和构建

以下所有 `remote_sim.ps1` 命令均在**本机 PowerShell** 的仓库根目录执行：

```powershell
Set-Location 'E:\CODE\C++\Grinder'
powershell -ExecutionPolicy Bypass -File .\catkin_ws\scripts\remote_sim.ps1 doctor
powershell -ExecutionPolicy Bypass -File .\catkin_ws\scripts\remote_sim.ps1 sync
powershell -ExecutionPolicy Bypass -File .\catkin_ws\scripts\remote_sim.ps1 shell
```

`doctor` 检查 SSH 和部分基础环境，不代表所有依赖或仿真链路已经通过。
`shell` 打开远端终端。首次在其中安装依赖，输入远端用户自己的 sudo 密码：

```bash
cd /home/dev/work/Grinder-remote-sim
sudo bash catkin_ws/scripts/install_remote_sim_deps.sh
# 可选：RViz 的 JSK 显示插件；当前远端已安装
sudo apt-get install -y ros-noetic-jsk-rviz-plugins
exit
```

安装脚本配置 ROS 软件源并安装 ROS Noetic、Gazebo、导航、编译工具和 Python 依赖，
不启动实机服务。JSK 当前是上面单独安装的可选依赖，不在安装脚本默认清单中。
随后回到本机 PowerShell 构建：

```powershell
powershell -ExecutionPolicy Bypass -File .\catkin_ws\scripts\remote_sim.ps1 build
```

`build` 会**先同步源码，再在远端执行 `PROFILE=sim` 构建，并单独构建 MST27 C++ 规划扩展**。
默认使用 8 个并行构建任务；本机 Windows 的旧 `.so`、`build/devel` 不会被复制过去使用。
首次构建还需要本机有上述 Livox SDK2/ROS1 驱动源码；缺失时先补齐源码再同步。

### 4. 每次启动：仿真、隧道、APP、RViz

启动前先确认上一套仿真已退出。每个终端都先切换到 `E:\CODE\C++\Grinder`，
下列前台进程分别使用不同终端，运行期间保持终端打开。

**终端 A：同步并启动远端仿真。** `start` 本身不自动同步、不编译。

```powershell
powershell -ExecutionPolicy Bypass -File .\catkin_ws\scripts\remote_sim.ps1 sync
powershell -ExecutionPolicy Bypass -File .\catkin_ws\scripts\remote_sim.ps1 start
```

默认 `gui:=false`，只在远端运行 Gazebo 服务；相机仍使用远端 WSLg 渲染。
调度和导航会启动，Super-LIO 初始为待命，需 APP 开始建图或加载地图才进入对应模式。

**终端 B：建立本机到远端的 APP/视频隧道。** 已有可用隧道时复用，不重复启动。

```powershell
powershell -ExecutionPolicy Bypass -File .\catkin_ws\scripts\remote_sim.ps1 tunnel
```

**终端 C：先启动 MuMu，再执行端口映射。** 此命令执行完成后会返回，可继续用该终端。

```powershell
powershell -ExecutionPolicy Bypass -File .\catkin_ws\scripts\remote_sim.ps1 app
```

打开 MuMu 中的 APP，固定 IP 填 **`127.0.0.1:8002`**。这里能使用回环地址，
是因为 `app` 已建立 ADB reverse；未映射时，该地址只指向 Android 自己。

| 通路 | 转发过程 |
| --- | --- |
| APP 协议 | MuMu `127.0.0.1:8002` → 本机 Windows `127.0.0.1:18002` → SSH → 远端 WSL `127.0.0.1:8002` |
| RTSP TCP | MuMu `127.0.0.1:8554` → 本机 Windows `127.0.0.1:18554` → SSH → 远端 WSL `127.0.0.1:8554` |

本机隧道端口使用 `18002/18554`，以便与本机仿真的 `8002/8554` 区分。
`app` 只配置映射，不启动仿真或 SSH 隧道。MuMu 重启后要重新执行 `app`。
视频还取决于 APP 使用的 RTSP 地址与 TCP 传输方式；隧道端口连通不等于视频播放验证通过。
APP 若强制 UDP 视频，当前 TCP 隧道不转发其 UDP 数据。

**终端 D：打开本机显示的 RViz。**

```powershell
powershell -ExecutionPolicy Bypass -File .\catkin_ws\scripts\remote_sim.ps1 rviz
```

RViz 进程在远端，ROS 连接留在远端，不需向局域网开放 ROS 动态 TCP 端口。
默认加载 `config/remote_sim.rviz`，固定坐标系 `map`，显示地图、机器人和规划路径；
点云和 TF 默认关闭，使用软件 OpenGL、15 FPS 以减少显示开销。
开始建图前没有 `/map` 或相关 TF、RViz 地图为空属于待命状态。

JSK 安装后重新打开 RViz，在 **Add → By display type → jsk_rviz_plugins** 中选择
`OverlayText`、`BoundingBoxArray`、`PolygonArray` 等显示类型。
每种显示仍需对应消息话题，安装插件本身不会自动产生数据。
RViz 的 Save Config 写入的是**远端文件**；需要保留布局时，将其保存到远端 `temp/`，
取回本机再维护本机的 `.rviz` 文件，避免下一次同步覆盖远端手改布局。

### 5. APP 建图和任务联调顺序

1. APP 连接成功后开始建图，先保持机器人静止，等待 Super-LIO 初始化。
2. 再遥控探索，观察 RViz 地图、机器人朝向与 APP 是否一致。
3. 在 APP 编辑地图、划分工作区、保存地图。保存成功后建图停止。
4. 加载保存的地图，按 APP 流程提交初始位姿，等待定位达到 `READY`。
5. 再创建任务、查看规划并开始作业；定位未就绪时不绕过任务启动门控。

地图和状态实际保存在远端，不会自动回传本机。调整雷达物理安装位置后需重新建图，
原有地图不要直接用于验证新的安装外参。

### 6. 日常修改代码和参数

编辑本机文件并保存后，手动同步：

```powershell
powershell -ExecutionPolicy Bypass -File .\catkin_ws\scripts\remote_sim.ps1 sync
```

也可另开**终端 E** 持续同步，默认每次同步结束后间隔 3 秒，`Ctrl+C` 结束监听：

```powershell
powershell -ExecutionPolicy Bypass -File .\catkin_ws\scripts\remote_sim.ps1 watch
```

`sync/watch` 只更新磁盘文件，**不自动构建、不重启节点，也不热更新运行中的参数**。
较大的多文件修改建议关闭 `watch`，改完后一次 `sync`，避免远端在不完整修改期间重启或加载地图模式。

| 改动内容 | 同步后如何生效 |
| --- | --- |
| 调度、地图、规划适配层等 Python | 停止本次任务/建图，重启相关节点；日常建议按第 8 步重启整套仿真 |
| YAML 参数、launch | 一般无需编译，重启读取该配置的节点；未明确支持动态加载时按需重启整套仿真 |
| URDF/Xacro、world、传感器噪声配置 | 重启 Gazebo，重新生成模型；安装位置变化后重新建图 |
| C++、规划器 C++ 扩展、Gazebo 插件 | 先停止仿真，再执行 `build`，成功后重新 `start` |
| `msg/srv`、`package.xml`、`CMakeLists.txt` | 执行 `build` 后重启相关节点 |
| RViz 布局 `.rviz` | 同步后重新打开 RViz 或重新加载配置 |

不要在另一个终端重复 `start` 来代替重启；旧节点会占用相同端口和 ROS 名称。
编译成功也不代表正在运行的 C++ 进程已经换成新版本。

### 7. 参数改哪个文件，以及覆盖顺序

| 要调整的内容 | 本机仓库相对路径 |
| --- | --- |
| 公共调度、规划等默认配置 | `catkin_ws/src/grinder_scheduler/config/scheduler.yaml` |
| Super-LIO 模式、定位质量门限 | `catkin_ws/src/grinder_scheduler/config/super_lio.yaml` |
| 仿真专用调度覆盖、车体尺寸、视频等 | `catkin_ws/src/grinder_gazebo/config/scheduler_sim_overrides.yaml` |
| 仿真导航 footprint 等覆盖 | `catkin_ws/src/grinder_gazebo/config/navigation_sim_overrides.yaml` |
| 雷达测距、IMU 噪声和零偏 | `catkin_ws/src/grinder_gazebo/config/sensor_noise.yaml` |
| 雷达安装位置、采样数、量程等默认启动值 | `catkin_ws/src/grinder_gazebo/launch/grinder_sim.launch` |
| RViz 默认显示布局 | `catkin_ws/src/grinder_gazebo/config/remote_sim.rviz` |

调度节点参数按以下顺序加载，后者覆盖前者的同名值：
`scheduler.yaml` → `super_lio.yaml` → `scheduler_local_config` →
`scheduler_sim_overrides.yaml` → launch 中显式设置的节点参数。
模式管理器单独加载 `super_lio.yaml`，安装位置等再由 launch 设置；
并非调度节点上的每个参数都会同步应用到模式管理器。
只改仿真行为时优先修改仿真文件；改公共默认值会影响以后部署的实车配置。

**`scheduler.local.yaml` 不参与源码同步。** 它属于设备覆盖文件，不能依靠修改本机该文件来更新远端。
`start` 每次会重写远端 `temp/remote_sim/scheduler.local.yaml` 并作为本次调度覆盖输入；
其中同名值仍可能被后加载的仿真 YAML 覆盖，不要手改这个临时文件保存长期配置。

临时实验也可在 `start` 后追加 launch 参数，不改文件。例如先停止旧仿真，再启动：

```powershell
powershell -ExecutionPolicy Bypass -File .\catkin_ws\scripts\remote_sim.ps1 start lidar_max_range:=30.0 navigation_local_planner:=teb
```

参数中的文件路径必须是**远端 Linux 路径**。例如自定义噪声文件同步后，传入：
`sensor_noise_config:=/home/dev/work/Grinder-remote-sim/catkin_ws/src/grinder_gazebo/config/sensor_noise.yaml`，
不能传 `E:\...` 或本机 WSL 的 `/mnt/e/...`。
启动参数只影响这一次进程，下次启动仍要传入，或修改本机 launch 默认值后同步。

### 8. 停止、重启和退出

1. 先在 APP 停止任务，结束当前建图操作；需要的地图先保存。
2. 在**终端 A** 按 `Ctrl+C` 退出仿真，等待节点清理结束。
3. 根据修改类型执行 `sync` 或 `build`，然后在终端 A 重新执行 `start`。
4. 仍在运行的 SSH 隧道和 ADB 映射可继续使用；需要时重新连接 APP、重新加载地图并等待定位就绪。
5. 完全退出时关闭 RViz，在隧道和 `watch` 终端按 `Ctrl+C`；停止隧道不会代替停止远端仿真。

若启动终端已丢失或 SSH 意外断开，不能仅凭本机窗口消失判断远端已停止。
在新的本机终端执行 `shell`，在远端检查：

```bash
pgrep -af 'roslaunch.*grinder_sim.launch'
ss -lntp 'sport = :8002 or sport = :8554'
```

若存在上次启动的进程，核对其命令和工作目录确为本次镜像，
再对该 `roslaunch` 的实际 PID 执行 `kill -INT <实际PID>`，等待清理后复查端口。
不要使用会影响其他 ROS 实例的全局 `killall`。脚本当前没有独立的 `stop/restart` 动作。

### 9. 检查运行状态与日志

本机执行 `shell` 后，终端位于远端镜像根目录，已加载可用的 ROS/工作区环境：

```powershell
powershell -ExecutionPolicy Bypass -File .\catkin_ws\scripts\remote_sim.ps1 shell
```

以下命令在这个**远端 shell** 中执行，`rostopic hz` 查看几秒后按 `Ctrl+C` 返回：

```bash
rosnode list
rosparam get /use_sim_time
rostopic hz /livox/lidar
rostopic hz /livox/imu
rostopic echo -n 1 /super_lio/localization_status
rosparam get /grinder_scheduler/localization_backend
ls -lt ~/.ros/log/latest
python3 catkin_ws/src/grinder_gazebo/test/check_mid360_chain.py
```

传感器名义频率是仿真时间中的雷达 10 Hz、IMU 200 Hz；墙钟接收速率还受实时因子影响。
`check_mid360_chain.py` 默认执行传感器只读检查。
加 `--map-cycle` 会实际建图、保存测试地图并重定位，只在没有 APP 任务的独立测试时使用。
日志先看终端 A 的输出，再看远端 `~/.ros/log/latest`；编译错误则看 `build` 终端。

#### 定位异常导致任务暂停：记录触发前后的逐帧数据

APP 的“配准质量连续不合格”对应 `registration_quality_failed`，是定位门控触发任务暂停。
它不单指匹配误差过大，还可能是未收敛、内点比例不足、匹配数量不足或协方差超限。
只看 APP 的概括原因，无法区分这些条件；定位恢复后任务仍需用户确认继续。

在远端 `shell` 中查看实时话题（`Ctrl+C` 退出查看，不影响任务）：

```bash
rostopic echo /super_lio/registration_diagnostics
rostopic echo /super_lio/localization_status
```

重点对照模式管理器实际参数和以下诊断字段：

| 字段或来源 | 含义 |
| --- | --- |
| `fitness_m2`、`inlier_ratio`、`valid_correspondences` | 匹配残差、内点比例、有效匹配数量 |
| `converged`、诊断 `level` | 本帧是否收敛、算法是否报告有效质量 |
| `kf_iterations`、`kf_max_iterations` | 实际迭代次数与迭代上限 |
| `kf_convergence_reason` | 如 `threshold_reached`、`iteration_limit`，区分正常收敛与迭代耗尽 |
| `kf_final_update_inf`、`kf_quit_eps`、`kf_final_max_component` | 最终状态更新量、收敛阈值及最大更新分量；不同状态分量单位不同，不能统一解释成距离 |
| `/lio/odom` 的 `pose.covariance[0/7/35]` | X/Y/航向方差，对照模式管理器方差门限 |
| 定位状态中的 `reason`、`good_frames`、`bad_frames` | 当前原因、连续好帧与坏帧计数 |

要捕获偶发暂停，在单独远端终端记录下面的话题；记录器只订阅，不下发运动指令。
每段约 32 MB，滚动保留最多 4 段，分段期间另有活动文件与少量格式开销。
旧段会被淘汰，发生暂停后应及时结束记录并备份，避免故障片段被覆盖。
若 `/localization_pause_recorder` 已存在，复用已有记录器，不重复启动同名节点。

```bash
mkdir -p temp/localization_monitor
rosparam dump temp/localization_monitor/mode_manager_parameters.yaml /super_lio_mode_manager
rosbag record --split --size=32 --max-splits=4 --buffsize=16 \
  -O "temp/localization_monitor/pause_$(date +%Y%m%d_%H%M%S)" \
  /super_lio/registration_diagnostics /super_lio/localization_status \
  /lio/odom /clock /cmd_vel /chassis/task_enable /scheduler/status /rosout \
  __name:=localization_pause_recorder
```

前台记录按 `Ctrl+C` 正常结束；若记录器在后台运行，可在远端执行：

```bash
rosnode kill /localization_pause_recorder
ls -lh temp/localization_monitor
```

结束后 `.bag.active` 会正常关闭为 `.bag`，可用 `rosbag info <文件.bag>` 查看。
分析时按消息头仿真时间关联暂停前后数据，并保留 APP 看到的墙钟时间作为索引，
不要直接把仿真时间数字当作 Windows 日期。
该记录不包含大点云或完整 IMU，适合先定位门控触发条件；算法复现需另行采集输入传感器数据。
本机 `sync/watch` 不回传或清除远端 `temp/`，需要时用 `scp` 单独取回记录。

### 10. 常见问题

| 现象 | 检查和处理 |
| --- | --- |
| SSH `Connection reset/refused` | 在远端 WSL 启动 `ssh`，确认监听 22；在远端 Windows 检查 2222 转发是否指向当前 WSL IP |
| `Permission denied (publickey)` | 检查 `dev` 的 `authorized_keys`、700/600 权限和本机 `-Identity` 对应私钥；不要把公钥当私钥传入 |
| `rsync` 不存在 | 本机 WSL 和远端 WSL 都安装 `rsync`，Windows 装了 SSH 不能替代 WSL 客户端依赖 |
| 拒绝覆盖非空镜像目录 | `sync` 只接管空目录或带正确来源标记的镜像；改用新的专用目录，不往已有工程目录强行添加标记 |
| 改配置后没变化 | 确认已经 `sync`、相关节点重启、文件实际被加载，并按第 7 步核对覆盖顺序；用 `rosparam get` 查看运行值 |
| 本机 `18002/18554` 被占用 | 已有隧道时复用；在本机 `Get-NetTCPConnection -LocalPort 18002,18554 -State Listen` 查看，关闭确定属于旧隧道的进程后再建 |
| 远端 `8002/8554` 被占用 | 按第 8 步检查并退出上一套仿真；不能靠再开一个 `start` 更新 |
| MuMu 连接失败 | 确认仿真、`tunnel` 均在运行，MuMu 启动后重新 `app`；APP 填 `127.0.0.1:8002` |
| MuMu ADB 路径/实例不匹配 | 用 `-Adb` 指定实际 `adb.exe`，用其 `devices` 查看实例后设置 `-AdbSerial` |
| RViz 无窗口或 `DISPLAY` 错误 | 检查本机 WSLg 的 `DISPLAY`、两边 `xauth` 和远端 `X11Forwarding yes`；RViz 使用单独的 `rviz` 动作 |
| RViz 地图为空 | 先确认 APP 已开始建图或加载地图；再查 `/map`、Super-LIO 状态和 `map` TF，待命时为空正常 |
| RViz 没有 JSK 类型 | 在远端安装 `ros-noetic-jsk-rviz-plugins`，关闭后重新启动 RViz |
| 视频连接失败 | 检查远端 8554、两级映射、APP 实际收到的 RTSP 主机地址和 TCP 传输方式；协议连接成功不代表视频连接成功 |
| startup health timeout | 在远端查 LiDAR/IMU 是否持续更新、实时因子和地图模式日志；不通过关闭就绪检查掩盖问题 |

### 11. 同步范围、改地址与地图取回

同步排除 Git、本地文档缓存、`build/devel/install/logs/temp`、根目录 `maps/`、
PCD/STCM、设备 `scheduler.local.yaml` 和项目生成的 `.so/.a`。
构建依赖的 Aurora 厂商 SDK 自带库单独允许同步，项目自身库仍由远端重新编译。
远端镜像中已从本机删除的源码会在下次同步时删除；排除的构建目录、地图和运行状态保留。
运行时数据不会反向覆盖本机；需要备份地图时，在保存完成后用独立目录取回，例如本机 PowerShell：

```powershell
New-Item -ItemType Directory -Force 'E:\Grinder-sim-map-backup' | Out-Null
scp -i "$env:USERPROFILE\.ssh\grinder_remote_sim" -P 2222 -r dev@172.20.22.114:/home/dev/work/Grinder-remote-sim/maps/sim_mid360 'E:\Grinder-sim-map-backup'
```

更换电脑时可用 `-RemoteHost`、`-Port`、`-RemoteUser`、`-RemoteRoot`、`-Identity`、
`-Distro` 覆盖默认连接配置；每个相关命令使用相同的一组参数。例如：

```powershell
powershell -ExecutionPolicy Bypass -File .\catkin_ws\scripts\remote_sim.ps1 doctor -RemoteHost 192.168.1.20 -Port 2222
```

`-Adb`、`-AdbSerial` 用于 MuMu；`-LocalControlPort`、`-LocalRtspPort` 改本机隧道端口时，
`tunnel` 和 `app` 两个动作必须传入一致的值。Android APP 入口仍为 `127.0.0.1:8002`。
直接在本机 WSL 使用 `bash catkin_ws/scripts/remote_sim.sh help` 也可操作，
对应连接、同步间隔和构建并行数使用脚本中的 `GRINDER_REMOTE_*` 环境变量。

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
