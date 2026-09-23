# Grinder 研磨机项目说明

本文档用于说明研磨机项目的工程结构、核心链路、启动方式与联调要点。

## 项目目标

本项目用于实现研磨机的：

- 地图管理（建图、保存、删除、切图）
- 区域编辑（工作区/障碍区/擦除区/裁剪区）
- 路径规划与任务执行（含区域重复执行）
- 底盘与磨盘控制（手动/自动）
- 上位机与安卓通过 SL-LinkA 协议通信

## 目录结构

```text
Grinder/
├── catkin_ws/
│   ├── src/
│   │   ├── grinder_scheduler/         # 调度核心：协议处理、任务状态机、地图与路径流程
│   │   ├── grinder_chassis_driver/    # 底盘驱动：/cmd_vel->轮速、Modbus485读写、状态上报
│   │   ├── 2-dnavigation-package/     # move_base / teb / 全局路径相关
│   │   ├── slamware_ros_sdk/          # 雷达SDK：map/odom/图像与模式控制
│   │   └── ...
│   ├── start_grinder_stack.sh         # 一键启动脚本
│   └── build_grinder_platform.sh      # 构建脚本
├── third_party/
│   ├── sl_linka/                      # 协议、SDK、PyQt上位机
│   └── path_planner/                  # 自适应路径规划器与可视化
├── tools/
│   └── mediamtx/                      # 本地RTSP服务工具
└── README.md
```

## 核心链路

### 1. 协议链路（APP/上位机 <-> 调度）

- 调度监听 `8002`（SL-LinkA）
- 地图类：`MapPreview/MapEdit/MapCatalog/MapSave/MapDelete/MapMetrics`
- 任务类：`TaskConfig/TaskCommand/TaskStatus/TaskResult`
- 新增：`LiveMapCacheClearRequest/Response (0x051E/0x051F)`

协议说明见：
[sl-link.md](/home/chersxir/work/Grinder/third_party/sl_linka/sl-link.md)

### 2. 导航与任务链路

- 调度生成/维护全局路径，执行时按段发布目标点
- 导航输出 `cmd_vel`，底盘驱动按任务使能决定是否接收
- `/chassis/task_enable`：
  - `true`：底盘处理 `/cmd_vel`
  - `false`：底盘忽略 `/cmd_vel`

### 3. 底盘控制链路

- 底盘驱动订阅 `/cmd_vel`（默认）
- 转换为左右轮速度，做限幅/方向处理后通过 Modbus 写入
- 配置文件：
[chassis_driver.yaml](/home/chersxir/work/Grinder/catkin_ws/src/grinder_chassis_driver/config/chassis_driver.yaml)

## 关键配置文件

- 调度配置：
[scheduler.yaml](/home/chersxir/work/Grinder/catkin_ws/src/grinder_scheduler/config/scheduler.yaml)
- 底盘配置：
[chassis_driver.yaml](/home/chersxir/work/Grinder/catkin_ws/src/grinder_chassis_driver/config/chassis_driver.yaml)
- 协议定义：
[sl_link.proto](/home/chersxir/work/Grinder/third_party/sl_linka/proto/sl_link.proto)

常用调度参数（`scheduler.yaml`）：

- `path_goal_reach_dist`：到点判定距离
- `path_segment_timeout`：当前段超时重发时间
- `draw_region_label_on_preview`：区域名称是否绘制
- `draw_region_id_on_preview`：是否拼接显示区域ID
- `live_map_cache_clear_mode`：`all` / `memory_only`

Super-LIO 模式、地图根目录和启停超时位于
`catkin_ws/src/grinder_scheduler/config/super_lio.yaml`；该文件不包含设备凭据，可随代码部署。

## ROS Noetic 安装、工业级编译与正式启动

以下流程面向 Ubuntu 20.04（focal）+ ROS Noetic，支持 x86_64 和 RK3588/aarch64，
传感器为 Livox MID-360S。工业回环使用 GTSAM 4.2 iSAM2，并优先使用 fast_gicp
（FastVGICP）；二维地图由 cloud_to_occupancy_grid 增量维护，不使用 GMapping。

工作区路径示例：

~~~bash
export GRINDER_WS=/home/neardi/work/Grinder/catkin_ws
cd "$GRINDER_WS"
~~~

建图和定位互斥：

~~~text
建图：super_lio_node + super_lio_loop + cloud_to_occupancy_grid
定位：map_server + relocation_node
~~~

不要同时运行 super_lio_node 和 relocation_node。

### 1. 安装 ROS Noetic

ROS Noetic 的目标系统是 Ubuntu 20.04。在板端执行：

~~~bash
sudo apt update
sudo apt install -y curl gnupg2 lsb-release ca-certificates

curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  | gpg --dearmor \
  | sudo tee /usr/share/keyrings/ros-archive-keyring.gpg >/dev/null

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros/ubuntu focal main" \
  | sudo tee /etc/apt/sources.list.d/ros1.list >/dev/null

sudo apt update
sudo apt install -y ros-noetic-desktop-full
~~~

如果官方源不可访问，将 /etc/apt/sources.list.d/ros1.list 中的
packages.ros.org/ros/ubuntu 替换为可用镜像后再执行 sudo apt update。

安装 rosdep、Catkin 和基础工具：

~~~bash
sudo apt install -y \
  python3-rosdep python3-rosinstall python3-rosinstall-generator \
  python3-wstool python3-catkin-tools build-essential cmake git pkg-config

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update
~~~

每个新终端先加载：

~~~bash
source /opt/ros/noetic/setup.bash
source "$GRINDER_WS/devel/setup.bash" 2>/dev/null || true
~~~

工作区第一次成功编译后，再把这两行加入 ~/.bashrc。

### 2. 安装工作区依赖

~~~bash
cd "$GRINDER_WS"
source /opt/ros/noetic/setup.bash

rosdep install \
  --from-paths src \
  --ignore-src \
  --rosdistro noetic \
  --skip-keys="gtsam fast_gicp" \
  -r -y

sudo apt install -y \
  libboost-all-dev libeigen3-dev libmetis-dev libpcl-dev \
  libgoogle-glog-dev libyaml-cpp-dev
~~~

GTSAM 和 fast_gicp 是外部依赖，按下一节安装；不要用 ROS 2 包解决 ROS 1 依赖。

### 2.1 MST27 路径规划依赖

调度器默认使用：

~~~yaml
planner_script_path: third_party/path_planner/mst27/mst27.py
~~~

MST27 的 Python 规划器需要 NumPy、SciPy、Matplotlib 和 OpenCV；生产环境还应编译
同目录下的 C++/pybind11 加速内核。安装依赖：

~~~bash
sudo apt install -y \
  python3-numpy \
  python3-scipy \
  python3-matplotlib \
  python3-opencv \
  python3-dev \
  pybind11-dev
~~~

设置仓库根目录，确保调度器能解析相对的 planner_script_path：

~~~bash
export GRINDER_BASE_DIR=/home/neardi/work/Grinder
~~~

编译 MST27 C++ 内核（Release）：

MST27 扩展当前使用独立 CMake 工程；不要假设仅执行 catkin_make 就一定会生成该
Python 扩展，正式部署应显式执行下面的 CMake 命令。

~~~bash
cd "$GRINDER_BASE_DIR/third_party/path_planner/mst27"
cmake \
  -S . \
  -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DPython3_EXECUTABLE="$(which python3)"
cmake --build build --parallel 2
~~~

扩展模块应生成在 mst27.py 同目录。验证 Python 依赖和 C++ 内核：

~~~bash
python3 -c "import numpy, scipy, matplotlib, cv2; print('MST27 Python dependencies: OK')"
python3 -c \
  "import sys; sys.path.insert(0, '$GRINDER_BASE_DIR/third_party/path_planner/mst27'); import _mst27_cpp; print('MST27 C++ backend: OK')"
~~~

调度器启动日志出现 [C++后端] 使用C++规划内核 表示使用加速内核；如果出现
Python fallback，路径规划仍可运行，但速度会明显下降。若工作区从 x86_64 复制到
RK3588，先检查扩展架构：

~~~bash
file "$GRINDER_BASE_DIR/third_party/path_planner/mst27/"_mst27_cpp*.so
~~~

架构不匹配时，只删除该目录下的旧 _mst27_cpp*.so，再在目标板重新编译，不能复用
x86_64 的 .so。

### 3. 安装工业回环依赖

| 依赖 | 作用 | 要求 |
| --- | --- | --- |
| GTSAM 4.2.2 | Pose3 图优化、iSAM2、协方差 | 必需 |
| fast_gicp | FastVGICP 点云配准 | 推荐 |
| ROS Noetic PCL 1.10 | 无 fast_gicp 时的 PCL-GICP 降级路径 | 已由 ROS/apt 提供 |

在线安装（推荐）：

~~~bash
cd "$GRINDER_WS"
JOBS=2 bash ./install_super_lio_loop_deps.sh
~~~

RK3588 建议 JOBS=2。脚本固定 GTSAM 4.2.2 和已验证的 fast_gicp 提交，并安装到
/usr/local，不替换 ROS 自带的 PCL。

GitHub 网络受限时，在电脑上下载以下源码后复制到板端：

- GTSAM 4.2.2：https://github.com/borglab/gtsam/tree/4.2.2
- fast_gicp：https://github.com/koide3/fast_gicp

~~~bash
scp gtsam-4.2.2.tar.gz neardi@<板端IP>:/home/neardi/
~~~

板端编译 GTSAM：

~~~bash
mkdir -p /home/neardi/deps
tar -xzf /home/neardi/gtsam-4.2.2.tar.gz -C /home/neardi/deps

cmake \
  -S /home/neardi/deps/gtsam-4.2.2 \
  -B /home/neardi/deps/gtsam-build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/usr/local \
  -DGTSAM_BUILD_TESTS=OFF \
  -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
  -DGTSAM_BUILD_UNSTABLE=OFF \
  -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
  -DGTSAM_USE_SYSTEM_EIGEN=ON

cmake --build /home/neardi/deps/gtsam-build --parallel 2
sudo cmake --install /home/neardi/deps/gtsam-build
sudo ldconfig
find /usr/local -name GTSAMConfig.cmake
~~~

只安装 GTSAM 也会生成工业图优化节点；缺少 fast_gicp 时会明确降级到较慢的
PCL-GICP，不会悄悄切换成轻量回环。

### 4. 正式 Release 编译

先编译 ROS 1 Livox 驱动。板端驱动包应位于
src/drivers/ws_livox/src/livox_ros_driver2；不要混用 ROS 2 的 ament_cmake_auto 工程。

~~~bash
cd "$GRINDER_WS"
source /opt/ros/noetic/setup.bash
catkin_make \
  -DCATKIN_WHITELIST_PACKAGES=livox_ros_driver2 \
  -DCMAKE_BUILD_TYPE=Release \
  -j2 -l2
~~~

再强制编译工业回环：

~~~bash
catkin_make \
  -DCATKIN_WHITELIST_PACKAGES="super_lio;super_lio_loop;cloud_to_occupancy_grid" \
  -DCMAKE_BUILD_TYPE=Release \
  -DSUPER_LIO_REQUIRE_INDUSTRIAL_BACKEND=ON \
  -DGTSAM_DIR=/usr/local/lib/cmake/GTSAM \
  -j2 -l2
~~~

SUPER_LIO_REQUIRE_INDUSTRIAL_BACKEND=ON 会在 GTSAM 缺失时直接失败，避免误以为已经
编译成工业版本。确认：

~~~bash
source "$GRINDER_WS/devel/setup.bash"
rospack find livox_ros_driver2
ls -lh "$GRINDER_WS/devel/lib/super_lio_loop/"
~~~

工业版本必须同时有：

~~~text
super_lio_loop_node
super_lio_graph_backend_node
~~~

也可以使用平台脚本：

~~~bash
cd "$GRINDER_WS"
CATKIN_JOBS=2 CATKIN_LOAD=2 PROFILE=mapping ./build_grinder_platform.sh
~~~

PROFILE=mapping 已自动传入工业后端强制选项。其它 profile：

~~~bash
PROFILE=runtime ./build_grinder_platform.sh   # 底盘、Aurora、调度器
PROFILE=nav     ./build_grinder_platform.sh   # move_base、RPP/TEB
PROFILE=full    ./build_grinder_platform.sh   # 平台主包
~~~

如果之前只编译过某个包，先检查旧白名单：

~~~bash
grep CATKIN_WHITELIST_PACKAGES "$GRINDER_WS/build/CMakeCache.txt"
~~~

必要时执行一次：

~~~bash
catkin_make -DCATKIN_WHITELIST_PACKAGES="" -DCMAKE_BUILD_TYPE=Release -j2 -l2
~~~

不要把 -DCMAKE_BUILD_TYPE=Release 粘贴到 CATKIN_WHITELIST_PACKAGES 的引号内部。

### 5. MID-360S 网络和驱动验证

雷达直连网口和路由器网口不要使用相同网段，否则 Linux 可能把雷达数据发到错误网卡：

~~~bash
ip -br addr
ip route
ip route get <雷达IP>
~~~

最后一条必须显示连接雷达的网卡（例如 eth0），不能显示 lo 或路由器网卡。
host_ip、雷达 ip 和端口必须与 livox_ros_driver2/config/MID360s_config.json 一致。

~~~bash
source /opt/ros/noetic/setup.bash
source "$GRINDER_WS/devel/setup.bash"
roslaunch livox_ros_driver2 msg_MID360s.launch
rostopic hz /livox/lidar
rostopic hz /livox/imu
~~~

不需要录制 LVX 数据时：

~~~bash
roslaunch livox_ros_driver2 msg_MID360s.launch \
  enable_lidar_bag:=false \
  enable_imu_bag:=false
~~~

Release 编译不会关闭 ROS 日志；上面的参数只关闭雷达/IMU bag 录制。

### 6. 工业级手工建图和定位

驱动正常后，另一个终端启动工业建图：

~~~bash
source /opt/ros/noetic/setup.bash
source "$GRINDER_WS/devel/setup.bash"
roslaunch cloud_to_occupancy_grid mid360_mapping.launch \
  rviz:=true save_3d_map:=true loop_closure:=true loop_backend:=industrial
~~~

检查：

~~~bash
rosnode list | grep -E 'super_lio_node|super_lio_loop|cloud_to_occupancy_grid'
rostopic hz /lio/odom
rostopic hz /lio/loop_cloud
rostopic hz /map
rostopic echo -n 1 /super_lio_loop/status
rosrun tf tf_echo map base_link
~~~

应看到 /super_lio_node、/super_lio_loop 和 /cloud_to_occupancy_grid。
保存 PCD、PGM 和 YAML：

~~~bash
mkdir -p /home/neardi/work/Grinder/maps
rosservice call /lio/get_3dmap "cmd_id: 1
enable_filter: true
map_dir: '/home/neardi/work/Grinder/maps'
map_name: 'grinder_map'"
rosservice call /cloud_to_occupancy_grid/save_map
~~~

定位模式不启动回环，而是加载二维地图和三维 PCD：

~~~bash
rosrun map_server map_server /home/neardi/work/Grinder/maps/grinder_map.yaml
roslaunch super_lio relocation.launch \
  rviz:=true map_dir:=/home/neardi/work/Grinder/maps \
  map_name:=loc_grinder_map.pcd \
  base_to_laser_x:=0.0 base_to_laser_y:=0.0 base_to_laser_z:=0.0 \
  base_to_laser_roll:=0.0 base_to_laser_pitch:=0.0 base_to_laser_yaw:=0.0
~~~

### 7. 正式环境一键启动

start_grinder_super_lio_base.sh 启动 Livox、底盘（可选）、RPP move_base（可选）、
调度器和 super_lio_mode_manager，但不会自动进入建图或定位：

~~~bash
cd "$GRINDER_WS"
GRINDER_BASE_DIR=/home/neardi/work/Grinder \
LIVOX_SETUP="$GRINDER_WS/devel/setup.bash" \
START_LIVOX=1 START_CHASSIS=1 START_NAV=1 \
./start_grinder_super_lio_base.sh
~~~

如果 Livox 在独立工作区：

~~~bash
LIVOX_SETUP=/home/neardi/Livox/ws_livox/devel/setup.bash \
./start_grinder_super_lio_base.sh
~~~

工业配置应保持以下值（文件：catkin_ws/src/grinder_scheduler/config/super_lio.yaml
和 scheduler.yaml）：

~~~yaml
super_lio_loop_closure: true
super_lio_autosave: false
~~~

修改 YAML 不需要重新编译，但必须重启调度器。启动后切换地图模式：

~~~bash
rosservice call /super_lio_mode/get_status
rosservice call /super_lio_mode/start_mapping "{}"
rosparam get /super_lio_mode_manager/super_lio_loop_closure
rosnode list | grep -E 'super_lio_loop|super_lio_node|relocation_node|map_server'
~~~

### 8. 常见故障

| 现象 | 原因 | 处理 |
| --- | --- | --- |
| GTSAM 4.2 not found | GTSAM 未安装或 CMake 找不到 | 检查 GTSAMConfig.cmake，重跑工业编译并传入 -DGTSAM_DIR=/usr/local/lib/cmake/GTSAM |
| 只有 super_lio_loop_node | 只生成了轻量回环 | 安装 GTSAM 后用 SUPER_LIO_REQUIRE_INDUSTRIAL_BACKEND=ON 重编译 |
| unknown node /super_lio_loop | 未进入建图或工业节点启动失败 | 检查 rosnode list、回环日志和工业可执行文件 |
| ament_cmake_auto 找不到 | 混用了 ROS 2 Livox 工程 | 使用 ROS 1 Noetic 的 livox_ros_driver2，不要 source ROS 2 |
| /livox/lidar 没有数据 | 网卡、雷达 IP 或 host_ip 不匹配 | 检查 ip route get <雷达IP> 必须走雷达网卡 |
| 建图和定位冲突 | 同时运行两种模式 | 先调用 /super_lio_mode/stop，再切换模式 |

## 手工操作参考（完成上面的安装和编译后）

MID-360S + Super-LIO 的固定内存实时二维建图见
`catkin_ws/src/2-dnavigation-package/cloud_to_occupancy_grid/README.md`。该流程直接使用 LIO 位姿和
当前配准点云生成 `/map`，不再叠加 GMapping。

### 1. 已完成编译后的手工操作

```bash
cd ~/work/Grinder/catkin_ws
# 工业正式编译请按上文第 4 节执行；这里仅加载已生成的工作区
source devel/setup.bash
```

### 2. 每个终端的基础环境

板端每开一个新终端都先执行：

```bash
source /opt/ros/noetic/setup.bash
source /home/neardi/work/Grinder/catkin_ws/devel/setup.bash
```

以下命令假定工作区位于 `/home/neardi/work/Grinder/catkin_ws`。建图与重定位
是互斥模式，不要同时运行 `super_lio_node` 和 `relocation_node`。

### 3. 启动 MID-360S 驱动

```bash
roslaunch livox_ros_driver2 msg_MID360s.launch
```

验证：

```bash
rostopic hz /livox/lidar
rostopic hz /livox/imu
```

### 4. 建图模式（手工调试）

启动 Super-LIO、回环后端和固定尺寸二维栅格地图：

```bash
roslaunch cloud_to_occupancy_grid mid360_mapping.launch \
  rviz:=true \
  save_3d_map:=true \
  loop_closure:=true \
  loop_backend:=industrial
```
```bash
roslaunch cloud_to_occupancy_grid mid360_mapping.launch \
  rviz:=true \
  save_3d_map:=true \
  loop_closure:=false
```

Super-LIO 只在位移、旋转、时间间隔和点数满足关键帧准入条件时向三维地图
追加点云；设备静止时不会逐帧重复累计。关键帧参数位于
`Super-LIO/src/super_lio/config/livox_360.yaml`。

验证建图输出：

```bash
rostopic hz /lio/odom
rostopic hz /lio/map_cloud
rostopic hz /map
rosrun tf tf_echo map base_link
```

保存三维定位地图：

```bash
mkdir -p /home/neardi/maps
rosservice call /lio/get_3dmap \
  "cmd_id: 1
enable_filter: true
map_dir: '/home/neardi/work/Grinder/maps'
map_name: 'grinder_map'
```

正常生成：

```text
/home/neardi/maps/loc_grinder_map.pcd
/home/neardi/maps/plan_grinder_map.pcd
```

保存二维导航地图：

```bash
rosservice call /cloud_to_occupancy_grid/save_map
```

二维地图的实际输出目录和文件名由
`cloud_to_occupancy_grid/config/mid360.yaml` 配置。确认 PCD、PGM 和 YAML 均已
成功生成后再停止建图节点：

```bash
rosnode kill /super_lio_node
rosnode kill /super_lio_loop
rosnode kill /cloud_to_occupancy_grid
```

Livox 驱动可以继续运行，供重定位使用。

### 5. 旧地图重定位模式（手工调试）

先启动二维静态地图。路径必须替换为本次保存的 YAML：

```bash
rosrun map_server map_server /home/neardi/work/Grinder/maps/grinder_map.yaml
```

再启动固定三维地图重定位：

```bash
roslaunch super_lio relocation.launch \
  rviz:=true \
  map_dir:=/home/neardi/work/Grinder/maps \
  map_name:=loc_grinder_map.pcd \
  base_to_laser_x:=0.0 \
  base_to_laser_y:=0.0 \
  base_to_laser_z:=0.0 \
  base_to_laser_roll:=0.0 \
  base_to_laser_pitch:=0.0 \
  base_to_laser_yaw:=0.0
```

`base_to_laser_*` 必须按实际安装外参填写。重定位模式由 launch 强制设置
`/lio/map/save_map=false`，不累计保存点云；固定地图定位还应保持
`/lio/relocation/update_map=false`。

首次启动后可在 RViz 使用 `2D Pose Estimate` 发布 `/initialpose` 做人工联调；正式 APP
链路通过 `RadarRelocalizationRequest (0x052A)` 一并传入当前 `map_id/map_revision` 和
`x/y/heading_deg`。调度器只在身份匹配后调用模式管理器发布 `/initialpose`。验证：

```bash
rostopic hz /lio/odom
rostopic hz /scan
rostopic echo -n 1 /map/info
rosrun tf tf_echo map base_link
```

### 6. 启动 move_base

旧地图和重定位均正常后启动导航。RPP 使用底盘反馈 `/odom_wheel`：

```bash
roslaunch teb_local_planner_tutorials robot_diff_drive.launch \
  local_planner:=rpp
```

如需 TEB：

```bash
roslaunch teb_local_planner_tutorials robot_diff_drive.launch \
  local_planner:=teb
```

验证代价地图和速度输出：

```bash
rostopic hz /move_base/global_costmap/costmap
rostopic hz /move_base/local_costmap/costmap
rostopic hz /cmd_vel
```

### 7. 启动底盘驱动

启动前检查 `grinder_chassis_driver/config/chassis_driver.yaml` 中的串口、波特率、
轮距、轮径和方向参数。真实底盘启动会访问 Modbus 硬件：

```bash
ls -l /dev/ttyS0
roslaunch grinder_chassis_driver chassis_driver.launch
```

默认 `task_enable_default=false`，节点启动时先下发零速。验证：

```bash
rosnode info /chassis_driver
rostopic hz /odom_wheel
rostopic echo /chassis/task_enable
```

### 8. 启动调度器和 Super-LIO 模式管理器

使用 Super-LIO 时，显式把调度定位输入设为 `/lio/odom`。move_base 已经单独
启动时，只启动 `scheduler.launch`，不要再启动一个包含导航的系统 launch：

```bash
roslaunch grinder_scheduler scheduler.launch \
  odom_topic:=/lio/odom \
  map_topic:=/map \
  super_lio_map_root:=/home/neardi/work/Grinder/maps
```

验证：

```bash
rosnode info /grinder_scheduler
rosnode info /super_lio_mode_manager
rostopic info /grinder/navigation/active_segment_plan
rostopic echo /chassis/task_enable
ss -lntp | grep 8002
```

完整导航链路为：

```text
grinder_scheduler
  -> /grinder/navigation/active_segment_plan
  -> move_base
  -> /cmd_vel
  -> grinder_chassis_driver
  -> 底盘
```

### 9. 一键启动 Super-LIO 常驻服务

```bash
cd /home/neardi/work/Grinder/catkin_ws
chmod +x start_grinder_super_lio_base.sh
./start_grinder_super_lio_base.sh
```

该脚本启动 Livox、底盘、RPP move_base、调度器和
`super_lio_mode_manager`。启动后地图模式为 `IDLE`，不会自动启动建图或重定位。

正式运行时由 APP 的 `MapModeRequest` 和 `MapSaveRequest` 控制地图生命周期：

```text
MAP_MODE_MAPPING enabled=true
  -> 启动 Super-LIO + cloud_to_occupancy_grid

MapSaveRequest map_id="" 或 map_id="LIVE_MAP"
  -> 保存 PCD 与 PGM/YAML
  -> LOWER 生成唯一正式 map_id，并在响应中返回
  -> 停止并复查建图节点
  -> 返回 map_revision、mapping_stopped、localization_started=false

MAP_MODE_LOCALIZATION enabled=true
  -> 返回 STARTING，再进入 LOCALIZING，等待带 map_id/map_revision 的初始位姿
  -> APP 轮询 0x052D；只有 lifecycle_state=READY 才允许开始任务
```

APP 必须在 `MAPPING` 仍处于活动状态时发送 `MapSaveRequest`，不要先发送
`MAP_MODE_LOCALIZATION`。`LIVE_MAP` 只是实时建图源的保留 ID，不会作为资产目录名；
保存成功后响应中的新 `map_id/map_revision` 才是后续地图选择和重定位使用的身份。地图
保存成功不表示定位已经启动或已达到 `READY`。

每张地图保存为独立资产包：

```text
/home/neardi/work/Grinder/maps/<map_id>/
├── loc_map.pcd
├── plan_map.pcd
├── map.yaml
├── map.pgm
└── map_info.json
```

可直接检查模式管理器：

```bash
rosservice call /super_lio_mode/get_status
rosservice call /super_lio_mode/start_mapping
rosservice call /super_lio_mode/stop
```

手工调用保存服务仅用于联调，`map_id` 不得与已有资产包重复：

```bash
rosservice call /super_lio_mode/save_map "map_id: 'test_001'
map_name: '测试地图'"
```

保存任一环节失败时建图保持运行，可以再次保存。地图已保存但重定位启动失败时，
资产包仍保留，底盘任务使能保持关闭。

### 10. 单独启动调度

```bash
cd ~/work/Grinder/catkin_ws
source devel/setup.bash
roslaunch grinder_scheduler scheduler.launch \
  odom_topic:=/lio/odom \
  map_topic:=/map \
  super_lio_map_root:=/home/neardi/work/Grinder/maps
```

### 11. 上位机启动（PyQt）

```bash
cd ~/work/Grinder
python3 third_party/sl_linka/sdk/python/tools/sl_linka_pyqt_debugger.py
```

连接参数：

- Host：调度程序IP（本机可 `127.0.0.1`）
- Port：`8002`

## 协议库生成

当 `sl_link.proto` 变更后，需重新生成 SDK：

```bash
cd ~/work/Grinder
bash third_party/sl_linka/scripts/generate_proto.sh all
```

若仅需 Python/Android：

```bash
bash third_party/sl_linka/scripts/generate_proto.sh python
bash third_party/sl_linka/scripts/generate_proto.sh kotlin
```

## 常用联调命令

查看调度订阅发布：

```bash
rosnode info /grinder_scheduler
```

查看导航目标发布：

```bash
rostopic echo /move_base_simple/goal
```

查看底盘任务使能：

```bash
rostopic echo /chassis/task_enable
```

查看底盘速度输入：

```bash
rostopic hz /cmd_vel
```

## 说明

- 地图、任务、区域均建议基于 `map_id` 进行绑定与操作。
- 清缓存指令 `0x051E` 为无参请求，具体清理范围由调度配置决定。
- 本项目是在线联调工程，修改后请重启对应节点验证参数是否已生效。
