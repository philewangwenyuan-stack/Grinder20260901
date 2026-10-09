# MID-360S 建图、二维地图生成与重定位流程

本文适用于以下环境：

- Ubuntu 20.04 / ROS Noetic
- RK3588 主机
- Livox MID-360S
- `livox_ros_driver2`
- `Super-LIO`
- `pcd2pgm`

文中的示例约定：

| 项目 | 配置 |
| --- | --- |
| 管理网口 | `eth1 = 192.168.1.2/24` |
| 雷达网口 | `eth0 = 192.168.1.99/24` |
| MID-360S | `192.168.1.199` |
| Livox 工作区 | `/home/neardi/Livox/ws_livox` |
| Grinder 工作区 | `/home/neardi/work/Grinder/catkin_ws` |
| 地图输出目录 | `/home/neardi/maps` |

## 1. 总体流程图

```mermaid
flowchart TD
    A[MID-360S<br/>192.168.1.199] -->|点云和 IMU| B[livox_ros_driver2]
    B --> C[/livox/lidar]
    B --> D[/livox/imu]

    C --> E[Super-LIO 建图模式]
    D --> E
    E --> F[/lio/odom]
    E --> G[/lio/map_cloud]
    E --> H[/scan]

    E -->|调用 /lio/get_3dmap| I[loc_grinder_map.pcd<br/>定位地图]
    E -->|调用 /lio/get_3dmap| J[plan_grinder_map.pcd<br/>规划点云]

    J -->|调用 /pcd2pgm/get_pgm| K[/map OccupancyGrid]
    K -->|map_saver| L[grinder_map.pgm]
    K -->|map_saver| M[grinder_map.yaml]

    I --> N[Super-LIO 重定位模式]
    C --> N
    D --> N
    N -->|等待初始猜测| O[/initialpose]
    O -->|NDT + ICP| P[/lio/odom 定位输出]

    L --> Q[map_server]
    M --> Q
    Q --> R[/map 二维导航地图]
```

地图文件的用途不同：

```text
loc_grinder_map.pcd   -> Super-LIO 三维重定位
plan_grinder_map.pcd  -> 投影生成二维地图
grinder_map.pgm/yaml  -> map_server 和二维导航
```

## 2. 配置雷达网络

### 2.1 配置 eth0

`eth0` 专门连接 MID-360S：

```bash
sudo nmcli connection modify ETH0 \
  ipv4.method manual \
  ipv4.addresses "192.168.1.99/24" \
  ipv4.gateway "" \
  ipv4.never-default yes \
  ipv4.ignore-auto-routes yes \
  ipv4.ignore-auto-dns yes
```

由于 `eth0` 和 `eth1` 当前处于同一网段，需要给雷达添加精确路由：

```bash
sudo nmcli connection modify ETH0 ipv4.routes "192.168.1.199/32"
sudo nmcli device reapply eth0
```

验证：

```bash
ip route get 192.168.1.199
ping -I eth0 -c 4 192.168.1.199
```

正确路由应包含：

```text
192.168.1.199 dev eth0 src 192.168.1.99
```

### 2.2 配置 Livox 驱动

`MID360s_config.json` 的关键字段：

```json
{
  "host_net_info": [
    {
      "host_ip": "192.168.1.99",
      "cmd_data_port": 56101,
      "push_msg_port": 56201,
      "point_data_port": 56301,
      "imu_data_port": 56401,
      "log_data_port": 56501
    }
  ],
  "lidar_configs": [
    {
      "ip": "192.168.1.199",
      "pcl_data_type": 1,
      "pattern_mode": 0
    }
  ]
}
```

`host_ip` 是 RK3588 的雷达网口地址，`lidar_configs.ip` 是雷达地址，两者不能相同。

## 3. 编译

安装基础依赖：

```bash
sudo apt install -y libgoogle-glog-dev libgflags-dev libtbb-dev
```

编译工作区中的相关包：

```bash
cd /home/neardi/work/Grinder/catkin_ws
source /opt/ros/noetic/setup.bash
source /home/neardi/Livox/ws_livox/devel/setup.bash
catkin_make -DCATKIN_WHITELIST_PACKAGES="basic;super_lio;pcd2pgm"
source devel/setup.bash
```

不要在 `catkin_ws/src` 中运行 `catkin_make`；必须在 `catkin_ws` 根目录运行。

## 4. 启动 MID-360S 驱动

终端 1：

```bash
source /opt/ros/noetic/setup.bash
source /home/neardi/Livox/ws_livox/devel/setup.bash
roslaunch livox_ros_driver2 msg_MID360s.launch
```

终端 2 验证：

```bash
source /opt/ros/noetic/setup.bash
source /home/neardi/Livox/ws_livox/devel/setup.bash
rostopic hz /livox/lidar
rostopic hz /livox/imu
```

## 5. Super-LIO 建图

终端 3：

```bash
source /opt/ros/noetic/setup.bash
source /home/neardi/Livox/ws_livox/devel/setup.bash
source /home/neardi/work/Grinder/catkin_ws/devel/setup.bash
roslaunch super_lio Livox_mid360.launch rviz:=false
```

正常日志应包含：

```text
KF init success
Map init success
Running...
```

检查输出：

```bash
rostopic hz /lio/odom
rostopic hz /lio/map_cloud
rostopic hz /scan
```

### 5.1 LaserScan 时间戳要求

`/scan` 必须继承原始 Livox 消息的采样时间：

```cpp
pub_scan_body(cloud_scan, msg->header.stamp);
```

发布时：

```cpp
scan.header.stamp = stamp;
```

验证：

```bash
rostopic echo -n 1 /scan/header
```

`secs` 和 `nsecs` 不能同时为零。

## 6. 保存三维 PCD

先创建输出目录：

```bash
mkdir -p /home/neardi/maps
```

调用保存服务：

```bash
rosservice call /lio/get_3dmap \
  "cmd_id: 1
enable_filter: true
map_dir: '/home/neardi/maps'
map_name: 'grinder_map'"
```

服务返回：

```text
success: True
map_data: []
```

`map_data: []` 是当前实现的正常行为；地图通过副作用写入磁盘。

应生成：

```text
/home/neardi/maps/loc_grinder_map.pcd
/home/neardi/maps/plan_grinder_map.pcd
```

必须检查文件不仅存在，而且包含点：

```bash
ls -lh /home/neardi/maps/*.pcd
grep -a -E '^(WIDTH|HEIGHT|POINTS|DATA)' /home/neardi/maps/loc_grinder_map.pcd | head -n 4
grep -a -E '^(WIDTH|HEIGHT|POINTS|DATA)' /home/neardi/maps/plan_grinder_map.pcd | head -n 4
```

`POINTS` 必须大于零。当前服务即使内部地图为空，也可能返回 `success: True`。

## 7. 由 PCD 生成 PGM/YAML

### 7.1 启动 pcd2pgm

终端 4：

```bash
source /opt/ros/noetic/setup.bash
source /home/neardi/work/Grinder/catkin_ws/devel/setup.bash
roslaunch pcd2pgm binary_map.launch file_directory:=/home/neardi/maps/ file_name:=plan_grinder_map min_height:=0.2 max_height:=0.8 radius:=0.5 neighbor_count:=10 resolution:=0.05
```

当前 `pcd2pgm` 启动后只发布默认空地图，不会自动加载 PCD。必须调用服务触发转换。

### 7.2 触发转换

终端 5：

```bash
source /opt/ros/noetic/setup.bash
source /home/neardi/work/Grinder/catkin_ws/devel/setup.bash
rosservice call /pcd2pgm/get_pgm "map_path: '/home/neardi/maps' map_name: 'plan_grinder_map' create_dir: false"
```

`pcd2pgm` 终端应显示：

```text
初始点云数据点数：...
直通滤波后点云数据点数：...
半径滤波后点云数据点数：...
pgm size = ...
```

检查栅格地图：

```bash
rostopic echo -n 1 /map/info
```

必须满足：

```text
resolution > 0
width > 0
height > 0
```

若过滤后点数为零，扩大高度范围并降低邻域阈值：

```bash
rosnode kill /pcd2pgm
roslaunch pcd2pgm binary_map.launch file_directory:=/home/neardi/maps/ file_name:=plan_grinder_map min_height:=-1.0 max_height:=2.0 radius:=1.0 neighbor_count:=2 resolution:=0.05
```

之后再次调用 `/pcd2pgm/get_pgm`。

### 7.3 写出 PGM/YAML

只有 `/map/info` 非空后才能保存：

```bash
rosrun map_server map_saver -f /home/neardi/maps/grinder_map map:=/map
```

应生成：

```text
/home/neardi/maps/grinder_map.pgm
/home/neardi/maps/grinder_map.yaml
```

确认：

```bash
ls -lh /home/neardi/maps/grinder_map.*
```

## 8. 进入重定位模式

### 8.1 准备定位地图

当前 `relocation.yaml` 默认加载：

```text
$(rospack find super_lio)/map/map.pcd
```

复制定位地图：

```bash
source /opt/ros/noetic/setup.bash
source /home/neardi/work/Grinder/catkin_ws/devel/setup.bash
mkdir -p "$(rospack find super_lio)/map"
cp -v /home/neardi/maps/loc_grinder_map.pcd "$(rospack find super_lio)/map/map.pcd"
```

再次检查点数：

```bash
grep -a -E '^(WIDTH|HEIGHT|POINTS|DATA)' "$(rospack find super_lio)/map/map.pcd" | head -n 4
```

### 8.2 停止建图节点

不要同时运行建图节点和重定位节点：

```bash
rosnode kill /super_lio_node
rosnode kill /pcd2pgm
```

Livox 驱动保持运行。

### 8.3 启动重定位

```bash
source /opt/ros/noetic/setup.bash
source /home/neardi/Livox/ws_livox/devel/setup.bash
source /home/neardi/work/Grinder/catkin_ws/devel/setup.bash
roslaunch super_lio relocation.launch rviz:=false
```

启动日志必须包含：

```text
Load map success
Map size: 大于0
```

### 8.4 发布初始位姿

当前实现必须收到一次 `/initialpose` 才会启动 NDT/ICP。仅在 YAML 中填写 `init_pose` 不会自动设置初始化触发标志。

推荐在 RViz 中：

1. 将 `Fixed Frame` 设置为 `map`。
2. 显示 `/lio/global_map`。
3. 点击 `2D Pose Estimate`。
4. 在机器人近似位置点击并拖动朝向。

如果机器人位于建图原点，可用命令发布：

```bash
rostopic pub -1 /initialpose geometry_msgs/PoseWithCovarianceStamped "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"
```

成功日志应包含：

```text
GET Initial guess
INIT start... obs_data size: 大于0 target size: 大于0
Global ICP Converged Succeed
```

如果出现：

```text
target size: 0
```

说明定位 PCD 没有加载成功或 `POINTS 0`，必须返回第 8.1 节检查地图。

## 9. 发布二维导航地图

停止 `pcd2pgm` 后启动 `map_server`，避免两个节点同时发布 `/map`：

```bash
source /opt/ros/noetic/setup.bash
rosrun map_server map_server /home/neardi/maps/grinder_map.yaml
```

验证：

```bash
rostopic echo -n 1 /map/info
rostopic hz /lio/odom
```

RViz 推荐配置：

| 显示项 | 配置 |
| --- | --- |
| Fixed Frame | `map` |
| Map | `/map` |
| PointCloud2 | `/lio/global_map` |
| Odometry | `/lio/odom` |
| LaserScan | `/scan` |

## 10. 常见问题

### `Failed to GET Initial guess`

尚未收到 `/initialpose`。使用 RViz 的 `2D Pose Estimate` 或命令行发布初始位姿。

### `target size: 0`

历史定位 PCD 为空或路径错误。检查：

```bash
ls -lh "$(rospack find super_lio)/map/map.pcd"
grep -a '^POINTS' "$(rospack find super_lio)/map/map.pcd"
```

### `/map/info` 中 width 和 height 为 0

这是 `pcd2pgm` 的默认空消息。调用 `/pcd2pgm/get_pgm`；若仍为空，查看过滤后的点数并调整高度范围。

### RViz 显示 `No TF data`

只看原始点云时可以使用 `livox_frame`。运行 Super-LIO 时将 Fixed Frame 设置为 `map`。

### PGM 不会随建图实时更新

离线的 `pcd2pgm` 仍然只在服务触发时投影 PCD。仓库现已提供
`cloud_to_occupancy_grid`，可直接消费 Super-LIO 的当前配准帧并增量发布
`/map`：

```bash
roslaunch cloud_to_occupancy_grid mid360_mapping.launch rviz:=true
```

该模式默认关闭 Super-LIO 的累计三维 PCD 缓存，以固定内存持续构建二维地图。
详细参数、保存服务和内存说明见
`catkin_ws/src/cloud_to_occupancy_grid/README.md`。

### gmapping 地图不更新

Super-LIO 当前把运动发布在 `map -> odom`，而 `odom -> base_laser_link` 是固定变换，与 gmapping 的标准里程计假设不同。生产流程建议使用保存后的 `plan_*.pcd` 生成 PGM，而不是叠加第二套 SLAM。

## 11. 最终验收清单

- [ ] `ping -I eth0 192.168.1.199` 正常。
- [ ] `/livox/lidar` 和 `/livox/imu` 有稳定频率。
- [ ] `/scan/header.stamp` 非零。
- [ ] 建图时 `/lio/odom` 和 `/lio/map_cloud` 正常更新。
- [ ] 实时二维建图时 `/map/info` 为 `3000 x 3000` 且 `/map` 持续更新。
- [ ] `loc_grinder_map.pcd` 与 `plan_grinder_map.pcd` 的 `POINTS` 大于零。
- [ ] `/map/info` 的 `width`、`height` 大于零。
- [ ] `grinder_map.pgm` 与 `grinder_map.yaml` 已生成。
- [ ] 重定位启动日志中的 `Map size` 大于零。
- [ ] 发布 `/initialpose` 后，日志中的 `target size` 大于零。
- [ ] NDT/ICP 收敛后 `/lio/odom` 随设备运动更新。
