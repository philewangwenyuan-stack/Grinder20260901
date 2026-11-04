# cloud_to_occupancy_grid

面向 MID-360S + Super-LIO 的按需分块二维增量建图节点。它使用 Super-LIO
提供的位姿和当前帧配准点云，不再运行 GMapping，也不在内存中累计三维点云。

## 数据流

```text
/livox/lidar -> Super-LIO -> /lio/map_cloud（当前配准帧）
                         -> industrial loop backend
                            (Scan Context + FastVGICP + GTSAM iSAM2)
                         -> /lio/loop_cloud + map->odom->base_laser_link
                                      |
                                      v
                         cloud_to_occupancy_grid（增量 + 回环重投影）
                         -> /map
                         -> /tablet/map
                         -> PGM + YAML
```

虽然 Super-LIO 的历史话题名是 `/lio/map_cloud`，源码实际每次发布的是当前帧
点云变换到 `map` 后的结果，不是累计地图。不要把本节点连接到真正的累计 PCD。

## 编译

目标机执行：

```bash
cd /home/neardi/work/Grinder/catkin_ws
source /opt/ros/noetic/setup.bash
source /home/neardi/Livox/ws_livox/devel/setup.bash
PROFILE=mapping ./build_grinder_platform.sh
source devel/setup.bash
```

首次构建先运行 `bash ./install_super_lio_loop_deps.sh`。`mapping` profile 会检查
GTSAM 是否存在，不会静默构建一个看似工业版、实际只有轻量 ICP 的配置。

## 启动二维建图

先准备保存目录：

```bash
sudo mkdir -p /data/maps/current
sudo chown -R neardi:neardi /data/maps
```

终端 1 启动雷达：

```bash
source /opt/ros/noetic/setup.bash
source /home/neardi/Livox/ws_livox/devel/setup.bash
roslaunch livox_ros_driver2 msg_MID360s.launch
```

终端 2 启动 Super-LIO 和二维建图：

```bash
source /opt/ros/noetic/setup.bash
source /home/neardi/Livox/ws_livox/devel/setup.bash
source /home/neardi/work/Grinder/catkin_ws/devel/setup.bash
roslaunch cloud_to_occupancy_grid mid360_mapping.launch rviz:=true
```

该 launch 默认设置 `save_3d_map:=false`，避免 Super-LIO 的累计 PCD 内存持续增长。
如果这次作业还必须保存用于重定位的三维 PCD，可临时传入
`save_3d_map:=true`，但内存仍会随建图时间增长。

## 验证与保存

```bash
rostopic hz /lio/map_cloud
rostopic hz /map
rostopic echo -n 1 /map/info
rosservice call /cloud_to_occupancy_grid/save_map
ls -lh /data/maps/current/grinder_map.{pgm,yaml}
```

清空当前二维地图：

```bash
rosservice call /cloud_to_occupancy_grid/reset_map
```

配置文件是 `config/mid360.yaml`。内部按 64 × 64 格分块分配，每块概率与观测
标记共 20 KiB；只观测过空闲空间的块也保留。500 × 500 m、0.05 m 分辨率
现在只是可建图范围，不再预分配一亿格。世界坐标仍由原点和分辨率决定，扩展
有效边界不会移动旧障碍。`reset_map` 释放所有块并清空有效边界。

更新栅格时增量维护已观测包围盒，发布、保存和冻结共用此边界及
`crop_padding`；默认两个 `*_cropped_map` 都为 true。未扫描地图仅输出 1 个
未知格。快照只遍历已分配块，不再扫描全容量。显式关闭裁剪仍可输出全范围，
但同样受快照上限约束。实时地图的尺寸和 YAML 原点可能随扫描范围变化，
接收方必须按每次 OccupancyGrid 的 origin/resolution 解释坐标。

`max_grid_memory_mb` 默认 256，约束实时与回环暂存块的合计 payload；哈希表、
点云、ROS 队列及消息快照另外占用内存，因此它不是整个进程的 RSS 上限。
`max_snapshot_cells` 默认 25000000，约束每张连续快照（约 23.8 MiB 的 int8
数组），避免零散远点导致巨大的外接矩形。发布/保存可并发持有快照，ROS
序列化也会产生副本。两项参数可通过 mapper.launch 参数覆盖。

块分配失败或达到上限时，节点锁存错误、停止接受栅格更新并拒绝发布新图、
冻结和保存，须清空或重新扫描；不会静默丢块后报告保存成功。回环使用同样
的分块结构，成功后原子替换，失败保留原栅格并锁存错误。快照超限则拒绝该
快照及冻结/保存。`~storage_status` 每次主地图定时发布时上报状态、合计块字节、
上限和实时块数，错误同时写 ROS ERROR 日志。下游可能仍持有上一次地图，
运维需监控该状态和日志；本次未新增调度告警协议。

纯 C++ 回归可直接执行：
`g++ -std=c++14 -UNDEBUG test/test_sparse_grid.cpp -o /tmp/test_sparse_grid && /tmp/test_sparse_grid`。
覆盖与密集概率数组的一致性、块边界、回环交换、共享内存上限和清空释放。

PGM 使用 205 表示未知区域。保存的 YAML 使用
`map_server_free_threshold: 0.196`，该值与内部 Log-Odds 的
`free_threshold` 分开配置，避免 map_server 把灰色未知区域加载成白色可通行区。

工业回环被接受后，后端增加 `/super_lio_loop/revision`。二维节点通过
`/super_lio_loop/get_keyframe` 按需读取优化后的局部关键帧，在独立缓冲区重建
栅格，完成后一次性替换 `/map`；旧地图在重建期间仍可读取，不会留下双墙。
重投影期间会短暂停止接收实时点云，以保证地图版本一致。

## TF 和高度切片

- 点云和 TF 都按点云的原始时间戳处理；时间戳为零时该帧会被拒绝。
- 当前 Super-LIO 发布 `map -> odom -> base_laser_link`，所以默认
  `sensor_frame: base_laser_link`。
- 只有系统存在有效的 `map -> livox_frame` 时间同步 TF 时，才将
  `sensor_frame` 改为 `livox_frame`。
- `min_obstacle_height` 和 `max_obstacle_height` 在 `global_frame` 中判断。
  如果地面不在 `z=0` 附近，需要结合实际点云高度调整。

## 定位模式加载二维地图

```bash
rosrun map_server map_server /data/maps/current/grinder_map.yaml
```

PGM/YAML 只用于二维显示和导航；Super-LIO 重定位仍需单独的三维 PCD。


### 编辑草稿冻结

`~freeze_map`（`std_srvs/Trigger`）由模式管理器在 `/lio/freeze_mapping` 成功后调用。管理器先写入 `~freeze_target_stamp_ns`（十进制纳秒字符串）。节点等待该时间戳的点云和当前回环重投影，强制处理最后一帧（绕过频率限制），忽略后续点云/回环版本，并保持发布、保存同一份冻结栅格。未到齐或 TF 尚未就绪返回失败供有界重试；冻结后 `reset_map` 拒绝清空，重新扫描须创建新节点实例。

集成回归（先构建、source devel）：`python3 test/test_mapper_freeze_ros.py`。它使用独立 localhost:11379 master、合成点云，不接入设备；覆盖 500m 空图无预分配、跨块/负世界坐标、清空释放、冻结与 PGM 尺寸一致、快照及块内存超限拒绝保存、回环重建边界。
