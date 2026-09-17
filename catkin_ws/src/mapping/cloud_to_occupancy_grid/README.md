# cloud_to_occupancy_grid

面向 MID-360S + Super-LIO 的固定内存二维增量建图节点。它使用 Super-LIO
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

配置文件是 `config/mid360.yaml`。150 m × 150 m、0.05 m 分辨率对应
3000 × 3000 个栅格；常驻 Log-Odds 和观测标记约 45 MB，发布或保存时会增加
约 9 MB 的短时快照，但不会因建图时间增长。

内部地图保持固定尺寸以避免导航代价地图反复重置。默认保存 PGM/YAML 时按
已观测栅格的包围盒裁剪，并在四周保留 `crop_padding: 1.0` 米；生成的 YAML
会同步修正地图原点。`publish_cropped_map: false` 保持实时 `/map` 的尺寸与原点
稳定，`save_cropped_map: true` 仅裁剪落盘文件。

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
