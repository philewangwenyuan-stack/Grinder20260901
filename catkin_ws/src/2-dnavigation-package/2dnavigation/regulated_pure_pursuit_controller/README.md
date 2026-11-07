# regulated_pure_pursuit_controller
Regulated Pure Pursuit Controller for ROS. Applicable for both [move_base](http://wiki.ros.org/move_base) and [move_base_flex](http://wiki.ros.org/move_base_flex). Tested with ROS-Noetic.

## Update 
Implement [vector_pure_pursuit](https://github.com/blackcoffeerobotics/vector_pursuit_controller). To use it, set the parameter `use_vector_pure_pursuit` to `true` and follow it's [Configuration Guide](https://docs.ros.org/en/humble/p/vector_pursuit_controller/)

# Installation
* Install move_base_flex
    ```sh
    sudo apt install ros-noetic-mbf-costmap-nav
    ```
* Clone code
    ```sh
    cd your_ws/src
    git clone https://github.com/datledoan/regulated_pure_pursuit_controller_ros.git
    catkin build
    ```
* See its [Configuration Guide Page](https://docs.nav2.org/configuration/packages/configuring-regulated-pp.html)

# Demo

|   regulated_pure_pursuit  |   vector_pure_pursuit |
| --- | --- |
| <video src="https://github.com/user-attachments/assets/71be3f36-5989-4482-b1c1-40f238029b19">  | <video src="https://github.com/user-attachments/assets/9df01c08-a293-4bc1-8be3-a28a0d35eb7e">  |

# Reference
- [nav2_regulated_pure_pursuit_controller](https://github.com/ros-navigation/navigation2/tree/main/nav2_regulated_pure_pursuit_controller)
- [regulated_pure_pursuit_controller](https://github.com/JohnTGZ/regulated_pure_pursuit_controller)
- [vector_pure_pursuit](https://github.com/blackcoffeerobotics/vector_pursuit_controller)
- S. Macenski, S. Singh, F. Martin, J. Gines, [**Regulated Pure Pursuit for Robot Path Tracking**](https://arxiv.org/abs/2305.20026). Autonomous Robots, 2023.

```bibtex
@article{macenski2023regulated,
      title={Regulated Pure Pursuit for Robot Path Tracking}, 
      author={Steve Macenski and Shrijit Singh and Francisco Martin and Jonatan Gines},
      year={2023},
      journal = {Autonomous Robots}
}
```
# 临时修改参数
bash
```
source /opt/ros/noetic/setup.bash
rosrun dynamic_reconfigure dynparam set \
  /move_base/RegulatedPurePursuitController \
  desired_linear_vel 0.03
```
### 低速角速度补偿

`low_speed_angular_deadband` 和 `low_speed_min_angular_vel` 默认均为 0.025 rad/s，支持 ROS 参数和 dynamic_reconfigure。前者须与底盘导航输入软件死区一致；后者是满足原曲率滞回条件时的补偿输出幅值，不代表实测物理启动阈值。仅在 0.015 ≤ |v| < 0.10 m/s 生效，输入死区设为 0 可关闭补偿；输出幅值小于输入死区或参数无效时不补偿。显式设为 0.045 / 0.050 可恢复历史补偿。补偿之后仍执行原碰撞检查和最终速度限幅，底盘安全处理保持不变。新增参数不加入 APP RPP 位掩码协议。


## Grinder scan diagnostics

`/rpp/collision_detail` reports front/side distance, internal body hits, self-filter hits,
nearest XY, scan age and failure reason. It supplements the existing collision latch.
The clear timer runs for both fixed-distance and expanded-footprint modes; invalid or
repeated scans cannot supply fresh clear confirmations.

Private startup parameters: `collision_self_filter_enabled=false`,
`collision_self_filter_polygon=[]` (flat XY pairs, base frame). Filter only measured
body-return regions after scan/TF validation; the navigation footprint is not a self mask.
Boundary points are retained. Missing/invalid enabled filter configuration fails closed.
No clearance thresholds are relaxed. The filter is not dynamically reconfigured.

`scan_classifier_test` tests the production geometry and clear-confirmation helper.
`scan_replay` accepts transformed scan points; `catkin_ws/scripts/replay_collision_scan.py`
applies an explicitly supplied 4x4 transform from JSON. Never substitute a transform
from another recording. See root README 7.4 for bounded read-only bag capture.

Offline bag replay also accepts `replay_collision_scan.py recording.bag /path/to/scan_replay
--bag --bounds MIN_X MAX_X MIN_Y MAX_Y --scan-index 0`. It loads only the bag's TF,
uses the scan timestamp, and never contacts a ROS master. The rosbag Python module,
tf2_ros and tf transformations from ROS Noetic are required for bag input; JSON input
only requires Python's standard library plus the compiled classifier executable.

## 可选入口直线角点分支（新旧对照）

配置位于 `teb_local_planner_tutorials/cfg/diff_drive/rpp_local_planner_params.yaml`。
默认 `use_line_based_corner_approach: false`，保留原控制逻辑。新分支需同时开启
`use_rotate_to_heading`、`use_corner_aware_rotate_to_heading` 和该开关。
以下四个参数仅在初始化读取；修改后先停止任务，再重启 move_base，不能用动态参数中途切换：

| 参数 | 默认 | 含义 |
| --- | --- | --- |
| `use_line_based_corner_approach` | `false` | 启用入口直线接近分支 |
| `line_corner_approach_distance_m` | `0.35` | 纵向剩余距离进入此范围后按比例限速，保留最低接近速度与原速度/加速度限制 |
| `line_corner_position_tolerance_m` | `0.05` | 纵向剩余距离不大于此值开始制动；不是横向或径向验收误差 |
| `line_corner_odom_timeout_s` | `0.6` | 原始里程计时间戳的新鲜度上限；缺失、未来或非有限速度时输出零速 |

新分支的顺序为：入口直线纠偏 → 按纵向距离减速 → 下发零速 → 停稳确认 →
固定出口朝向原地转向 → 再次停稳确认 → 沿下一条入口线恢复。
虚拟前视点始终在入口直线上，可以落在角点之后的入口延长线上，不追逐缩短到零的角点，
也不让普通前视点提前进入出口线。前视距离复用 `curvature_lookahead_dist`
（关闭固定曲率前视时复用普通前视距离）。不增加横向误差暂停门禁，不倒车追逐已经越过的角点。
这不能补偿未标定的运动中心、车体倾斜或磨盘外力矩。

停止确认使用 `odom_topic`（项目配置为 `/odom_wheel`）的原始 `twist` 和时间戳，
要求速度低于原有静止阈值、持续 `rotate_to_heading_stable_cycles / controller_frequency` 秒，
且至少出现两个不同时间戳样本；同一个 10 Hz 样本在 20 Hz 控制循环中不会重复充数。
因此新增一个仅在新分支开启时使用的同话题订阅，避免通用 OdometryHelper 丢弃时间戳。
该检查只验证 ROS 消息的新鲜度和数值，不改变驱动反馈的来源或无效反馈发布语义。

角点从完整输入路径识别，忽略重复点和共线补点，普通分段终点不视为角点。
等价路径刷新保留角点坐标、出入口方向和阶段；真实重规划与 `cancel()` 清理阶段。
项目 `/chassis/task_enable=false`（停止、暂停、急停）也会作废阶段；控制调用间隔超过
里程计超时后按实际位置重新获取角点。保留原有碰撞、急停及定位保护。
旧分支继续使用 `rotate_to_heading_corner_distance` 的径向语义，新分支使用上面的纵向容差。

### 离线验证与闭环仿真

Linux + ROS Noetic，使用独立测试构建目录，不连接生产板端：

```bash
source /opt/ros/noetic/setup.bash
mkdir -p /tmp/rpp-line-build && cd /tmp/rpp-line-build
cmake /absolute/path/to/regulated_pure_pursuit_controller \
  -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=ON -DPYTHON_EXECUTABLE=/usr/bin/python3
cmake --build . --target tests line_corner_sim -j2
./devel/lib/regulated_pure_pursuit_controller/line_corner_test
```

`test/line_corner_sim.cpp` 调用实际 RPP 控制器，通过独立 ROS Master 的内存代价地图、
同步 TF 和合成 `/odom_wheel` 反馈运行差速模型，不发布运动指令到硬件。
输入 CSV 第一行为表头，后续两列为地图 x、y（米）；按 0.1 m 补点模拟调度局部路径。
必须使用独立 Master；模拟器会写自己的测试参数，并临时设置该 Master 的 `/move_base/controller_frequency=20`。

```bash
# 独立终端启动专用 Master：ROS_MASTER_URI=http://127.0.0.1:11397 roscore -p 11397
export ROS_MASTER_URI=http://127.0.0.1:11397
export LD_LIBRARY_PATH=/tmp/rpp-line-build/devel/lib:$LD_LIBRARY_PATH
sim=/tmp/rpp-line-build/devel/lib/regulated_pure_pursuit_controller/line_corner_sim
# 两次必须使用相同路径、响应模型和初始误差，只切换 0/1。
"$sim" path.csv old.csv 0 0.2
"$sim" path.csv new.csv 1 0.2
python3 /absolute/path/to/test/plot_line_corner_comparison.py \
  --path path.csv --legacy old.csv --line new.csv --output comparison.png
```

末尾模型输入依次为响应时间常数（秒，必填，0 表示理想响应）、
初始横向偏差（默认 0.06 m）、初始航向偏差（默认 0.03 rad）。
默认速度为 0.03 m/s，原地转速 0.10 rad/s；20 Hz 控制、10 Hz 合成里程计，
每秒等价刷新路径。模拟器还检查坏时间戳/非有限反馈保持零速、刷新保留阶段、
出口转向不得发生于纵向剩余距离大于 5 cm 时。
响应模型是敏感性假设，不是测得的底盘延迟；未建模磨盘力矩、打滑或真实雷达/碰撞。
仿真通过不等于实机验收通过，需现场对照轨迹、停车位置和转后偏差。
