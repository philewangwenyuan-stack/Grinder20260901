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
