# regulated_pure_pursuit_controller
Regulated Pure Pursuit Controller for ROS. Applicable for both [move_base](http://wiki.ros.org/move_base) and [move_base_flex](http://wiki.ros.org/move_base_flex). Tested with ROS-Noetic.

## Update 
Implement [vector_pure_pursuit](https://github.com/blackcoffeerobotics/vector_pursuit_controller). To use it, set the parameter `use_vector_pure_pursuit` to `true` and follow it's [Configuration Guide](https://docs.ros.org/en/humble/p/vector_pursuit_controller/)

### 本项目的 Vector 配置

项目导航 YAML 启用 `use_vector_pure_pursuit: true`，使用 `k: 6.0`、`min_turning_radius: 0.20` m，原地对齐仍关闭。Vector 分支同时使用前瞻点的位置和所在路径段的方向计算曲率，继续经过原有速度约束、低速补偿、碰撞检查及最终限幅；并不新增绕障规划或保证输出固定半径圆弧。开启独立曲率前瞻时，当前实现仍用 `curvature_lookahead_dist` 对应的曲率计算减速与行驶角速度。

`min_turning_radius` 是计算半径下限，不是期望转弯半径。Vector 模式下应设正值：禁止倒车且目标落在车后时，当前实现直接使用该半径，设为 0 会在后续取倒数时除零。0.20 m 是数值验证的起始配置，尚非实车标定值，也不限制内侧轮必须正转。`k` 为无量纲权重参数，不应简单理解为增大就一定更快收敛。

绕柱路径须由规划器提供，且须考虑整车及前置磨盘的扫掠范围；跟踪参考曲线成功不等于障碍物避碰验收。重启导航以加载 YAML，或在停止任务后同步调整动态配置；不会自动修改已经运行的节点。设 `use_vector_pure_pursuit: false` 可返回标准 RPP，`k` 和此半径下限不参与标准曲率计算。

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
