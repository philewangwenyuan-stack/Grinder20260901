# grinder_chassis_driver

`grinder_chassis_driver` 是一个面向 `RK3588 + ROS1 Noetic` 的小车底盘驱动包，使用 `Python` 实现，通过 `RS485 / Modbus RTU` 与底盘控制器通信。

这个包的目标是提供一个能直接在板端运行的纯 Python 驱动节点，完成底盘运动控制和基础状态回读，并把底层寄存器接口整理成 ROS 话题和服务，方便上层导航、调度或调试工具接入。

## 1. 工程能力

当前版本已经实现：

- 左右轮速度控制
- 磨盘转速控制
- 磨盘开关控制
- 工作模式切换
- 磨盘升降控制
- 照明控制
- 启动安全置零
- 命令超时自动停轮
- 通信失败后的降级处理
- 基础状态发布
- `/diagnostics` 诊断信息发布
- 读取 `0x4017/0x4018` 电机实际转速并发布轮式 `nav_msgs/Odometry`

当前版本暂未实现：

- `/cmd_vel` 到轮速的转换
- 轮式里程计 TF 发布（轮式 Odometry 不广播 TF，Aurora 继续提供 TF）
- 故障清除寄存器的真实写入
- 更完整的底盘状态寄存器解析

## 2. 工程结构

主要文件如下：

- [scripts/chassis_driver_node.py](/home/chersxir/work/Grinder/grinder_chassis_driver/scripts/chassis_driver_node.py)
  ROS 可执行入口
- [src/grinder_chassis_driver/chassis_driver_node.py](/home/chersxir/work/Grinder/grinder_chassis_driver/src/grinder_chassis_driver/chassis_driver_node.py)
  主节点逻辑，负责 ROS 话题/服务、命令缓存、轮询、状态发布
- [src/grinder_chassis_driver/modbus_transport.py](/home/chersxir/work/Grinder/grinder_chassis_driver/src/grinder_chassis_driver/modbus_transport.py)
  纯 Python 串口与 Modbus RTU 传输层
- [src/grinder_chassis_driver/register_map.py](/home/chersxir/work/Grinder/grinder_chassis_driver/src/grinder_chassis_driver/register_map.py)
  寄存器地址、枚举值、`int16` 编解码和命令结构
- [msg/WheelSpeedCommand.msg](/home/chersxir/work/Grinder/grinder_chassis_driver/msg/WheelSpeedCommand.msg)
  左右轮速度控制消息
- [msg/WheelSpeedState.msg](/home/chersxir/work/Grinder/grinder_chassis_driver/msg/WheelSpeedState.msg)
  左右轮目标值和反馈值
- [msg/ChassisStatus.msg](/home/chersxir/work/Grinder/grinder_chassis_driver/msg/ChassisStatus.msg)
  底盘基础状态
- [srv/EnableChassis.srv](/home/chersxir/work/Grinder/grinder_chassis_driver/srv/EnableChassis.srv)
  底盘使能/停机服务
- [srv/ClearFault.srv](/home/chersxir/work/Grinder/grinder_chassis_driver/srv/ClearFault.srv)
  故障清除服务占位
- [config/chassis_driver.yaml](/home/chersxir/work/Grinder/grinder_chassis_driver/config/chassis_driver.yaml)
  默认参数配置
- [launch/chassis_driver.launch](/home/chersxir/work/Grinder/grinder_chassis_driver/launch/chassis_driver.launch)
  启动文件

## 3. 寄存器映射

当前实现按下面这组寄存器写死：

- `0x4010` 左轮速度 `LeftWheelSpeed`
- `0x4011` 右轮速度 `RightWheelSpeed`
- `0x4012` 磨盘转速 `DiscSpeed`
- `0x4013` 磨盘开关 `DiscControl`
- `0x4014` 工作模式 `WorkMode`
- `0x4015` 磨盘升降
- `0x4016` 照明控制
- `0x4017` 左电机实时速度（只读，`int16`，默认按 rpm）
- `0x4018` 右电机实时速度（只读，`int16`，默认按 rpm）

实现假设：

- 这段寄存器既可写也可读
- 前 3 个寄存器是有符号 `int16`
- 其余控制寄存器按 `uint16` 处理

如果厂家的“读寄存器地址表”与这份假设不一致，需要同步调整 [register_map.py](/home/chersxir/work/Grinder/grinder_chassis_driver/src/grinder_chassis_driver/register_map.py) 和状态轮询逻辑。

## 4. 节点说明

节点名：

- `grinder_chassis_driver`

核心运行逻辑：

1. 节点启动后读取 ROS 参数并初始化串口
2. 如果启用了 `startup_zero_output`，启动时先下发安全值
3. 订阅 `/chassis/...` 控制话题，将命令写入对应寄存器
4. 按 `poll_rate_hz` 周期性读取 `0x4010 ~ 0x4016`，并读取 `0x4017 ~ 0x4018` 的实际电机速度
5. 将回读结果发布到状态话题、诊断话题和 `/odom_wheel`
6. 如果超过 `command_timeout` 未收到新的轮速命令，则自动停轮
7. 如果通信连续失败，节点标记为掉线并尝试进入安全状态

## 5. 话题说明

### 5.1 订阅话题

#### `/chassis/wheel_speed_cmd`

类型：`grinder_chassis_driver/WheelSpeedCommand`

字段：

- `int16 left_wheel_speed`
- `int16 right_wheel_speed`

作用：

- 直接控制左右轮速度
- 节点收到消息后会先做限幅，再分别写入 `0x4010` 和 `0x4011`
- 该话题也会刷新运动命令超时计时器

#### `/chassis/disc_speed_cmd`

类型：`std_msgs/Int16`

字段：

- `data`

作用：

- 控制磨盘转速
- 写入寄存器 `0x4012`

#### `/chassis/disc_enable_cmd`

类型：`std_msgs/Bool`

字段：

- `data`

作用：

- 控制磨盘开关
- `true` 对应 `0x0001`
- `false` 对应 `0x0000`
- 写入寄存器 `0x4013`

#### `/chassis/work_mode_cmd`

类型：`std_msgs/UInt16`

字段：

- `data`

作用：

- 控制工作模式
- 当前约定：
- `1` 表示自动模式
- `2` 表示手动模式
- 写入寄存器 `0x4014`

#### `/chassis/disc_lift_cmd`

类型：`std_msgs/UInt16`

字段：

- `data`

作用：

- 控制磨盘升降
- 当前约定：
- `1` 表示下降
- `2` 表示升起
- 写入寄存器 `0x4015`

#### `/chassis/light_cmd`

类型：`std_msgs/Bool`

字段：

- `data`

作用：

- 控制照明开关
- `true` 对应 `0x0001`
- `false` 对应 `0x0000`
- 写入寄存器 `0x4016`

### 5.2 发布话题

#### `/chassis/wheel_speed_state`

类型：`grinder_chassis_driver/WheelSpeedState`

字段：

- `std_msgs/Header header`
- `int16 target_left_wheel_speed`
- `int16 target_right_wheel_speed`
- `int16 feedback_left_wheel_speed`
- `int16 feedback_right_wheel_speed`
- `bool feedback_valid`

作用：

- 发布左右轮目标速度和轮询回读速度
- `feedback_valid=false` 表示当前通信异常或设备未连通

#### `/chassis/status`

类型：`grinder_chassis_driver/ChassisStatus`

字段：

- `std_msgs/Header header`
- `bool connected`
- `bool enabled`
- `uint16 work_mode`
- `int16 disc_speed_target`
- `int16 disc_speed_feedback`
- `bool disc_enabled`
- `uint16 disc_lift_state`
- `bool light_enabled`
- `uint32 consecutive_failures`
- `string last_error`

作用：

- 发布底盘驱动当前的综合状态
- `connected` 表示最近一次读写是否成功
- `enabled` 表示节点是否允许对外输出控制命令
- `consecutive_failures` 表示连续通信失败次数
- `last_error` 用于诊断串口打开失败、超时、CRC 错误等问题

#### `/odom_wheel`

类型：`nav_msgs/Odometry`

作用：

- `twist.twist.linear.x` 为左右轮实际速度平均得到的底盘线速度（m/s）
- `twist.twist.angular.z` 为差速模型计算得到的底盘角速度（rad/s）
- 计算过程为：电机 rpm ÷ 减速比得到轮 rpm，再按轮半径和轮距换算
- 该消息不发布 TF；`pose` 被标记为不可用，避免与 Aurora 的位姿/TF 来源冲突

#### `/diagnostics`

类型：`diagnostic_msgs/DiagnosticArray`

作用：

- 与 ROS 诊断体系兼容
- 当前包含：
- 串口名
- 波特率
- 从站 ID
- 连续失败次数
- 在线/离线状态

## 6. 服务说明

### `/chassis/enable`

类型：`grinder_chassis_driver/EnableChassis`

请求：

- `bool enable`

响应：

- `bool success`
- `string message`

作用：

- `enable=true` 时恢复底盘输出，并尝试把当前缓存命令重新写入寄存器
- `enable=false` 时进入停机状态，并主动下发安全停止

### `/chassis/clear_fault`

类型：`grinder_chassis_driver/ClearFault`

请求：

- 无字段

响应：

- `bool success`
- `string message`

作用：

- 当前只是占位服务
- 由于厂家尚未提供故障清除寄存器，所以目前固定返回失败说明

## 7. 参数说明

默认参数文件见 [chassis_driver.yaml](/home/chersxir/work/Grinder/grinder_chassis_driver/config/chassis_driver.yaml)。

主要参数如下：

- `~port`
  默认 `/dev/ttyS0`
- `~baudrate`
  默认 `115200`
- `~slave_id`
  默认 `1`
- `~timeout`
  默认 `0.1`
- `~poll_rate_hz`
  默认 `20.0`
- `~command_timeout`
  默认 `0.5`
- `~write_verify`
  默认 `false`
- `~max_retries`
  默认 `3`
- `~stop_on_timeout`
  默认 `true`
- `~startup_zero_output`
  默认 `true`
- `~left_speed_min`
  默认 `-32768`
- `~left_speed_max`
  默认 `32767`
- `~right_speed_min`
  默认 `-32768`
- `~right_speed_max`
  默认 `32767`
- `~disc_speed_min`
  默认 `-32768`
- `~disc_speed_max`
  默认 `32767`
- `~max_cmd_step_rpm`
  默认 `50`
- `~max_echo_deviation`
  默认 `200`
- `~max_echo_failures`
  默认 `2`
- `~safe_stop_on_error_interval`
  默认 `1.0`
- `~wheel_odom_topic`
  默认 `/odom_wheel`
- `~wheel_odom_frame_id`
  默认 `odom`
- `~wheel_odom_child_frame_id`
  默认 `base_link`
- `~actual_motor_speed_scale`
  默认 `1.0`；如果寄存器不是直接 rpm，可用该参数换算为 rpm
- `~actual_speed_left_sign`、`~actual_speed_right_sign`
  默认 `1`；用于修正实际速度正负方向
- `~actual_speed_swap_lr`
  默认 `false`；用于修正左右反馈寄存器顺序

参数含义：

- `write_verify`
  写寄存器后是否再回读校验
- `stop_on_timeout`
  是否在控制命令超时后自动停轮
- `startup_zero_output`
  节点启动时是否先发送安全零输出
- `max_cmd_step_rpm`
  左右轮每次下发允许变化的最大步进（斜坡限制）
- `max_echo_deviation`
  下发轮速与寄存器回读轮速允许的最大偏差
- `max_echo_failures`
  连续回显偏差超限次数，超限后触发安全停车
- `safe_stop_on_error_interval`
  通信异常触发“安全全量写入”的最小时间间隔（秒），用于避免总线刷屏

### 7.1 `modbus_20260427.cpp` 算法参数对照表

下表用于对照 `third_party/modbus_20260427.cpp` 与当前 ROS 驱动中的同类参数，便于现场调参与一致性检查。

| `modbus_20260427.cpp` 常量 | ROS 参数名 | 默认值 | 作用 |
| --- | --- | --- | --- |
| `MAX_INPUT_V` | `~cmd_vel_max_input_v` | `0.083` | 线速度输入限幅（m/s） |
| `MAX_INPUT_W` | `~cmd_vel_max_input_w` | `0.1` | 角速度输入限幅（rad/s） |
| `MAX_ABS_WHEEL_RPM` | `~cmd_vel_max_abs_wheel_rpm` | `1500.0` | 左右轮目标转速绝对值限幅（rpm） |
| `MAX_CMD_STEP_RPM` | `~max_cmd_step_rpm` | `50` | 每次控制周期轮速步进上限（斜坡限制） |
| `MAX_ECHO_DEVIATION` | `~max_echo_deviation` | `200` | 命令与回显允许偏差上限（rpm） |
| `MAX_ECHO_FAILURE_COUNT` | `~max_echo_failures` | `2` | 连续回显偏差超限触发安全停车阈值 |
| `WHEEL_BASE` | `~cmd_vel_wheel_track_m` | `0.5` | 左右轮中心距（m） |
| `WHEEL_RADIUS` | `~cmd_vel_wheel_radius_m` | `0.1` | 轮半径（m） |
| `GEAR_RATIO` | `~cmd_vel_gear_ratio` | `60.0` | 轮速换算齿比 |
| `REG_LEFT_CMD` | 固定寄存器 `0x4010` | `0x4010` | 左轮速度目标寄存器 |
| `REG_RIGHT_CMD` | 固定寄存器 `0x4011` | `0x4011` | 右轮速度目标寄存器 |

## 8. 消息与服务定义

### `WheelSpeedCommand.msg`

```text
int16 left_wheel_speed
int16 right_wheel_speed
```

### `WheelSpeedState.msg`

```text
std_msgs/Header header
int16 target_left_wheel_speed
int16 target_right_wheel_speed
int16 feedback_left_wheel_speed
int16 feedback_right_wheel_speed
bool feedback_valid
```

### `ChassisStatus.msg`

```text
std_msgs/Header header
bool connected
bool enabled
uint16 work_mode
int16 disc_speed_target
int16 disc_speed_feedback
bool disc_enabled
uint16 disc_lift_state
bool light_enabled
uint32 consecutive_failures
string last_error
```

### `EnableChassis.srv`

```text
bool enable
---
bool success
string message
```

### `ClearFault.srv`

```text
---
bool success
string message
```

## 9. 构建与运行

### 9.1 编译

如果包已经放到 catkin 工作空间的 `src/` 下：

```bash
cd /home/chersxir/work/Grinder/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

### 9.2 启动

```bash
roslaunch grinder_chassis_driver chassis_driver.launch
```

如果要临时覆盖串口：

```bash
roslaunch grinder_chassis_driver chassis_driver.launch port:=/dev/ttyS0
```

## 10. 调试示例

发送左右轮速度：

```bash
rostopic pub /chassis/wheel_speed_cmd grinder_chassis_driver/WheelSpeedCommand "{left_wheel_speed: 100, right_wheel_speed: 100}" -r 10
```

关闭底盘输出：

```bash
rosservice call /chassis/enable "enable: false"
```

打开磨盘：

```bash
rostopic pub /chassis/disc_enable_cmd std_msgs/Bool "data: true" -1
```

设置磨盘转速：

```bash
rostopic pub /chassis/disc_speed_cmd std_msgs/Int16 "data: 800" -1
```

查看状态：

```bash
rostopic echo /chassis/status
```

查看诊断：

```bash
rostopic echo /diagnostics
```

## 11. 当前限制和注意事项

- 当前“状态回读”读取 `0x4010 ~ 0x4016`，实际电机速度读取 `0x4017/0x4018`
- 如果实际设备的读地址和写地址不同，需要改轮询实现
- 当前节点没有实现 `/cmd_vel -> 左右轮速` 转换
- 当前节点发布速度来源为真实电机反馈的 `/odom_wheel`，不发布轮式里程计 TF
- 当前节点没有实现故障清除真实功能
- 若串口不存在或权限不足，`last_error` 会直接反映在 `/chassis/status` 和 `/diagnostics`
- 该驱动当前面向单从站设备，默认 `slave_id=1`

## 12. 后续建议

建议下一步继续补：

- 串口收发十六进制日志
- `/cmd_vel` 转左右轮速转换节点
- 故障码寄存器解析
- 真实底盘联调记录
- Modbus 从站模拟器，便于离线自测
