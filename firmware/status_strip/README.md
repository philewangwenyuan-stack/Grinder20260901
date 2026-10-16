# RK3588 控制盒状态灯带

硬件：微雪 ESP32-S3-Zero（FH4R2，4 MB Flash）、10 cm / 60 灯每米的
5V WS2812B 灯带（6 颗灯）、74AHCT125、330Ω 电阻、乳白扩散罩。
本固件只驱动指示灯，不参与底盘控制或证明安全停车。

## 接线

RK3588 USB 数据线接开发板 Type-C，同时供电和通信。开发板 5V 引脚
接灯带 5V 和 AHCT125 VCC；所有 GND 共地。不再并接外部 5V，不用 3V3
给灯带供电。电源能力以实际 USB 口、线材及开发板验证结果为准。

使用 SOIC-14 裸芯片时，74AHCT125：14 脚 VCC 接 5V，7 脚 GND 接地；
1 脚 /1OE 接地，2 脚 1A 接 GPIO4，3 脚 1Y 经 330Ω 接灯带 DIN。
VCC/GND 间就近接 0.1µF 陶瓷电容；灯带入口接 100µF/10V 电解电容，注意极性。
未使用的 /OE（4、10、13）接 5V，未使用 A（5、9、12）接地，未使用 Y 悬空。
按芯片圆点/缺口确认 1 脚，不能按商品照片方向猜。若买模块，按模块引脚标签接。
数据电阻放在第一颗灯 DIN 附近；GPIO21 是板载灯，本项目用 GPIO4。

官方资料：
[开发板原理图](https://files.waveshare.com/wiki/ESP32-S3-Zero/ESP32-S3-Zero-Sch.pdf)、
[AHCT125 数据手册](https://www.ti.com/lit/ds/symlink/sn74ahct125.pdf)。

## 编译与烧录

使用 PlatformIO Core 6.1.18；项目固定 espressif32 6.9.0 / Arduino 2.0.17，
Adafruit NeoPixel 1.12.3。使用 DevKitC-1 的编译定义但覆盖为 4 MB Flash；
不使用 PSRAM，USB 使用芯片原生 CDC/JTAG。

```bash
python3 -m pip install platformio==6.1.18
cd firmware/status_strip
pio run
# 用户接好控制器后，手动执行；不要把底盘串口误当烧录端口。
pio run -t upload --upload-port /dev/ttyACM0
```

首次烧录按住 BOOT，再连接 USB，必要时按 RESET 后释放 BOOT。
Windows 烧录端口用实际 COM 号。烧录完成重启，6 颗灯应黄灯常亮。
固件不会等待 USB 打开；只有 USB 口开始供电后才能亮灯。
Wi-Fi/Bluetooth 不初始化；看门狗超时 5 秒。调试日志不能混入 USB 协议。

## 状态灯效

| 状态 | 灯效 |
| --- | --- |
| 控制器启动、ROS 状态不完整 | 黄色常亮 |
| 待机、准备、正常停止、路径规划 | 绿色常亮 |
| 自动任务 | 蓝色呼吸（2 秒） |
| 自动任务人为暂停 | 蓝色慢闪（1 Hz） |
| RPP 碰撞风险触发的暂停 | 黄色慢闪（1 Hz） |
| 手动控制活动或非自动任务磨盘使能 | 青色常亮 |
| 完成 | 绿色闪三次后常亮 |
| 故障、底盘断连、就绪后状态过期 | 红色慢闪（1 Hz） |
| 急停提示未清除 | 红色快闪（4 Hz） |

固件默认及最多输出 102/255（40%）亮度；6 颗灯整圈同步。
调整亮度上限需同时更新 ROS 节点、配置和 ESP32 固件并重新烧录；
旧固件会拒绝超过原上限51的状态帧，板端旧配置51则仍以20%运行。
调度器记录暂停原因；RPP 风险信号恢复为 FALSE 不会清除此原因，也不会自动续跑。
黄色慢闪保持到任务成功恢复、新任务成功启动或明确停止；重复 PAUSE 不覆盖碰撞原因。
恢复因障碍仍存在而失败时也继续黄色慢闪。该提示表示前方碰撞风险，不代表已经发生撞击。
急停优先于所有状态，包括 USB 超时。普通 RUNNING 或重新连接不会清除急停。
固件复位后从黄灯开始，由 ROS 来源重新发送持久化急停提示。

## Linux / ROS 启动

因为增加安装入口和 std_srvs 依赖，目标 Linux 必须重新 Catkin 构建：

```bash
cd catkin_ws
PROFILE=runtime ./build_grinder_platform.sh
source devel/setup.bash
sudo apt-get install python3-serial
# 先跑模拟后端：不会打开 USB，不会发送运动命令。
roslaunch grinder_scheduler status_indicator.launch
rostopic echo /indicator/state
```

模拟后端用真实生产状态判定；缺少任一必需来源保持黄灯，已就绪后失去来源变红。
必需来源：`/scheduler/status`、`/chassis/status`、`/scheduler/control_activity`、
`/scheduler/indicator_emergency`、`/chassis/indicator_emergency`。
`control_activity` 的 JSON 含 `automatic_active`、`manual_active` 布尔值以及
`pause_reason`（空字符串、`manual`、`collision`）；只有 PAUSED 才发布非空原因。
旧来源省略原因时按普通蓝色暂停处理，新版来源负责保留碰撞暂停原因。
底盘 `connected=false` 或调度 ERROR 即使尚未首次就绪也显示故障。
不将 APP、视频、MQTT 在线作为绿色就绪条件；定位质量保持由调度现有状态反映。

不接底盘的台架测试可设置 `ignore_chassis_status: true`，或独立 launch 时加
`ignore_chassis_status:=true`（通过 scheduler.launch 启动则用
`indicator_ignore_chassis_status:=true`）。修改 YAML 后重启灯带节点。
该开关默认 false；开启后忽略底盘状态缺失、过期和 connected=false，
不使用底盘磨盘反馈判定手动灯效；已收到的底盘急停 TRUE 仍优先显示。
调度状态、活动状态和调度急停状态仍必须有效，ESP32 USB 超时仍报红。
这只改变灯效，不改变任务或底盘安全控制；接回底盘时恢复 false。
`/indicator/state` 会显示 `ignore_chassis_status`，便于核对测试开关。
此开关在 ROS 灯带节点中，ESP32 插在 Windows 电脑上且未运行该节点时不会生效。

接上控制器后查设备身份并确保运行用户有串口权限：

```bash
python3 -m serial.tools.list_ports -v
sudo usermod -aG dialout "$USER"
# 重新登录后，填写控制器的实际序列号：
roslaunch grinder_scheduler status_indicator.launch backend:=serial usb_serial_number:=实际序列号
# 或使用稳定的 by-id 路径，不能用易变 ttyACM 编号作为生产配置：
roslaunch grinder_scheduler status_indicator.launch backend:=serial device:=/dev/serial/by-id/实际设备
```

也可以在 `scheduler.launch` 传入 `enable_status_indicator:=true`
和 `indicator_backend:=serial`、`indicator_usb_serial_number:=实际序列号`。
默认不开启；单独 launch 默认 mock，不接硬件。
MID-360 + Super-LIO 整套开机自启通过 `grinder.service` 统一管理；安装和配置步骤见
[根 README](../../README.md#12-板端开机自启主系统与灯带统一开关)。服务已经启动灯带时，不要另开独立 launch。
配置位于 `grinder_scheduler/config/status_indicator.yaml`，固件灯珠数量固定为 6，
实际灯珠数量变更需修改固件并重新烧录。

`/indicator/state` 返回状态名、后端、ACK、待清除标志和通信错误；
`/diagnostics` 的 `grinder/status_indicator` 报告 USB 通信情况。
`acknowledged=true` 只证明协议响应，不能证明灯珠发光或机器停车。

## 急停灯光提示复位

调度和底盘收到急停时，各自锁存显示记录并立即发布。磁盘保存由独立线程完成，
不在停车路径做文件 I/O；突然掉电早于保存时仍可能丢失刚收到的记录。
默认记录位于运行账户的 `~/.ros/grinder_indicator/`，
可以在各自 YAML 设置 `indicator_emergency_file`。重启使用相同账户和记录路径。
损坏/不可读取记录保守显示急停，日志报告保存失败。

```bash
# 只清除灯光提示，不复位真实急停、不使能底盘、不清除底盘故障。
rosservice call /indicator/clear_emergency "{}"
```

服务会清除两个来源记录，等待新 FALSE 状态，再向 ESP32 发送显式清除标志。
返回成功表示复位已排队；查看 `/indicator/state` 的 `clear_pending=false`
和 `acknowledged=true` 才能确认控制器已响应。通信恢复前持续重试最新清除请求。
来源内部的 `/scheduler/clear_emergency_indicator`、`/chassis/clear_emergency_indicator`
不应单独用于用户复位：它们不能单独清除控制器锁存。
重新发生急停会取消尚未发送的显示复位。显示记录不承担运动锁存职责。

正式解除软件急停使用上位机 `emergency_stop.enabled=false` 或：

```bash
rosservice call /scheduler/reset_emergency_stop "{}"
```

驱动在3秒内验证安全输出确认及两次新的停止读数后解除，调度仍保持停止，
随后自动调用灯光清除；不会自动恢复任务。运动锁存记录在 `~/.ros/grinder_safety/`，
跨重启保留，与灯光提示文件独立；修改账户/HOME时需保持记录位置一致。
`/scheduler/motion_safety` 和 `/chassis/motion_safety`（JSON String）报告锁存、复位和安全写入状态。
无底盘且忽略底盘状态时，调度入口只排队清灯、保留运动锁存并返回失败说明。
灯光清除在测试模式只可跳过从未出现且服务不存在的底盘来源；已有来源必须响应。
清除后按实时状态重新选灯，其他故障可继续红色慢闪。物理急停须按设备流程另外解除。

## USB 协议 v1

ASCII 换行帧，最大 127 字节：

```text
S1,<session>,<sequence>,<state>,<brightness>,<clear>*<CRC>\n
A1,<session>,<sequence>*<CRC>\n
```

CRC-16/CCITT-FALSE：初值 FFFF、多项式 1021，对 `*` 前的 ASCII 内容计算，
4 位大写十六进制。session 为 16 位小写十六进制，每次串口连接生成新值。
sequence 为 1..FFFFFFFF，递增；state 为 0..8，顺序与固件 State 枚举一致；
新增 8 为 COLLISION_PAUSED，原有 0..7 编号不变；部署时同步更新调度代码与 ESP32 固件。
旧固件会拒绝状态码 8 并进入通信超时红灯，不能显示黄色慢闪。
brightness 为 0..102；clear 为 0/1，EMERGENCY 不允许 clear=1。
每次仅一个未确认帧，ACK 超时 0.5 秒，失败后 2 秒重连，只发送最新状态。
发送周期 0.5 秒；固件 3 秒没收到新有效帧切换故障。
重复帧返回 ACK 但不延长心跳；旧序号及最近 4 个已退休连接会话被拒绝。
首次通信前保持黄灯；新会话也不能隐式清除已锁存急停。

## 离线验证

```bash
python3 -m unittest discover -s catkin_ws/src/grinder_scheduler/test -p 'test_status_indicator*.py'
python3 -m unittest discover -s catkin_ws/src/grinder_chassis_driver/test -p 'test_indicator_emergency.py'
cd firmware/status_strip
c++ -std=c++11 -Iinclude test/protocol_host.cpp -o protocol_host
./protocol_host
# 可选：让 Python 帧与实际 C++ 解析器互通（从仓库根目录执行）：
# INDICATOR_PROTOCOL_TEST_EXE=/absolute/path/protocol_host python3 -m unittest discover \
#   -s catkin_ws/src/grinder_scheduler/test -p 'test_status_indicator*.py'
```

硬件验收测电压/电流、检查颜色、停止 ROS 灯带节点验证 3 秒后变红、拔插 USB
验证重连。使用模拟状态输入，不向底盘控制话题发送测试轮速、磨盘或急停命令。

本次开发验证：ESP32 固件编译通过；Ubuntu 20.04 / Python 3.8 下 23 项指示灯
测试（含 Python/C++ 协议互通）及底盘完整 13 项单测通过；底盘 Catkin 构建通过。
独立测试工作区未安装 `slamware_ros_sdk`，调度包 Catkin 配置在该依赖处受阻，
需在具备完整项目依赖的目标 Linux 上完成构建。尚未烧录或进行实机灯效/电气验收。
