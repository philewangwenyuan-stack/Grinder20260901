# 底盘通信审计依据

- `catkin_ws/src/grinder_chassis_driver/config/chassis_driver.yaml`：19200、5 Hz、命令超时 5 秒、`write_verify: false`、通信故障回零关闭、原始串口日志开启。
- `catkin_ws/src/grinder_chassis_driver/src/grinder_chassis_driver/chassis_driver_node.py`：每轮两次读取；`/odom_wheel` 随轮询发布；双轮速度相同时普通写入去重；安全零速部分路径也经过该普通写入函数。
- `catkin_ws/src/grinder_chassis_driver/src/grinder_chassis_driver/modbus_transport.py`：功能码 16 写入等待 8 字节确认；读响应按设备返回的字节数接收，但未核对等于请求寄存器数乘二。
- `catkin_ws/src/grinder_scheduler/config/scheduler.yaml`：手动命令看门狗 2 秒。
- 平板手动摇杆在持续输入时约每 200 ms 发一次，部分页面在位置变化时还会立即发送。
- 19200、8N1 理论上每字节约 0.521 ms；当前两次轮询合计约 44 字节，双轮写入加确认约 21 字节。理论时长不含控制器处理和 RS485 换向。
