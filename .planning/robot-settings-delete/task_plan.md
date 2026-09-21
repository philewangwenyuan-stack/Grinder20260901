# 机器人设置与当前地图删除

## Goal

完成板端端到端支持：

1. 机器人 footprint、`base_link -> base_laser_link` 和 RPP 核心参数具备可读取/写入链路；RPP 支持临时动态应用，持久化设置有明确语义。
2. 当前地图删除不再被平台文件服务器超时阻塞：本地地图与 registry 先完成删除，远端删除进入可重试的后台流程。

## Phases

- [complete] Phase 1 — 核对现有 SL-Link、scheduler 地图删除、平台删除和 ROS 参数入口。
- [complete] Phase 2 — 设计并实现协议/板端设置读写与 RPP 动态应用。
- [complete] Phase 3 — 修复当前地图删除顺序、结果语义和远端重试。
- [complete] Phase 4 — 补充测试、配置说明和最小构建/语法验证。
- [complete] Phase 17 — 修正重定位编辑态、中心点/箭头标记和地图交互。
- [complete] Phase 18 — 将导航参数收敛到核心字段并统一设置页风格。
- [complete] Phase 19 — 增加路径规划参数页面并接入协议、scheduler 与规划器。
- [complete] Phase 20 — 完成静态检查与协议生成物核对；本轮不打包 APK。

## Constraints

- 保持 Python 3.8 兼容；不直接修改通用 ROS navigation 源码，优先使用已有配置/适配层。
- 删除流程必须保留安全保护：当前地图停止使用后再删除本地数据；远端状态未知不能伪装成远端成功。
- Android 端字段必须真正映射到板端运行时或持久化配置，不能只增加假输入框。
- footprint 默认基准按用户确认的矩形 `x=[-0.60,1.00]`、`y=±0.47`；最终是否写入现有 YAML 要以链路核对结果为准。

## Errors Encountered

| Error | Resolution |
| --- | --- |
| 用户提供的生成图片路径带有 `/C:/` 前缀导致 Windows 路径错误 | 使用 `C:\\Users\\...` 原生 Windows 路径读取 |
