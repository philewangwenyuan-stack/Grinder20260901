# Findings

- 板端协议接收已由 `sl_linka_adapter.py` 解析帧；当前每秒采样与每 60 秒 RX 汇总由上一任务加入。
- `scheduler_node.py` 约 1 万行，应仅查看地图分片相关局部函数。
- `super_lio_mode_manager.py` 串行执行模式切换，状态与超时可在操作边界采集。
- 根目录规划文件是另一已完成任务，保留原样。
- `build_map_chunks()` 会生成完整输出列表；本次仅测构建时间和分片数，不改变传输协议或缓存策略。
- `super_lio_mode_manager` 是单独进程；其指标不能与 scheduler 内存全局变量共享，需分别导出文件。
- `scheduler.yaml` 在本地被 Git 忽略，生产默认值必须写入源码；调参仍可在设备的本地 YAML 中完成。
- `scheduler.launch` 现在提供开关与统一导出目录；默认只启动原有进程，系统指标节点需显式启用。
- JSON 快照每进程单文件原子替换，分片构建数与成功调用 sendall 的分片数分开统计。
