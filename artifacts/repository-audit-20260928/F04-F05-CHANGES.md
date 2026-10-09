# F04/F05 修复结果

2026-09-28：改动已暂存，用户分支 HEAD 保持 `2c02e72`，未创建分支提交。

- Super-LIO 的 82 个原有源码、配置和文档文件转为根仓库普通文件，另加入
  `UPSTREAM.md` 记录上游地址、提交及 GPL-3.0 许可证来源。原本未提交的源码改动保留。
  补齐现有 `basic` 依赖的 package.xml/CMake 导出声明，避免按依赖选择构建时跳过该包。
- 原嵌套 Git 历史保存在该目录被忽略的 `.git.nested-backup-20260928-f04/`。
  工作区补丁、原文件哈希及原始设备配置备份在根仓库
  `.git/local-backups/f04-f05-20260928/`；生成的 PCD/图片/NPY 留在本地并排除。
- `scheduler.yaml` 作为公共默认配置纳入版本控制，清空设备服务凭据和端点，
  默认关闭平台上传、MQTT 和远端推流。15 个设备差异参数迁到被忽略的
  `scheduler.local.yaml`；公共配置合并本地覆盖与迁移前参数完全一致。
- 真实启动脚本自动发现本地覆盖；也可通过 `GRINDER_SCHEDULER_LOCAL_CONFIG`
  或 launch 参数 `scheduler_local_config` 指定文件。新增启动前 YAML 校验。
  仿真默认不发现设备配置，并在最后强制关闭三个外部服务。
- Catkin 安装排除 `scheduler.local.yaml`；默认配置、代码回退值、四个 launch
  和文档已同步。

验证：独立临时快照从根仓库克隆到新目录，确认无 gitlink、无秘密配置，
82 个原始文件均可恢复；5 项配置行为测试通过，Python 3.8/Bash 检查通过。
使用实际 ROS Noetic XML 加载器验证了真实/仿真两层 launch、arg/env 覆盖、
缺失/无效本地配置拒绝及仿真服务隔离。全程未启动 ROS 节点或操作硬件。

Catkin 构建已尝试：`basic` 已纳入依赖顺序，但当前 WSL 环境缺少外部
`livox_ros_driver2`，在 CMake 配置阶段停止，因此没有完整编译通过结论。
验证明细位于 `.planning/f04-f05-recovery/verification.json`。

F01/F02 的停车缓存和急停并发行为本次未修改。
