# 机器人与 Android APP 联合复审

目标：复核既有 F01–F16；审计资源、效率、通信和后台返回连续性。只写审计材料，保留业务代码与既有改动，不操作硬件。

- [x] 核对机器人与 APP 当前基线、旧问题状态。
- [x] 追踪 APP 生命周期、连接、重连、数据同步与控制。
- [x] 对照双端协议、队列、分片、状态恢复及资源热点。
- [x] 执行离线验证，记录证据及无法验证的实机场景。
- [x] 交付优先级、修改建议、收益和验收矩阵。

错误：外层 E:/CODE/grindingrobot 非 Git 仓库，已定位内层 grindingrobot。

环境/验证记录：WSL E_ABORT，改 Windows Python；Gradle daemon 原生内存失败，单 worker 重试后测试 JVM 页面文件不足。直接 JVM 首次选择缓存旧依赖与旧 device 测试造成运行错误，随后按声明固定 protobuf/Kotlin/coroutines 并重编译 controller 测试，73 项通过。缓存分支探针初次抽取包装器，改抽取 impl_core 后通过。Windows transport 测试缺 termios，未冒充 Linux 通过。

结果：16 项旧发现逐项复核、13 项新发现、资源预算和 8 场景前后台验收矩阵；AUDIT.md 与 verification.json 已保存。未修改业务代码或提交既有修复。
