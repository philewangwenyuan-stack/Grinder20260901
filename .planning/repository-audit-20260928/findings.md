# 审计证据
初始 git status: Super-LIO 子模块存在工作区改动。导航目录与实际定位/建图目录存在差异。

已确认：整块写缓存只在整块写中更新；单写/轮速写后安全停机有跳写风险。降级规划仅使用区域包围盒。无 .gitmodules 但有 Super-LIO gitlink。测试存在 sys.modules 全局污染。地图 overlay.npy 非原子覆盖。

假传输复现成功：安全停机增量帧数为 0；启用斜坡超时后发送轮速停在 80 RPM。WSL 底盘纯 Python 测试 8/8 通过。scheduler 设置 PYTHONPATH 后仍 19 项中 5 错误，包含过时 srv stub 和全局模块污染。scheduler.yaml 被忽略且需检查是否存在模板。

最终报告：16 项优先/可靠性问题、3 项结构建议。45 Python/16 XML/6 Bash 检查通过。底盘 8 测试通过；调度独立 16 项通过，metrics 4 项中 1 错误，2 文件受 Protobuf 环境阻止。重点证据：缓存停车丢帧、斜坡超时残速、启动覆盖急停、危险降级、孤立 gitlink、被忽略的必需 YAML。
