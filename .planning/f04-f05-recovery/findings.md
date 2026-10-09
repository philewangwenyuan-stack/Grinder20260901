# 证据
Super-LIO 是目录式嵌套 .git；上游 Liansheng-Wang/Super-LIO，GPL-3.0 LICENSE 已存在；有 SetMap.srv 新增且 CMake 已引用。scheduler.yaml 未跟踪，含设备参数；现有仿真覆盖文件虽然注明隔离服务，但 file_upload/mqtt 实为 true。

新增 5 项配置行为测试通过。原有 82 文件哈希校验：仅 .gitignore 因新增生成物排除规则而改变，其他内容保持一致。gitlink 已全部移除，LICENSE/新增 SetMap.srv 已纳入索引。受控源码原有空白格式未自动整理；业务改动 diff --check 通过。临时 index 生成候选 commit，用户分支 HEAD 不变。
