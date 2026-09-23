# Grinder 全代码工业化审查

## Goal

从完整运行链路评估代码风险、收益与实施优先级，输出可直接用于版本排期的清单；本任务只审查，不修改业务代码。

## Phases

- [complete] Phase 1 — 盘点关键模块、代码规模、测试与部署入口。
- [complete] Phase 2 — 审查安全、状态机、协议、地图和规划链路。
- [complete] Phase 3 — 审查定位建图、导航、底盘、资源与运维链路。
- [complete] Phase 4 — 汇总风险、收益、工作量和推荐版本顺序。

## Constraints

- 不递归加载整个源码树；按 AGENTS.md 路由定点检查。
- 不修改业务源码、配置或生成物。
- 优先关注停机、误动作、错误定位、数据损坏和不可恢复故障。

## Errors Encountered

| Error | Resolution |
| --- | --- |
| 仓库根目录存在上一任务的 task_plan.md | 使用独立 `.planning/codebase-industrial-audit/` 目录，避免覆盖历史计划。 |
| PowerShell 将首次组合搜索中的 `|` 解析为管道 | 改用单引号正则后重试成功；未影响源码。 |
| AGENTS.md 中二维建图包路径缺少 `mapping/` 前缀 | 通过 `rg --files` 定位到 `catkin_ws/src/mapping/cloud_to_occupancy_grid/` 后继续审查。 |
