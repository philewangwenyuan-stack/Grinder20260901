# 多区域规划预处理优化结果

2026-10-05，按用户授权优化全局审计 G10。本轮只改规划适配器和 MST27 预处理，
并新增回归测试、说明和定位索引；此前暂存的 F04/F05 保持原状，未提交代码。

## 实现

- 每次 `PlannerAdapter.plan` 先应用原图、擦除、障碍、裁剪，再创建一个
  `PreparedPlanningMap`，各区域共用该请求的只读地图。
- 二值图和膨胀图用 `uint8` 保存；基础膨胀只计算一次，连接寻路也复用它。
- 角度图按精确角度缓存，不舍入；比例、基础净距、角度净距、柱子尺寸固定在
  上下文中，不匹配时拒绝复用。不同地图请求重新创建上下文。
- NumPy 到 C++ 的 `int32` 输入转换复用，保留现有 C++ ABI；日志占据格计数
  也计算一次，避免每区域日志重复扫描全图。
- 默认常驻数组预算 128 MiB，派生缓存采用 LRU；超过预算的单数组不保留。
  必需原图和基础图仍需存在，SciPy 临时数组、region mask、C++ 内部网格和结果
  不在缓存预算内，不能把预算当作总 RSS 上限。

没有跨请求按单独 map_version 缓存，避免原图/参数变化后误用旧结果。本次保留
全图坐标范围：ROI 会改变贴边/长墙连通域分类以及连接路径的可达范围，需要独立
证明后再做。区域 mask 与 C++ 内部求解仍按区域处理，未宣称消除全部 O(RN) 工作。

## 耗时与内存

环境：桌面 x86_64 WSL Ubuntu 20.04、Python 3.8.10、NumPy 1.17.4、SciPy 1.3.3，
同一份从当前源码重新编译的 C++ 内核（g++ -O2），没有 ROS 或硬件连接。
比较的是**修改前保存的完整 adapter/MST27 源码**与优化后的完整 adapter；每样本
使用独立子进程，包含预处理、区域规划、区域连接与输出转换，排除 Python 模块导入。

1000×1000、5 cm/cell、10 个区域、相同角度，膨胀半径 0.6 m、角度净距 0.7 m：

| 指标 | 修改前 | 优化后 |
| --- | --- | --- |
| 三次完整规划耗时 | 6536.62 / 6523.53 / 6502.52 ms | 782.78 / 788.36 / 782.77 ms |
| 耗时中位数 | 6523.53 ms | 782.78 ms |
| 进程峰值 RSS 中位数 | 222.89 MiB | 185.02 MiB |
| 输出路径 | 2451 点 | 2451 点，所有记录 SHA256 相同 |
| 同角度派生图构造 | 各区域分别构造 | 1 次 |
| 缓存 | 无共享预处理 | 3 次 miss、27 次 hit |

该样例耗时降低 **88.00%**，峰值 RSS 约降低 37.87 MiB（17%）。这不是 RK3588
或实际整项作业的改善比例；不同角度、缓存淘汰、地图大小和求解复杂度会改变收益。
额外按运行时实际 backend 检查的一次重复样本为 6514.60→800.55 ms，路径记录仍相同。

3000×3000 的 Windows 单次准备与10次同角度复用检查：必需图18,000,000字节，
派生缓存81,000,000字节，五份常驻数组合计99,000,000字节（约94.41 MiB）；
27 hit/3 miss，在预算内。**这只是缓存容量检查，不是总 RSS 或完整任务基准。**
3000×3000 的完整 WSL 基准命令异常退出1且没有结果；主机当时剩余虚拟内存约
2.55 GiB，未将失败解释为算法缺陷，也未虚报大图完整耗时。目标 RK3588 P95/RSS
仍需在真实地图和负载下验收。

原始结果：[三次基准](E:/CODE/C++/Grinder/artifacts/planner-preprocessing-20261005/benchmark.json)、
[运行时 backend 核验](E:/CODE/C++/Grinder/artifacts/planner-preprocessing-20261005/benchmark-runtime-verified.json)、
[3000²缓存检查](E:/CODE/C++/Grinder/artifacts/planner-preprocessing-20261005/cache-3000.json)。

## 验证

核心回归包含：原图只读/独立快照、紧凑类型、角度/不同净距/柱子尺寸逐格等价、
10区域复用、LRU淘汰、请求间新图隔离、错图/错净距拒绝、无预算回退、
Python/实际 C++ 全路径与碰撞检查等价、C++ 故障回退 Python。
既有算法会拒绝部分障碍地图方向，优化后仍拒绝，未通过放松碰撞检查让测试通过。
适配器测试包含多区路径/长度等价、下一请求地图更新、无 prepare_map 兼容和 native C++。
现有4项方向测试也执行；Python3.8语法与 git diff --check 通过。

测试记录：[Linux原生验证](E:/CODE/C++/Grinder/artifacts/planner-preprocessing-20261005/linux_verification.json)、
[语法验证](E:/CODE/C++/Grinder/artifacts/planner-preprocessing-20261005/syntax_verification.json)。
Linux当前17项全部通过，无跳过；Windows对应回归15项通过、2项原生C++因不可加载Linux .so而跳过。没有执行完整Catkin
构建或真实机器人验证；本轮未改消息、服务、CMake或C++源码，原生接口已离线验证。

复现基准（WSL/Linux，先将对应版本原生扩展放在 `native/`）：

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1 \
python3 artifacts/planner-preprocessing-20261005/benchmark.py \
  --size 1000 --regions 10 --repeats 3 \
  --native-dir "$PWD/artifacts/planner-preprocessing-20261005/native"
python3 artifacts/planner-preprocessing-20261005/verify_linux.py
```

核心修改：[PlannerAdapter](E:/CODE/C++/Grinder/catkin_ws/src/grinder_scheduler/src/grinder_scheduler/planner_adapter.py:170)、
[PreparedPlanningMap](E:/CODE/C++/Grinder/third_party/path_planner/mst27/mst27.py:1860)。
当前优化不修复审计中的停车、急停、连接直线回退和路径查询副作用；这些仍按原优先级整改。
