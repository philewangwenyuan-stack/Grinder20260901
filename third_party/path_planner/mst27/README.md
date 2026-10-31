# mst27 C++ 路径规划内核

运行时目录：

```text
work/Grinder/third_party/path_planner/mst27/
├── mst27.py
├── _mst27_cpp*.so              # 目标平台 CMake 编译生成
├── CMakeLists.txt               # 独立扩展构建入口
└── cpp/
    ├── CMakeLists.txt           # catkin_make 使用
    └── mst27_cpp.cpp
```

调度器配置：

```yaml
planner_script_path: third_party/path_planner/mst27/mst27.py
```

该路径相对于 `GRINDER_BASE_DIR`；因此应将 `GRINDER_BASE_DIR` 设置为
`work/Grinder`。

## 扫掠方向

Grinder 调度适配器以 `RobotConfig.output_point_spacing=0` 请求全局关键点折线，
当前导航段再按 `scheduler.yaml: active_segment_point_spacing` 补点（默认 0.3 米）。
本包独立 API/演示的 `output_point_spacing` 行为保留；调度协议中的同名旧字段只保留兼容。
关键点压缩不删除实际拐角、往返绕障或作业属性边界，完整路径仍按边进行碰撞复检。

`StageConfig.direction` 支持 `x` / `-x` / `y` / `-y`，表示首条作业线的行进方向。
调度器会先把 `x/y` 换算成旋转后的有效 `direction_angle`，再由 MST27 应用正负号；
例如地图规划角为 `30°` 时，`x/-x/y/-y` 分别对应
`30°/210°/120°/300°`。后续作业线仍按弓字形交替方向。
若重规划起点已位于指定方向的末端，规划器先生成转场连接，再按指定方向开始首条覆盖线。

## 严格作业线间距

相邻中间基准作业线按 `path_spacing * (1 - overlap_ratio)` 米固定推进；
`overlap_ratio=0` 时就是设置的路径间距。任意角度均按垂直于作业线的距离计算，
保留浮点栅格坐标，不将间距四舍五入到整数格，也不按区域宽度重新均分。

首末作业线仍通过现场指定起终点。非整倍数余量仅在末端边缘吸收；
若余量不足半间距，或需匹配终点的往返方向，仅调整末端最多三个间隔，
其余中间间隔保持参数值，边缘间隔不大于设置的有效间距。
窄区域可能全部属于边缘调整范围。绕柱侧让和转场连接不属于固定间距基准线。
`output_point_spacing` / `active_segment_point_spacing` 只控制沿线补点，与作业线间距不同。

离线回归：`python3 third_party/path_planner/mst27/test/test_spacing.py -v`。
修改后须在目标 Linux 环境重新构建 `_mst27_cpp` 并重启调度器，已有规划需重新生成。

## 编译原生扩展（RK3588 / Ubuntu 20.04）

```bash
# sudo apt update
# sudo apt install -y build-essential cmake python3-dev pybind11-dev

# cd ~/catkin_ws
# catkin_make
# source devel/setup.bash

cd /home/neardi/work/Grinder/third_party/path_planner/mst27

cmake -S . -B build-board-release \
  -DCMAKE_BUILD_TYPE=Release \
  -DPython3_EXECUTABLE="$(which python3)"

cmake --build build-board-release --parallel 2


```

正式部署显式构建此独立 CMake 工程，不依赖选择性 Catkin 构建是否覆盖扩展。
编译成功后，`_mst27_cpp*.so` 会生成在 `mst27.py` 同目录，规划器将自动优先使用 C++ 内核。

## 等价膨胀优化、掉头连接与分段计时

方形障碍膨胀使用横、纵两次最大值滤波，与原方形结构元素二值膨胀逐格等价，
包含图像边缘；不改变半径、障碍范围或最终碰撞复检。避免大半径结构核的重复邻域扫描。
最终碰撞复检按最多65536个样本分批向量化，保留原半栅格采样位置、取整和首末点规则，
不抽样跳点，也不创建整条密集路径。超出地图或命中占据格仍拒绝输出。

相邻作业线优先尝试安全的外侧共同掉头线，延长较短线而不删掉较长线的覆盖端点，
消除端点量化差形成的厘米级往返。三段连接都必须通过原障碍/区域检查。
空间不足时保留原安全连接，允许必要回退；不承诺所有场景均无折返，也不生成圆弧U型。
Python回退路径同样优先尝试外侧连接，segment补点不承担几何修复。

原 `plan_core` 接口不变。新增 `plan_core_profiled` 返回 `path` 和 `timings_ms`，
计时来自本次调用的局部变量，包含输入解析、扫掠、连接、规范化、核心总时长、补点及序列化。
Python `planner.last_profile` 另记录阶段预处理、原生输入准备与最终碰撞复检。
调度每个区域只输出一条 `MST27 region phases ms`，不在每个点上打印日志。
兼容旧扩展，但要取得完整分段计时及修复需重新编译扩展并重启调度器。

回归：`python3 -m unittest discover -s third_party/path_planner/mst27/test -p 'test_*.py'`。
包含逐格膨胀等价、多角度掉头、紧边界回退、障碍检查及原间距/起终点测试。

## 多区域共享预处理

`PlannerAdapter.plan` 在应用原图、擦除、障碍与裁剪后，为当前请求创建
`PreparedPlanningMap`。各区域共享只读 `uint8` 原图和基础膨胀图；相同精确角度
共用角度规划图，C++ 直接接收这些 `uint8` 图，不再缓存扩大的 `int32` 副本。区域 mask 和寻路仍独立计算，
保留全图坐标、贴边障碍分类、净距与最终碰撞复检，不裁切 ROI。

包内角度派生缓存随请求结束释放；调度适配器可通过 `PlanningCache` 跨请求复用内容键匹配的
只读基础数组，新请求仍独立创建派生缓存。上下文固定分辨率对应的比例、基础膨胀半径、
角度膨胀半径和柱子最大尺寸；角度不舍入。输入必须是二值占据图，阶段特定障碍
须先应用到输入图；错误地图或净距上下文会被拒绝。旧规划器不提供 `prepare_map`
时，适配器仍走独立副本的兼容路径。

默认保留数组预算为 128 MiB，包含必需原图/基础图，派生图使用 LRU 淘汰；
单个派生数组超预算时只供当前调用使用，不缓存。若两份必需图本身超过预算，
关闭派生缓存而保留必需图。该预算不是进程 RSS 上限，临时 SciPy 数组、区域
mask、C++ 内部网格复制及路径结果仍有额外开销。日志
`Planner preprocessing cache` 输出必需/缓存字节、派生缓存容量与命中次数。

纯算法离线回归（不需要 ROS、不连接硬件）：

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
python3 third_party/path_planner/mst27/test/test_preprocessing.py -v
python3 catkin_ws/src/grinder_scheduler/test/test_planner_preprocessing.py -v
```

从仓库根目录执行。覆盖路径/碰撞结果等价、正负/旋转角度、不同净距、输入保护、
地图更新隔离、容量淘汰、C++ 转换复用及故障回退。有可加载原生扩展时也验证
实际 C++ 输出；没有扩展时明确跳过对应测试。

## 原生障碍预处理与字节栅格接口（4.8.0）

扩展提供 `PREPROCESSING_API=1`、`prepare_stage_grid` 和 `inflate_obstacles`。
8连通域只遍历一次，同时计算边界、尺寸和角度投影；贴边或超尺寸墙体保留轮廓，
独立小障碍生成保守角度外接矩形。矩形方形膨胀使用两次滑动窗口，墙体/基础圆形膨胀使用
精确平方欧氏距离变换。仍使用完整栅格、相同半格投影扩张和比较容差，不改变安全半径、
未知格语义、作业间距或最终路径复检，没有增加ROI裁剪或跨请求派生缓存。

`prepare_stage_grid` 返回 `grid`（连续uint8）、`stats`（分类数量）及
`timings_ms`（`classify_rectangles_ms` / `square_ms` / `disk_ms`）。
生产 `plan_core` / `plan_core_profiled` 的基础图和阶段图直接接收uint8；直接使用旧int32接口
的调用方仍兼容。进入C++后保留一份独立字节副本，再释放GIL计算，避免并发请求写共享数组；
这是减少转换和复制，不是零拷贝。整数256等非字节输入不会因强制转uint8而变成空闲格。

旧扩展缺少能力标记时仍使用Python/SciPy预处理；`MST27_DISABLE_CPP=1` 可强制完整参考路径。
新源码和对应架构扩展应一起部署并重启调度，不能把x86扩展复制到ARM64。

```bash
python3 -m unittest discover -s third_party/path_planner/mst27/test -p 'test_*.py'
python3 third_party/path_planner/mst27/test/benchmark_native_preprocessing.py
# 在目标平台用现场PGM复测；只读离线计算，不连接ROS或启动任务：
python3 third_party/path_planner/mst27/test/benchmark_native_preprocessing.py --map-image /path/to/map.pgm
```

基准PGM将灰度<250的格视为占据（含未知），不自动叠加APP区域编辑；它衡量预处理而非完整规划。
每个后端独立进程，报告首次/中位耗时、输出SHA256和Linux进程峰值RSS；比较时两端输出摘要必须相同。
2026-10-31本机WSL/x86 Release，1261×1647合成图、角度-7.834°、基础半径30格、
矩形半径14格、柱子上限60格，3次中位值：基础膨胀151.90→37.93ms，
区域准备270.06→36.90ms，独立进程峰值RSS171.39→79.34MiB。
输入使用约1000个碎片回波及柱墙，结果逐格一致；不能当作RK3588实测或APP端到端延迟。

## 验证

```bash
cd ~/work/Grinder/third_party/path_planner/mst27
python3 -c "import _mst27_cpp; print(_mst27_cpp.plan_core)"
```

启动调度器后，日志出现 `[C++后端] 使用C++规划内核` 即表示已启用。
