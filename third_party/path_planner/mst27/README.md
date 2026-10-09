# mst27 C++ 路径规划内核

运行时目录：

```text
work/Grinder/third_party/path_planner/mst27/
├── mst27.py
├── _mst27_cpp*.so              # catkin_make 后生成
├── CMakeLists.txt               # 独立调试用
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

`StageConfig.direction` 支持 `x` / `-x` / `y` / `-y`，表示首条作业线的行进方向。
调度器会先把 `x/y` 换算成旋转后的有效 `direction_angle`，再由 MST27 应用正负号；
例如地图规划角为 `30°` 时，`x/-x/y/-y` 分别对应
`30°/210°/120°/300°`。后续作业线仍按弓字形交替方向。
若重规划起点已位于指定方向的末端，规划器先生成转场连接，再按指定方向开始首条覆盖线。

## 使用 catkin_make 编译（RK3588 / Ubuntu 20.04）

```bash
# sudo apt update
# sudo apt install -y build-essential cmake python3-dev pybind11-dev

# cd ~/catkin_ws
# catkin_make
# source devel/setup.bash

cd /home/neardi/work/Grinder/third_party/path_planner/mst27

rm -rf build
rm -f _mst27_cpp*.so

cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DPython3_EXECUTABLE="$(which python3)"

cmake --build build -j"$(nproc)"


```

`grinder_scheduler/CMakeLists.txt` 已将本目录作为 CMake 子工程加入。
编译成功后，`_mst27_cpp*.so` 会生成在 `mst27.py` 同目录，规划器将自动优先使用 C++ 内核。

## 多区域共享预处理

`PlannerAdapter.plan` 在应用原图、擦除、障碍与裁剪后，为当前请求创建
`PreparedPlanningMap`。各区域共享只读 `uint8` 原图和基础膨胀图；相同精确角度
共用角度规划图，C++ 所需 `int32` 数组也复用。区域 mask 和寻路仍独立计算，
保留全图坐标、贴边障碍分类、净距与最终碰撞复检，不裁切 ROI。

缓存随请求结束释放，不按 `map_version` 跨请求复用，因此实时地图、编辑或参数
变化后的下一请求总是重新准备。上下文固定分辨率对应的比例、基础膨胀半径、
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

## 验证

```bash
cd ~/work/Grinder/third_party/path_planner/mst27
python3 -c "import _mst27_cpp; print(_mst27_cpp.plan_core)"
```

启动调度器后，日志出现 `[C++后端] 使用C++规划内核` 即表示已启用。
