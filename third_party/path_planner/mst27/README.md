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

## 验证

```bash
cd ~/work/Grinder/third_party/path_planner/mst27
python3 -c "import _mst27_cpp; print(_mst27_cpp.plan_core)"
```

启动调度器后，日志出现 `[C++后端] 使用C++规划内核` 即表示已启用。
