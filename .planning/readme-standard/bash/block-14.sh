cd "$GRINDER_BASE_DIR/third_party/path_planner/mst27"
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DPython3_EXECUTABLE="$(command -v python3)"
cmake --build build --parallel 2
python3 -c "import numpy, scipy, matplotlib, cv2; import _mst27_cpp; print('MST27 dependencies: OK')"
