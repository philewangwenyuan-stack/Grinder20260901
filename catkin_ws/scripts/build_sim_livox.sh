#!/usr/bin/env bash
# Build the supplied Livox SDK outside its vendor source directory.
# This script never launches a hardware driver or changes the vendor sources.
set -euo pipefail
workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
vendor="${workspace}/src/drivers/ws_livox/src"
cache="${workspace}/../temp/grinder_sim_livox"
jobs="${CATKIN_JOBS:-2}"
for file in Livox_SDK2/CMakeLists.txt livox_ros_driver2/package.xml; do
  if [[ ! -f "${vendor}/${file}" ]]; then
    echo "[ERROR] Missing supplied Livox source: ${vendor}/${file}" >&2
    exit 1
  fi
done
cmake -S "${vendor}/Livox_SDK2" -B "${cache}/sdk" -DCMAKE_BUILD_TYPE=Release
cmake --build "${cache}/sdk" --target livox_lidar_sdk_static -j"${jobs}"
echo "[INFO] Livox SDK ready; ROS1 driver will be built by the root Catkin workspace."
