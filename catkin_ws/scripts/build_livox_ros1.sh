#!/usr/bin/env bash
# Prepare local source dependencies and a static SDK; never starts a driver.
set -euo pipefail
workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
vendor="${workspace}/src/drivers/ws_livox/src"
cache="${workspace}/../temp/grinder_sim_livox"
jobs="${CATKIN_JOBS:-2}"

python3 "${workspace}/scripts/prepare_livox_sources.py" --workspace "${workspace}"
cmake -S "${vendor}/Livox_SDK2" -B "${cache}/sdk" -DCMAKE_BUILD_TYPE=Release
cmake --build "${cache}/sdk" --target livox_lidar_sdk_static -j"${jobs}"
echo "[INFO] Livox ROS1 sources and static SDK ready for the root Catkin build."
