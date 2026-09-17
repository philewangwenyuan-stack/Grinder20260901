#!/usr/bin/env bash
set -euo pipefail

# Reproducible CPU-only dependency install for Ubuntu 20.04 / ROS Noetic / RK3588.
# Run explicitly on the target; this script never replaces the ROS-provided PCL.
GTSAM_TAG="${GTSAM_TAG:-4.2.2}"
FAST_GICP_COMMIT="${FAST_GICP_COMMIT:-0e7ec1441c99f7be453db2ea216d5de029387417}"
PREFIX="${PREFIX:-/usr/local}"
JOBS="${JOBS:-2}"
WORK_DIR="$(mktemp -d /tmp/grinder-loop-deps.XXXXXX)"

cleanup() {
  case "${WORK_DIR}" in
    /tmp/grinder-loop-deps.*) rm -rf -- "${WORK_DIR}" ;;
    *) echo "Refusing to remove unexpected temporary path: ${WORK_DIR}" >&2 ;;
  esac
}
trap cleanup EXIT

sudo apt-get update
sudo apt-get install -y --no-install-recommends \
  build-essential cmake git libboost-all-dev libeigen3-dev libmetis-dev

set +u
source /opt/ros/noetic/setup.bash
set -u

git clone --depth 1 --branch "${GTSAM_TAG}" https://github.com/borglab/gtsam.git "${WORK_DIR}/gtsam"
cmake -S "${WORK_DIR}/gtsam" -B "${WORK_DIR}/gtsam-build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${PREFIX}" \
  -DGTSAM_BUILD_TESTS=OFF \
  -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
  -DGTSAM_BUILD_UNSTABLE=OFF \
  -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
  -DGTSAM_USE_SYSTEM_EIGEN=ON
cmake --build "${WORK_DIR}/gtsam-build" --parallel "${JOBS}"
sudo cmake --install "${WORK_DIR}/gtsam-build"

git clone https://github.com/koide3/fast_gicp.git "${WORK_DIR}/fast_gicp"
git -C "${WORK_DIR}/fast_gicp" checkout "${FAST_GICP_COMMIT}"
cmake -S "${WORK_DIR}/fast_gicp" -B "${WORK_DIR}/fast-gicp-build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${PREFIX}" \
  -DBUILD_apps=OFF \
  -DBUILD_test=OFF \
  -DBUILD_PYTHON_BINDINGS=OFF \
  -DBUILD_VGICP_CUDA=OFF
cmake --build "${WORK_DIR}/fast-gicp-build" --parallel "${JOBS}"
sudo cmake --install "${WORK_DIR}/fast-gicp-build"
sudo ldconfig

echo "Installed GTSAM ${GTSAM_TAG} and fast_gicp ${FAST_GICP_COMMIT} into ${PREFIX}."
echo "Rebuild with: PROFILE=mapping ./build_grinder_platform.sh"
