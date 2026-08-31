#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${WORKSPACE}"

if [[ ! -f /opt/ros/noetic/setup.bash ]]; then
  echo "[ERROR] ROS Noetic is not installed at /opt/ros/noetic" >&2
  exit 1
fi
set +u
source /opt/ros/noetic/setup.bash
set -u

if [[ ! -f devel/setup.bash ]]; then
  echo "[ERROR] catkin workspace is not built. Run: PROFILE=sim ./build_grinder_platform.sh" >&2
  exit 1
fi

if [[ -f devel/.catkin ]] && ! grep -Fq "${WORKSPACE}/src" devel/.catkin; then
  echo "[ERROR] devel was generated for another workspace:" >&2
  tr ';' '\n' < devel/.catkin | sed 's/^/[ERROR]   /' >&2
  echo "[ERROR] rebuild safely with: PROFILE=sim ./build_grinder_platform.sh" >&2
  exit 1
fi
set +u
source devel/setup.bash
set -u

SIM_PACKAGE_PATH="$(rospack find grinder_gazebo 2>/dev/null || true)"
if [[ "${SIM_PACKAGE_PATH}" != "${WORKSPACE}/src/grinder_gazebo" ]]; then
  echo "[ERROR] grinder_gazebo is not available from this workspace." >&2
  echo "[ERROR] rebuild with: PROFILE=sim ./build_grinder_platform.sh" >&2
  exit 1
fi

if ! command -v ffmpeg >/dev/null 2>&1; then
  echo "[WARN] ffmpeg is not installed; APP control will work, but local RTSP video will be unavailable." >&2
  echo "[WARN] Install it with: sudo apt-get install -y ffmpeg" >&2
fi

if ! python3 -c 'import scipy' >/dev/null 2>&1; then
  echo "[WARN] python3-scipy is not installed; the scheduler will use its fallback planner." >&2
  echo "[WARN] Install it with: sudo apt-get install -y python3-scipy" >&2
fi

for port in 8002 8554; do
  if command -v ss >/dev/null 2>&1 && ss -ltn "sport = :${port}" | grep -q LISTEN; then
    echo "[ERROR] TCP port ${port} is already in use. Stop the previous simulator/device stack first." >&2
    exit 1
  fi
done

echo "[INFO] Starting X920 Gazebo simulator"
echo "[INFO] APP protocol: tcp://0.0.0.0:8002"
echo "[INFO] Local RTSP:  rtsp://<host>:8554/left and /right"
exec roslaunch grinder_gazebo grinder_sim.launch "$@"
