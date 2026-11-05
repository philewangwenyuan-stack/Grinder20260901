#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${WORKSPACE}"
# GPU lidar and simulated cameras need rendering even with gui:=false.
# SSH shells may omit DISPLAY although the local WSLg server is available.
if [[ -z "${DISPLAY:-}" && -S /mnt/wslg/.X11-unix/X0 ]]; then
  export DISPLAY=:0
fi
source "${WORKSPACE}/scripts/scheduler_config.sh"
configure_scheduler_config "${WORKSPACE}" sim

# Keep the selected external scene across source syncs (temp/ is device-local).
SIM_WORLD="${GRINDER_SIM_WORLD:-}"
if [[ -z "${SIM_WORLD}" && -f "${WORKSPACE}/temp/sim_world_path" ]]; then
  IFS= read -r SIM_WORLD < "${WORKSPACE}/temp/sim_world_path" || true
  SIM_WORLD="${SIM_WORLD%$'\r'}"
fi
for launch_arg in "$@"; do
  if [[ "${launch_arg}" == world_name:=* ]]; then
    SIM_WORLD="${launch_arg#world_name:=}"
  fi
done
if [[ -n "${SIM_WORLD}" ]]; then
  if [[ ! -f "${SIM_WORLD}" ]]; then
    echo "[ERROR] Simulation world does not exist: ${SIM_WORLD}" >&2
    exit 1
  fi
  export GRINDER_SIM_WORLD="$(realpath "${SIM_WORLD}")"
  SIM_MODELS_PATH="$(dirname "${GRINDER_SIM_WORLD}")/../models"
  if [[ -d "${SIM_MODELS_PATH}" ]]; then
    export GAZEBO_MODEL_PATH="$(realpath "${SIM_MODELS_PATH}")${GAZEBO_MODEL_PATH:+:${GAZEBO_MODEL_PATH}}"
  fi
fi

if [[ -f "${WORKSPACE}/temp/sim_spawn_pose" ]]; then
  SIM_SPAWN_POSE=()
  read -r -a SIM_SPAWN_POSE < "${WORKSPACE}/temp/sim_spawn_pose" || true
  if [[ "${#SIM_SPAWN_POSE[@]}" -ne 3 ]]; then
    echo "[ERROR] temp/sim_spawn_pose must contain: x y yaw (metres, radians)" >&2
    exit 1
  fi
  for pose_value in "${SIM_SPAWN_POSE[@]}"; do
    if [[ ! "${pose_value}" =~ ^-?[0-9]+([.][0-9]+)?$ ]]; then
      echo "[ERROR] Invalid simulation spawn coordinate: ${pose_value}" >&2
      exit 1
    fi
  done
  # Explicit launch arguments remain last and override these device-local defaults.
  set -- "x:=${SIM_SPAWN_POSE[0]}" "y:=${SIM_SPAWN_POSE[1]}" "yaw:=${SIM_SPAWN_POSE[2]}" "$@"
fi

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

for executable in lib/libgrinder_mid360_sensor.so lib/super_lio/super_lio_node lib/super_lio/relocation_node; do
  if [[ ! -f "${WORKSPACE}/devel/${executable}" ]]; then
    echo "[ERROR] Missing simulation component: ${executable}" >&2
    echo "[ERROR] Rebuild with: PROFILE=sim ./build_grinder_platform.sh" >&2
    exit 1
  fi
done

if ! command -v ffmpeg >/dev/null 2>&1; then
  echo "[WARN] ffmpeg is not installed; APP control will work, but local RTSP video will be unavailable." >&2
  echo "[WARN] Install it with: sudo apt-get install -y ffmpeg" >&2
fi

if ! python3 -c 'import scipy' >/dev/null 2>&1; then
  echo "[WARN] python3-scipy is not installed; the scheduler will use its fallback planner." >&2
  echo "[WARN] Install it with: sudo apt-get install -y python3-scipy" >&2
fi

for port in "${SL_LINKA_PORT}" "${SL_LINKA_CONTROL_PORT}" 8554; do
  if command -v ss >/dev/null 2>&1 && ss -ltn "sport = :${port}" | grep -q LISTEN; then
    echo "[ERROR] TCP port ${port} is already in use. Stop the previous simulator/device stack first." >&2
    exit 1
  fi
done

echo "[INFO] Starting X920 + virtual MID-360S + managed Super-LIO simulator"
echo "[INFO] World: ${GRINDER_SIM_WORLD:-${SIM_PACKAGE_PATH}/worlds/factory_floor.world}"
echo "[INFO] Start mapping in the APP; keep stationary until Super-LIO initializes."
echo "[INFO] APP protocol: tcp://0.0.0.0:${SL_LINKA_PORT}"
echo "[INFO] APP control: tcp://0.0.0.0:${SL_LINKA_CONTROL_PORT}"
echo "[INFO] Local RTSP:  rtsp://<host>:8554/left and /right"
exec roslaunch grinder_gazebo grinder_sim.launch "$@"
