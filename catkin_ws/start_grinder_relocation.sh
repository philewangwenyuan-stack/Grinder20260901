#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="${WORKSPACE:-${SCRIPT_DIR}}"
MAP_DIR="${MAP_DIR:-/home/neardi/work/Grinder/maps}"
MAP_NAME="${MAP_NAME:-grinder_map}"
MAP_YAML="${MAP_YAML:-${MAP_DIR}/${MAP_NAME}.yaml}"
LOC_PCD="${LOC_PCD:-loc_${MAP_NAME}.pcd}"
RVIZ="${RVIZ:-true}"
FORCE_STOP_MAPPING="${FORCE_STOP_MAPPING:-0}"
BASE_TO_LASER_X="${BASE_TO_LASER_X:-0.0}"
BASE_TO_LASER_Y="${BASE_TO_LASER_Y:-0.0}"
BASE_TO_LASER_Z="${BASE_TO_LASER_Z:-0.0}"
BASE_TO_LASER_ROLL="${BASE_TO_LASER_ROLL:-0.0}"
BASE_TO_LASER_PITCH="${BASE_TO_LASER_PITCH:-0.0}"
BASE_TO_LASER_YAW="${BASE_TO_LASER_YAW:-0.0}"
LOG_DIR="${WORKSPACE}/logs/relocation_$(date +%Y%m%d_%H%M%S)"
mkdir -p "${LOG_DIR}"

source /opt/ros/noetic/setup.bash
source "${WORKSPACE}/devel/setup.bash"

PIDS=()
PGIDS=()

cleanup() {
  echo
  echo "[INFO] stopping relocation-mode processes..."
  for pgid in "${PGIDS[@]:-}"; do
    if kill -0 -- "-${pgid}" 2>/dev/null; then
      kill -- "-${pgid}" 2>/dev/null || true
    fi
  done
  sleep 2
  for pgid in "${PGIDS[@]:-}"; do
    if kill -0 -- "-${pgid}" 2>/dev/null; then
      kill -9 -- "-${pgid}" 2>/dev/null || true
    fi
  done
}

trap 'cleanup; exit 130' INT
trap 'cleanup; exit 143' TERM

launch_bg() {
  local name="$1"
  local command="$2"
  local logfile="${LOG_DIR}/${name}.log"
  echo "[INFO] launch ${name}"
  echo "       ${command}"
  setsid bash -lc "source /opt/ros/noetic/setup.bash; source '${WORKSPACE}/devel/setup.bash'; ${command}" \
    >"${logfile}" 2>&1 &
  local pid=$!
  PIDS+=("${pid}")
  PGIDS+=("${pid}")
  sleep 2
  if ! kill -0 "${pid}" 2>/dev/null; then
    echo "[ERROR] ${name} exited early; check ${logfile}"
    exit 1
  fi
}

if ! rosparam list >/dev/null 2>&1; then
  echo "[ERROR] ROS master is not available. Start start_grinder_super_lio_base.sh first."
  exit 1
fi

if [[ ! -r "${MAP_YAML}" ]]; then
  echo "[ERROR] map YAML not readable: ${MAP_YAML}"
  exit 1
fi
if [[ ! -r "${MAP_DIR}/${LOC_PCD}" ]]; then
  echo "[ERROR] localization PCD not readable: ${MAP_DIR}/${LOC_PCD}"
  exit 1
fi

MAPPING_NODES=()
for node in /super_lio_node /cloud_to_occupancy_grid /super_lio_loop; do
  if rosnode list 2>/dev/null | grep -Fxq "${node}"; then
    MAPPING_NODES+=("${node}")
  fi
done
if (( ${#MAPPING_NODES[@]} > 0 )); then
  if [[ "${FORCE_STOP_MAPPING}" != "1" ]]; then
    echo "[ERROR] mapping nodes are still running: ${MAPPING_NODES[*]}"
    echo "[ERROR] Save the map first, then stop them; or rerun with FORCE_STOP_MAPPING=1."
    exit 1
  fi
  echo "[WARN] stopping mapping nodes: ${MAPPING_NODES[*]}"
  rosnode kill "${MAPPING_NODES[@]}" >/dev/null
  sleep 2
fi

for node in /map_server /relocation_node; do
  if rosnode list 2>/dev/null | grep -Fxq "${node}"; then
    echo "[ERROR] ${node} is already running. Stop the previous relocation mode first."
    exit 1
  fi
done

launch_bg "map_server" "rosrun map_server map_server '${MAP_YAML}'"
launch_bg "relocation" "roslaunch super_lio relocation.launch rviz:=${RVIZ} map_dir:='${MAP_DIR}' map_name:='${LOC_PCD}' base_to_laser_x:=${BASE_TO_LASER_X} base_to_laser_y:=${BASE_TO_LASER_Y} base_to_laser_z:=${BASE_TO_LASER_Z} base_to_laser_roll:=${BASE_TO_LASER_ROLL} base_to_laser_pitch:=${BASE_TO_LASER_PITCH} base_to_laser_yaw:=${BASE_TO_LASER_YAW}"

echo
echo "[INFO] relocation mode started."
echo "[INFO] map: ${MAP_YAML}"
echo "[INFO] localization cloud: ${MAP_DIR}/${LOC_PCD}"
echo "[INFO] Publish /initialpose with RViz '2D Pose Estimate' to finish initialization."
echo "[INFO] Verify: rosrun tf tf_echo map base_link"
echo "[INFO] Logs: ${LOG_DIR}"
echo "[INFO] Press Ctrl+C to stop map_server and relocation only."
wait
