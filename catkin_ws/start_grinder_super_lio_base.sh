#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="${WORKSPACE:-${SCRIPT_DIR}}"
LIVOX_SETUP="${LIVOX_SETUP:-}"
START_LIVOX="${START_LIVOX:-1}"
START_CHASSIS="${START_CHASSIS:-1}"
START_NAV="${START_NAV:-1}"
NAV_MAP_YAML="${NAV_MAP_YAML:-/home/neardi/work/Grinder/maps/grinder_map.yaml}"
SL_LINKA_PORT="${SL_LINKA_PORT:-8002}"
LOG_DIR="${WORKSPACE}/logs/super_lio_base_$(date +%Y%m%d_%H%M%S)"
mkdir -p "${LOG_DIR}"

source /opt/ros/noetic/setup.bash
source "${WORKSPACE}/devel/setup.bash"

detect_livox_setup() {
  if rospack find livox_ros_driver2 >/dev/null 2>&1; then
    return 0
  fi
  local candidate
  for candidate in \
    "${LIVOX_SETUP}" \
    "/home/neardi/Livox/ws_livox/devel/setup.bash" \
    "/home/neardi/ws_livox/devel/setup.bash" \
    "/home/neardi/livox_ws/devel/setup.bash"; do
    if [[ -n "${candidate}" && -f "${candidate}" ]]; then
      LIVOX_SETUP="${candidate}"
      source "${LIVOX_SETUP}"
      rospack find livox_ros_driver2 >/dev/null 2>&1 && return 0
    fi
  done
  return 1
}

PIDS=()
PGIDS=()

cleanup() {
  echo
  echo "[INFO] stopping Super-LIO base services..."
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
  setsid bash -lc "source /opt/ros/noetic/setup.bash; if [[ -f '${LIVOX_SETUP}' ]]; then source '${LIVOX_SETUP}'; fi; source '${WORKSPACE}/devel/setup.bash'; ${command}" \
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

wait_for_master() {
  local deadline=$((SECONDS + 20))
  until rosparam list >/dev/null 2>&1; do
    if (( SECONDS >= deadline )); then
      echo "[ERROR] ROS master not ready; check ${LOG_DIR}/roscore.log"
      exit 1
    fi
    sleep 1
  done
}

if bash -c ":</dev/tcp/127.0.0.1/${SL_LINKA_PORT}" >/dev/null 2>&1; then
  echo "[ERROR] TCP ${SL_LINKA_PORT} is already occupied; grinder_scheduler may already be running."
  exit 1
fi

echo "[INFO] logs: ${LOG_DIR}"

if ! pgrep -f "roscore" >/dev/null 2>&1; then
  launch_bg "roscore" "roscore"
fi
wait_for_master

if [[ "${START_LIVOX}" == "1" ]]; then
  if ! detect_livox_setup; then
    echo "[ERROR] livox_ros_driver2 is not available in the sourced ROS environments."
    echo "[ERROR] Set LIVOX_SETUP to the actual Livox workspace setup.bash,"
    echo "[ERROR] or start the Livox driver separately and rerun with START_LIVOX=0."
    exit 1
  fi
  local_livox_package="$(rospack find livox_ros_driver2)"
  local_livox_launch="$(find "${local_livox_package}" -maxdepth 3 -type f -name msg_MID360s.launch -print -quit)"
  if [[ -z "${local_livox_launch}" ]]; then
    echo "[ERROR] Could not locate livox_ros_driver2/msg_MID360s.launch under ${local_livox_package}."
    exit 1
  fi
  echo "[INFO] Livox driver package: ${local_livox_package}"
  python3 "${SCRIPT_DIR}/scripts/check_livox_udp_ports.py" "${local_livox_package}" "${local_livox_launch}"
  launch_bg "livox" "roslaunch livox_ros_driver2 msg_MID360s.launch"
fi

if [[ "${START_CHASSIS}" == "1" ]]; then
  launch_bg "chassis" "roslaunch grinder_chassis_driver chassis_driver.launch"
fi

if [[ "${START_NAV}" == "1" ]]; then
  launch_bg "move_base_rpp" "roslaunch teb_local_planner_tutorials robot_diff_drive.launch local_planner:=rpp"
fi

launch_bg "scheduler" "roslaunch grinder_scheduler scheduler.launch odom_topic:=/lio/odom map_topic:=/map navigation_map_yaml_path:=${NAV_MAP_YAML} super_lio_map_root:=/home/neardi/work/Grinder/maps"

echo
echo "[INFO] Super-LIO base services started."
echo "[INFO] Started: Livox (optional), chassis (optional), RPP move_base (optional), scheduler/SL-LinkA/RTSP and Super-LIO mode manager."
echo "[INFO] Mapping, relocation and map_server are intentionally not started."
echo "[INFO] Press Ctrl+C to stop processes started by this script."
wait
