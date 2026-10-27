#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="${WORKSPACE:-${SCRIPT_DIR}}"
source "${SCRIPT_DIR}/scripts/scheduler_config.sh"
configure_scheduler_config "${WORKSPACE}" device
LIVOX_SETUP="${LIVOX_SETUP:-}"
START_LIVOX="${START_LIVOX:-1}"
START_CHASSIS="${START_CHASSIS:-1}"
START_NAV="${START_NAV:-1}"
START_INDICATOR="${START_INDICATOR:-0}"
INDICATOR_BACKEND="${INDICATOR_BACKEND:-serial}"
INDICATOR_USB_SERIAL_NUMBER="${INDICATOR_USB_SERIAL_NUMBER:-}"
INDICATOR_DEVICE="${INDICATOR_DEVICE:-}"
NAV_MAP_YAML="${NAV_MAP_YAML:-/home/neardi/work/Grinder/maps/grinder_map.yaml}"
SUPER_LIO_MAP_ROOT="${SUPER_LIO_MAP_ROOT:-${WORKSPACE}/../maps}"
SL_LINKA_PORT="${SL_LINKA_PORT:-8002}"
mkdir -p "${WORKSPACE}/logs"
exec 9>"${WORKSPACE}/logs/super_lio_base.lock"
if ! flock -n 9; then
  echo "[ERROR] Super-LIO stack is already running in this workspace." >&2
  exit 1
fi
LOG_DIR="${WORKSPACE}/logs/super_lio_base_$(date +%Y%m%d_%H%M%S)"
mkdir -p "${LOG_DIR}"

# Catkin/ROS setup files legitimately read unset environment variables in a
# fresh systemd session. Keep strict checking for our code, not their setup.
source_ros_setup() {
  local setup_status=0
  set +u
  source "$1" || setup_status=$?
  set -u
  return "${setup_status}"
}

source_ros_setup /opt/ros/noetic/setup.bash
source_ros_setup "${WORKSPACE}/devel/setup.bash"

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
      # Probe the external driver in a child shell: its setup must not replace
      # the Grinder supervisor's ROS package paths.
      if bash -c 'source "$1" && rospack find livox_ros_driver2' _ "${candidate}" >/dev/null 2>&1; then
        return 0
      fi
    fi
  done
  return 1
}

PIDS=()
PGIDS=()

cleanup() {
  trap - EXIT INT TERM
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
  for pid in "${PIDS[@]}"; do
    wait "${pid}" 2>/dev/null || true
  done
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

launch_bg() {
  local name="$1"
  shift
  local logfile="${LOG_DIR}/${name}.log"
  echo "[INFO] launch ${name}"
  # The environment is already sourced. Preserve each argument and release
  # the launcher lock in children; only the supervising shell owns it.
  setsid "$@" 9>&- >"${logfile}" 2>&1 &
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
  launch_bg "roscore" roscore
fi
wait_for_master

if [[ "${START_INDICATOR}" == "1" ]]; then
  launch_bg "indicator" roslaunch grinder_scheduler status_indicator.launch \
    "backend:=${INDICATOR_BACKEND}" "usb_serial_number:=${INDICATOR_USB_SERIAL_NUMBER}" \
    "device:=${INDICATOR_DEVICE}"
fi

if [[ "${START_LIVOX}" == "1" ]]; then
  if ! detect_livox_setup; then
    echo "[ERROR] livox_ros_driver2 is not available in the sourced ROS environments."
    echo "[ERROR] Set LIVOX_SETUP to the actual Livox workspace setup.bash,"
    echo "[ERROR] or start the Livox driver separately and rerun with START_LIVOX=0."
    exit 1
  fi
  livox_command=()
  if ! rospack find livox_ros_driver2 >/dev/null 2>&1; then
    livox_command=(bash -c 'source "$1" || exit; shift; exec "$@"' _ "${LIVOX_SETUP}")
  fi
  local_livox_package="$("${livox_command[@]}" rospack find livox_ros_driver2)"
  local_livox_launch="$(find "${local_livox_package}" -maxdepth 3 -type f -name msg_MID360s.launch -print -quit)"
  if [[ -z "${local_livox_launch}" ]]; then
    echo "[ERROR] Could not locate livox_ros_driver2/msg_MID360s.launch under ${local_livox_package}."
    exit 1
  fi
  echo "[INFO] Livox driver package: ${local_livox_package}"
  python3 "${SCRIPT_DIR}/scripts/check_livox_udp_ports.py" "${local_livox_package}" "${local_livox_launch}"
  launch_bg "livox" "${livox_command[@]}" roslaunch livox_ros_driver2 msg_MID360s.launch
fi

if [[ "${START_CHASSIS}" == "1" ]]; then
  launch_bg "chassis" roslaunch grinder_chassis_driver chassis_driver.launch
fi

if [[ "${START_NAV}" == "1" ]]; then
  launch_bg "move_base_rpp" roslaunch teb_local_planner_tutorials robot_diff_drive.launch local_planner:=rpp
fi

launch_bg "scheduler" roslaunch grinder_scheduler scheduler.launch odom_topic:=/lio/odom map_topic:=/map \
  "navigation_map_yaml_path:=${NAV_MAP_YAML}" "super_lio_map_root:=${SUPER_LIO_MAP_ROOT}"

echo
echo "[INFO] Super-LIO base services started."
echo "[INFO] Started: Livox (optional), chassis (optional), RPP move_base (optional), scheduler/SL-LinkA/RTSP and Super-LIO mode manager."
echo "[INFO] Mapping, relocation and map_server are intentionally not started."
echo "[INFO] Press Ctrl+C to stop processes started by this script."
# A required process exiting (even successfully) means the stack is incomplete.
# EXIT cleanup tears down the other processes and systemd reports failure.
wait -n || true
echo "[ERROR] A supervised process exited; stopping the stack." >&2
exit 1
