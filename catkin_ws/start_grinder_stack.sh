#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="${WORKSPACE:-${SCRIPT_DIR}}"
LOG_DIR="${WORKSPACE}/logs/startup_latest"
rm -rf "${LOG_DIR}"
mkdir -p "${LOG_DIR}"

# Runtime options (can be overridden by env before running this script)
AURORA_IP="${AURORA_IP:-192.168.0.102}"
START_NAV="${START_NAV:-1}"
START_CHASSIS="${START_CHASSIS:-1}"
START_RF2O="${START_RF2O:-1}"
START_EKF_TEST="${START_EKF_TEST:-1}"
RESET_STATE="${RESET_STATE:-0}"
NAV_MAP_YAML="${NAV_MAP_YAML:-${WORKSPACE}/src/2-dnavigation-package/2dnavigation/teb_local_planner_tutorials/maps/map.yaml}"
SL_LINKA_PORT="${SL_LINKA_PORT:-8002}"
SL_LINKA_READY_TIMEOUT="${SL_LINKA_READY_TIMEOUT:-45}"
HOST_ARCH="$(uname -m)"
MEDIAMTX_BIN="${MEDIAMTX_BIN:-}"

detect_mediamtx_path() {
  if [[ -n "${MEDIAMTX_BIN}" ]]; then
    echo "${MEDIAMTX_BIN}"
    return 0
  fi
  local base="${WORKSPACE}/../tools/mediamtx"
  local common="${base}/mediamtx"
  local arch_candidate=""
  case "${HOST_ARCH}" in
    aarch64|arm64) arch_candidate="${base}/mediamtx_aarch64" ;;
    x86_64|amd64) arch_candidate="${base}/mediamtx_x86_64" ;;
  esac
  if [[ -n "${arch_candidate}" && -x "${arch_candidate}" ]]; then
    echo "${arch_candidate}"
    return 0
  fi
  echo "${common}"
  return 0
}

PIDS=()
PGIDS=()

cleanup() {
  echo
  echo "[INFO] stopping all launched processes..."
  for pgid in "${PGIDS[@]:-}"; do
    if kill -0 -- "-${pgid}" 2>/dev/null; then
      kill -- "-${pgid}" 2>/dev/null || true
    fi
  done
  sleep 3
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
  local cmd="$2"
  local logfile="${LOG_DIR}/${name}.log"
  echo "[INFO] launch ${name}"
  echo "       ${cmd}"
  setsid bash -lc "source /opt/ros/noetic/setup.bash && source ${WORKSPACE}/devel/setup.bash && ${cmd}" \
    >"${logfile}" 2>&1 &
  local pid=$!
  PIDS+=("${pid}")
  PGIDS+=("${pid}")
  sleep 2
  if ! kill -0 "${pid}" 2>/dev/null; then
    echo "[ERROR] ${name} exited early, check ${logfile}"
    exit 1
  fi
}

wait_for_tcp_port() {
  local name="$1"
  local host="$2"
  local port="$3"
  local timeout_s="$4"
  local start_ts
  start_ts="$(date +%s)"
  echo "[INFO] waiting for ${name} on ${host}:${port} ..."
  while true; do
    if bash -c ":</dev/tcp/${host}/${port}" >/dev/null 2>&1; then
      echo "[INFO] ${name} is ready on ${host}:${port}"
      return 0
    fi
    if (( $(date +%s) - start_ts >= timeout_s )); then
      echo "[ERROR] ${name} not ready on ${host}:${port} within ${timeout_s}s"
      echo "[ERROR] check ${LOG_DIR}/grinder_system.log"
      return 1
    fi
    sleep 1
  done
}

tcp_port_open() {
  local host="$1"
  local port="$2"
  bash -c ":</dev/tcp/${host}/${port}" >/dev/null 2>&1
}

echo "[INFO] logs: ${LOG_DIR}"
echo "[INFO] host arch: ${HOST_ARCH}"
if tcp_port_open "127.0.0.1" "${SL_LINKA_PORT}"; then
  echo "[ERROR] SL-LinkA port ${SL_LINKA_PORT} is already open before startup."
  echo "[ERROR] A previous grinder_scheduler may still be running. Please stop stale ROS processes before launching again."
  echo "[ERROR] Useful checks: pgrep -af 'scheduler_node.py|grinder_scheduler|roslaunch'"
  exit 1
fi
if [[ "${RESET_STATE}" == "1" ]]; then
  STATE_DIR="${WORKSPACE}/../temp/grinder_scheduler_state"
  echo "[INFO] RESET_STATE=1, clearing persisted scheduler state: ${STATE_DIR}"
  rm -rf "${STATE_DIR}"
fi

SELECTED_MEDIAMTX="$(detect_mediamtx_path)"
if [[ ! -x "${SELECTED_MEDIAMTX}" ]]; then
  echo "[WARN] mediamtx binary not executable: ${SELECTED_MEDIAMTX}"
fi
if command -v file >/dev/null 2>&1 && [[ -f "${SELECTED_MEDIAMTX}" ]]; then
  MEDIAMTX_FILE_DESC="$(file "${SELECTED_MEDIAMTX}" || true)"
  echo "[INFO] mediamtx: ${SELECTED_MEDIAMTX}"
  echo "[INFO] mediamtx file: ${MEDIAMTX_FILE_DESC}"
fi

# 1) roscore
if ! pgrep -f "roscore" >/dev/null 2>&1; then
  launch_bg "roscore" "roscore"
else
  echo "[INFO] roscore already running, skip."
fi

# 2) full grinder system launch (aurora + chassis + scheduler + optional navigation)
if [[ "${START_CHASSIS}" == "1" ]]; then
  CHASSIS_ARG="start_chassis_driver:=true"
else
  CHASSIS_ARG="start_chassis_driver:=false"
fi

if [[ "${START_NAV}" == "1" ]]; then
  launch_bg "grinder_system" "roslaunch grinder_scheduler grinder_system.launch aurora_ip_address:=${AURORA_IP} ${CHASSIS_ARG} start_navigation:=true navigation_map_yaml_path:=${NAV_MAP_YAML} local_rtsp_mediamtx_path:=${SELECTED_MEDIAMTX}"
else
  launch_bg "grinder_system" "roslaunch grinder_scheduler grinder_system.launch aurora_ip_address:=${AURORA_IP} ${CHASSIS_ARG} start_navigation:=false navigation_map_yaml_path:=${NAV_MAP_YAML} local_rtsp_mediamtx_path:=${SELECTED_MEDIAMTX}"
fi

# 3) Parallel laser odometry and EKF test. Neither node publishes odom->base_link,
# so the existing Aurora localization and navigation stack remain unchanged.
if [[ "${START_RF2O}" == "1" ]]; then
  launch_bg "rf2o_laser_odometry" "roslaunch rf2o_laser_odometry rf2o_laser_odometry.launch"
else
  echo "[INFO] START_RF2O=0, skip RF2O laser odometry."
fi

if [[ "${START_EKF_TEST}" == "1" ]]; then
  launch_bg "ekf_test" "roslaunch rf2o_laser_odometry ekf.launch"
else
  echo "[INFO] START_EKF_TEST=0, skip EKF test."
fi

wait_for_tcp_port "SL-LinkA service" "127.0.0.1" "${SL_LINKA_PORT}" "${SL_LINKA_READY_TIMEOUT}"

echo
echo "[INFO] grinder stack started."
echo "[INFO] use Ctrl+C in this terminal to stop all processes started by this script."
echo "[INFO] logs are in ${LOG_DIR}"

wait
