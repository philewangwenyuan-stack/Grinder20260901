#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="${WS_DIR:-${SCRIPT_DIR}}"

PROFILE="${PROFILE:-full}"                # full|runtime|nav|aurora|scheduler|chassis|sim
CLEAN_ON_ARCH_CHANGE="${CLEAN_ON_ARCH_CHANGE:-1}"
CLEAN_ON_PATH_CHANGE="${CLEAN_ON_PATH_CHANGE:-1}"
SKIP_G2O="${SKIP_G2O:-0}"

WS_DIR="$(cd "${WS_DIR}" && pwd -P)"

HOST_ARCH="$(uname -m)"
HOST_OS="$(uname -s)"

if [[ "${HOST_OS}" != "Linux" ]]; then
  echo "[ERROR] Only Linux is supported by this script. Current: ${HOST_OS}"
  exit 1
fi

case "${HOST_ARCH}" in
  x86_64) ARCH_TAG="x86_64" ;;
  aarch64|arm64) ARCH_TAG="aarch64" ;;
  *)
    echo "[ERROR] Unsupported architecture: ${HOST_ARCH}. Expected x86_64 or aarch64."
    exit 1
    ;;
esac

if [[ -z "${CATKIN_JOBS:-}" ]]; then
  if [[ "${ARCH_TAG}" == "aarch64" ]]; then
    CATKIN_JOBS=2
    CATKIN_LOAD=2
  else
    CATKIN_JOBS="$(nproc)"
    CATKIN_LOAD="$(nproc)"
  fi
else
  CATKIN_LOAD="${CATKIN_LOAD:-${CATKIN_JOBS}}"
fi

echo "[INFO] workspace: ${WS_DIR}"
echo "[INFO] host arch: ${HOST_ARCH} -> ${ARCH_TAG}"
echo "[INFO] profile: ${PROFILE}"
echo "[INFO] catkin jobs: -j${CATKIN_JOBS} -l${CATKIN_LOAD}"

BUILD_CACHE="${WS_DIR}/build/CMakeCache.txt"

backup_catkin_spaces() {
  local reason_tag="$1"
  local timestamp
  timestamp="$(date +%Y%m%d_%H%M%S)"
  for directory in build devel install; do
    local source_path="${WS_DIR}/${directory}"
    local backup_path="${WS_DIR}/${directory}.bak.${reason_tag}.${timestamp}"
    if [[ -e "${source_path}" ]]; then
      mv "${source_path}" "${backup_path}"
      echo "[INFO] moved ${directory} -> $(basename "${backup_path}")"
    fi
  done
}

if [[ -f "${BUILD_CACHE}" ]]; then
  EXPECTED_SOURCE="$(cd "${WS_DIR}/src" && pwd -P)"
  PREV_SOURCE="$(grep -E '^CMAKE_HOME_DIRECTORY:INTERNAL=' "${BUILD_CACHE}" | sed 's/^[^=]*=//' || true)"
  if [[ -n "${PREV_SOURCE}" && "${PREV_SOURCE}" != "${EXPECTED_SOURCE}" ]]; then
    if [[ "${CLEAN_ON_PATH_CHANGE}" == "1" ]]; then
      echo "[WARN] build cache source path mismatch:"
      echo "[WARN]   cached:  ${PREV_SOURCE}"
      echo "[WARN]   current: ${EXPECTED_SOURCE}"
      backup_catkin_spaces "path-mismatch"
    else
      echo "[ERROR] build cache was created from another workspace: ${PREV_SOURCE}"
      echo "[ERROR] set CLEAN_ON_PATH_CHANGE=1 or move build/devel/install aside."
      exit 1
    fi
  fi
fi

if [[ -f "${BUILD_CACHE}" ]]; then
  PREV_ARCH="$(grep -E '^CMAKE_SYSTEM_PROCESSOR:INTERNAL=' "${BUILD_CACHE}" | sed 's/.*=//' || true)"
  if [[ -n "${PREV_ARCH}" && "${PREV_ARCH}" != "${HOST_ARCH}" && "${PREV_ARCH}" != "${ARCH_TAG}" ]]; then
    if [[ "${CLEAN_ON_ARCH_CHANGE}" == "1" ]]; then
      echo "[WARN] build cache architecture mismatch: ${PREV_ARCH} -> ${HOST_ARCH}"
      backup_catkin_spaces "arch-${PREV_ARCH}"
    else
      echo "[ERROR] build cache architecture mismatch: ${PREV_ARCH} -> ${HOST_ARCH}"
      echo "[ERROR] set CLEAN_ON_ARCH_CHANGE=1 or manually clean build/devel/install."
      exit 1
    fi
  fi
fi

run_catkin_pkg() {
  local pkg="$1"
  echo
  echo "[INFO] building package whitelist: ${pkg}"
  (
    cd "${WS_DIR}"
    set +u
    source /opt/ros/noetic/setup.bash
    set -u
    catkin_make -j"${CATKIN_JOBS}" -l"${CATKIN_LOAD}" -DCATKIN_WHITELIST_PACKAGES="${pkg}"
  )
}

run_catkin_with_deps() {
  echo
  echo "[INFO] building packages with dependencies: $*"
  (
    cd "${WS_DIR}"
    set +u
    source /opt/ros/noetic/setup.bash
    set -u
    catkin_make --only-pkg-with-deps "$@" -j"${CATKIN_JOBS}" -l"${CATKIN_LOAD}"
  )
}

build_nav_stack() {
  local nav_pkgs=(
    "voxel_grid"
    "costmap_2d"
    "nav_core"
    "base_local_planner"
    "carrot_planner"
    "clear_costmap_recovery"
    "rotate_recovery"
    "navfn"
    "global_planner"
    "dwa_local_planner"
    "map_server"
    "base_global_planner"
    "teb_local_planner"
    "regulated_pure_pursuit_controller"
    "teb_local_planner_tutorials"
    "move_base"
  )
  for p in "${nav_pkgs[@]}"; do
    run_catkin_pkg "${p}"
  done
}

build_aurora_pkg() {
  run_catkin_pkg "slamware_ros_sdk"
}

if [[ "${SKIP_G2O}" != "1" && ( "${PROFILE}" == "full" || "${PROFILE}" == "nav" ) ]]; then
  G2O_MAKE_SH="${WS_DIR}/src/2-dnavigation-package/3rdparty/g2omake.sh"
  if [[ -f "${G2O_MAKE_SH}" ]]; then
    echo
    echo "[INFO] ensuring g2o dependency via ${G2O_MAKE_SH}"
    bash "${G2O_MAKE_SH}" || true
  fi
fi

case "${PROFILE}" in
  full)
    run_catkin_pkg "grinder_chassis_driver"
    build_aurora_pkg
    run_catkin_pkg "grinder_scheduler"
    build_nav_stack
    ;;
  runtime)
    run_catkin_pkg "grinder_chassis_driver"
    build_aurora_pkg
    run_catkin_pkg "grinder_scheduler"
    ;;
  nav)
    build_nav_stack
    ;;
  aurora)
    build_aurora_pkg
    ;;
  scheduler)
    run_catkin_pkg "grinder_scheduler"
    ;;
  chassis)
    run_catkin_pkg "grinder_chassis_driver"
    ;;
  sim)
    run_catkin_with_deps \
      grinder_gazebo \
      grinder_scheduler \
      move_base \
      carrot_planner \
      regulated_pure_pursuit_controller
    ;;
  *)
    echo "[ERROR] unknown PROFILE=${PROFILE}"
    echo "[ERROR] valid values: full|runtime|nav|aurora|scheduler|chassis|sim"
    exit 1
    ;;
esac

echo
echo "[INFO] build finished for profile=${PROFILE} arch=${ARCH_TAG}"
echo "[INFO] source ${WS_DIR}/devel/setup.bash before running nodes."
