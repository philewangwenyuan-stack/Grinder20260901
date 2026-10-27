#!/usr/bin/env bash
# Local source -> dedicated remote mirror. Run this client inside local WSL.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SOURCE_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
REMOTE_HOST="${GRINDER_REMOTE_HOST:-172.20.22.114}"
REMOTE_USER="${GRINDER_REMOTE_USER:-dev}"
REMOTE_PORT="${GRINDER_REMOTE_PORT:-2222}"
REMOTE_ROOT="${GRINDER_REMOTE_ROOT:-/home/${REMOTE_USER}/work/Grinder-remote-sim}"
IDENTITY="${GRINDER_REMOTE_KEY:-${HOME}/.ssh/grinder_remote_sim}"
ACTION="${1:-help}"
[[ "$#" == 0 ]] || shift
die() { echo "[ERROR] $*" >&2; exit 1; }
[[ "${REMOTE_HOST}" =~ ^[a-zA-Z0-9][a-zA-Z0-9.-]*$ ]] || die 'Invalid host'
[[ "${REMOTE_USER}" =~ ^[a-zA-Z_][a-zA-Z0-9_-]*$ ]] || die 'Invalid user'
[[ "${REMOTE_PORT}" =~ ^[0-9]+$ ]] && (( REMOTE_PORT > 0 && REMOTE_PORT < 65536 )) || die 'Invalid port'
[[ "${REMOTE_ROOT}" =~ ^/home/${REMOTE_USER}/[a-zA-Z0-9_./-]+$ && "${REMOTE_ROOT}" != *'/../'* && "${REMOTE_ROOT}" != */.. ]] || die 'Mirror must be an absolute directory below the remote user home'
SSH_FLAGS=(-i "${IDENTITY}" -p "${REMOTE_PORT}" -o IdentitiesOnly=yes
  -o ConnectTimeout=8 -o ServerAliveInterval=15 -o ServerAliveCountMax=3
  -o StrictHostKeyChecking=accept-new)
TARGET="${REMOTE_USER}@${REMOTE_HOST}"
quote() { printf '%q' "$1"; }
remote() { ssh "${SSH_FLAGS[@]}" "${TARGET}" "bash -lc $(quote "$1")"; }
check_client() {
  command -v ssh >/dev/null || die 'Install openssh-client in local WSL'
  [[ -f "${IDENTITY}" ]] || die "SSH key missing: ${IDENTITY}"
}
mirror_check() {
  remote "set -e; test -f '${REMOTE_ROOT}/.grinder-source-mirror'; test \"\$(cat '${REMOTE_ROOT}/.grinder-source-mirror')\" = $(quote "${SOURCE_ROOT}")"
}
sync_source() {
  command -v rsync >/dev/null || die 'Install rsync in local WSL'
  remote "set -e; test \"\$(realpath -m '${REMOTE_ROOT}')\" = '${REMOTE_ROOT}' || { echo 'Mirror path must be canonical and must not traverse symlinks' >&2; exit 1; }; if test -d '${REMOTE_ROOT}' && test ! -f '${REMOTE_ROOT}/.grinder-source-mirror' && test -n \"\$(ls -A '${REMOTE_ROOT}')\"; then echo 'Refusing to overwrite an unmarked nonempty directory' >&2; exit 1; fi; mkdir -p '${REMOTE_ROOT}'; if test -f '${REMOTE_ROOT}/.grinder-source-mirror'; then test \"\$(cat '${REMOTE_ROOT}/.grinder-source-mirror')\" = $(quote "${SOURCE_ROOT}"); else printf '%s\\n' $(quote "${SOURCE_ROOT}") > '${REMOTE_ROOT}/.grinder-source-mirror'; fi"
  local transport
  printf -v transport '%q ' ssh "${SSH_FLAGS[@]}"
  # Preserve remote-only setup tools and runtime files across source branches.
  rsync -az --safe-links --stats -e "${transport}" \
    --exclude='.git/' --exclude='.git.nested-backup-*/' \
    --exclude='/.grinder-source-mirror' --exclude='/.planning/' \
    --exclude='/docs/' --exclude='/artifacts/' --exclude='/maps/' \
    --exclude='/task_plan.md' --exclude='/findings.md' --exclude='/progress.md' \
    --exclude='.venv/' --exclude='venv/' --exclude='node_modules/' --exclude='.gradle/' \
    --exclude='build/' --exclude='build-*/' --exclude='build_wsl_*/' --exclude='build.bak*/' --exclude='devel/' \
    --exclude='devel.bak*/' --exclude='install/' --exclude='logs/' --exclude='temp/' \
    --include='/catkin_ws/src/2-dnavigation-package/aurora_remote_public/lib/***' \
    --exclude='__pycache__/' --exclude='*.pyc' --exclude='*.so' --exclude='*.so.*' \
    --exclude='*.pcd' --exclude='*.stcm' \
    --exclude='*.a' --exclude='scheduler.local.yaml' \
    "${SOURCE_ROOT}/" "${TARGET}:${REMOTE_ROOT}/"
  echo '[OK] Source synced. Restart affected nodes after changing Python/launch files.'
}
case "${ACTION}" in
  help)
    echo 'remote_sim.sh doctor|sync|watch|build|start|rviz|shell'
    echo 'Defaults: dev@172.20.22.114:2222, /home/dev/work/Grinder-remote-sim'
    echo 'Use remote_sim.ps1 tunnel/app in Windows for APP/MuMu.'
    exit 0 ;;
  doctor)
    check_client
    remote "hostname; whoami; uname -m; nproc; command -v rsync; for x in /opt/ros/noetic/setup.bash /mnt/wslg/.X11-unix/X0; do test ! -e \"\$x\" || ls -ld \"\$x\"; done; command -v gzserver || true" ;;
  sync) check_client; sync_source ;;
  watch)
    check_client
    SSH_FLAGS+=(-o BatchMode=yes)
    interval="${GRINDER_REMOTE_SYNC_INTERVAL:-3}"
    [[ "${interval}" =~ ^[1-9][0-9]*$ ]] || die 'Sync interval must be positive integer seconds'
    echo '[INFO] Watching by incremental sync; Ctrl+C stops. No automatic build/restart.'
    while true; do sync_source; sleep "${interval}"; done ;;
  build)
    check_client; sync_source
    jobs="${GRINDER_REMOTE_BUILD_JOBS:-8}"
    [[ "${jobs}" =~ ^[1-9][0-9]*$ ]] || die 'Build jobs must be a positive integer'
    remote "set -e; cd '${REMOTE_ROOT}/catkin_ws'; PROFILE=sim CATKIN_JOBS=${jobs} CATKIN_LOAD=${jobs} bash ./build_grinder_platform.sh; cd '${REMOTE_ROOT}'; cmake -S third_party/path_planner/mst27 -B temp/remote_mst27 -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3; cmake --build temp/remote_mst27 -j${jobs}" ;;
  start)
    check_client; mirror_check
    args=''
    for arg in "$@"; do args+=" $(quote "${arg}")"; done
    # Gazebo cameras render on the remote WSLg display, not over the network.
    remote "set -e; cd '${REMOTE_ROOT}/catkin_ws'; export DISPLAY=\${GRINDER_REMOTE_DISPLAY:-:0}; mkdir -p '${REMOTE_ROOT}/temp/remote_sim'; printf '%s\\n' 'local_rtsp_public_host: 127.0.0.1' > '${REMOTE_ROOT}/temp/remote_sim/scheduler.local.yaml'; exec bash ./start_grinder_sim.sh gui:=false scheduler_local_config:='${REMOTE_ROOT}/temp/remote_sim/scheduler.local.yaml'${args}" ;;
  rviz)
    check_client; mirror_check
    [[ -n "${DISPLAY:-}" ]] || die 'Local WSLg DISPLAY is missing'
    args=''
    if [[ "$#" == 0 ]]; then
      args=" -d '${REMOTE_ROOT}/catkin_ws/src/grinder_gazebo/config/remote_sim.rviz'"
    fi
    for arg in "$@"; do args+=" $(quote "${arg}")"; done
    command="set -e; test -n \"\${DISPLAY:-}\" || { echo 'Enable X11Forwarding and install xauth on remote sshd' >&2; exit 1; }; source /opt/ros/noetic/setup.bash; source '${REMOTE_ROOT}/catkin_ws/devel/setup.bash'; export QT_QPA_PLATFORM=xcb QT_X11_NO_MITSHM=1 LIBGL_ALWAYS_SOFTWARE=1; exec rosrun rviz rviz${args}"
    ssh -Y "${SSH_FLAGS[@]}" "${TARGET}" "bash -lc $(quote "${command}")" ;;
  shell)
    check_client
    command="set -e; cd '${REMOTE_ROOT}'; if test -f /opt/ros/noetic/setup.bash; then source /opt/ros/noetic/setup.bash; fi; if test -f '${REMOTE_ROOT}/catkin_ws/devel/setup.bash'; then source '${REMOTE_ROOT}/catkin_ws/devel/setup.bash'; fi; exec bash -i"
    exec ssh -t "${SSH_FLAGS[@]}" "${TARGET}" "bash -lc $(quote "${command}")" ;;
  *) die "Unknown action: ${ACTION}" ;;
esac
