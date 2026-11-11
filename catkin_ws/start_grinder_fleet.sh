#!/usr/bin/env bash
set -euo pipefail
WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WORKSPACE"
if [[ -z "${DISPLAY:-}" && -S /mnt/wslg/.X11-unix/X0 ]]; then
  export DISPLAY=:0
fi
if [[ ! -f "$WORKSPACE/devel/setup.bash" ]]; then
  echo '[ERROR] Build the simulation first: PROFILE=sim ./build_grinder_platform.sh' >&2
  exit 1
fi
set +u
source /opt/ros/noetic/setup.bash
source "$WORKSPACE/devel/setup.bash"
set -u
exec rosrun grinder_gazebo start_fleet.py --workspace "$WORKSPACE" "$@"
