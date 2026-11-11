#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export GAZEBO_MODEL_PATH="$ROOT/models${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"
export GAZEBO_MODEL_DATABASE_URI=""
if [[ "${1:-}" == "--headless" ]]; then
  shift
  exec gzserver --verbose -r "$ROOT/worlds/huadu_factory.world" "$@"
fi
exec gazebo --verbose -r "$ROOT/worlds/huadu_factory.world" "$@"
