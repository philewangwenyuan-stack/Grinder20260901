#!/usr/bin/env bash
# Compatibility entrypoint; mapping and simulation share the ROS1 dependencies.
set -euo pipefail
exec bash "$(dirname "${BASH_SOURCE[0]}")/build_livox_ros1.sh"
