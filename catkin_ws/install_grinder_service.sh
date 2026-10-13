#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUN_USER="${1:-${SUDO_USER:-}}"
if [[ -z "${RUN_USER}" || "${RUN_USER}" == root ]]; then
  echo "Usage: sudo bash $0 <ordinary-run-user>" >&2
  exit 1
fi
exec python3 "${SCRIPT_DIR}/scripts/grinder-service" install --workspace "${SCRIPT_DIR}" --user "${RUN_USER}"
