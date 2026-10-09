#!/usr/bin/env bash
# Sourced by startup scripts; no ROS nodes or hardware are started here.

configure_scheduler_config() {
  local workspace="$1"
  local profile="$2"
  local config_dir="${workspace}/src/grinder_scheduler/config"

  # Preserve an explicitly empty variable as an opt-out. Device scripts discover
  # the migrated local file; simulation never discovers production credentials.
  if [[ -z "${GRINDER_SCHEDULER_LOCAL_CONFIG+x}" ]]; then
    GRINDER_SCHEDULER_LOCAL_CONFIG=""
    if [[ "${profile}" == "device" && -f "${config_dir}/scheduler.local.yaml" ]]; then
      GRINDER_SCHEDULER_LOCAL_CONFIG="${config_dir}/scheduler.local.yaml"
    fi
  fi
  if [[ -n "${GRINDER_SCHEDULER_LOCAL_CONFIG}" ]]; then
    GRINDER_SCHEDULER_LOCAL_CONFIG="$(realpath -m -- "${GRINDER_SCHEDULER_LOCAL_CONFIG}")"
  fi
  export GRINDER_SCHEDULER_LOCAL_CONFIG

  python3 "${workspace}/scripts/check_scheduler_config.py" \
    --default-config "${config_dir}/scheduler.yaml" \
    --local-config "${GRINDER_SCHEDULER_LOCAL_CONFIG}"
}
