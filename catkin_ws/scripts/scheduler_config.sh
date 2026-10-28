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
    --local-config "${GRINDER_SCHEDULER_LOCAL_CONFIG}" || return 1
  local channel_ports configured_data configured_control
  channel_ports="$(python3 "${workspace}/scripts/check_scheduler_config.py" \
    --default-config "${config_dir}/scheduler.yaml" \
    --local-config "${GRINDER_SCHEDULER_LOCAL_CONFIG}" --print-ports)" || return 1
  read -r configured_data configured_control <<< "${channel_ports}"
  SL_LINKA_PORT="${SL_LINKA_PORT:-${configured_data}}"
  SL_LINKA_CONTROL_PORT="${SL_LINKA_CONTROL_PORT:-${configured_control}}"
  if [[ ! "${SL_LINKA_PORT}" =~ ^[0-9]+$ || ! "${SL_LINKA_CONTROL_PORT}" =~ ^[0-9]+$ ]] ||
     (( SL_LINKA_PORT < 1 || SL_LINKA_PORT > 65535 || SL_LINKA_CONTROL_PORT < 1 || SL_LINKA_CONTROL_PORT > 65535 )) ||
     [[ "${SL_LINKA_PORT}" == "${SL_LINKA_CONTROL_PORT}" ]]; then
    echo "[ERROR] SL-LinkA data/control ports must be different integers in 1..65535" >&2
    return 1
  fi
  export SL_LINKA_PORT SL_LINKA_CONTROL_PORT
}
