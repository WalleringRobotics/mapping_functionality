#!/usr/bin/env bash
set -euo pipefail
record_repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
record_root="${WR_MAPPING_ROOT:-/mnt/nvme/mapping}"
record_mount="${WR_MAPPING_MOUNT:-/mnt/nvme}"
record_config="${WR_MAPPING_CONFIG:-$record_repo/configs/oakd-survey.json}"
record_python="${WR_MAPPING_PYTHON:-$record_repo/.venv/bin/python}"
if [[ -n "${WR_MAPPING_ROS_SETUP:-}" ]]; then
  # ROS-generated setup scripts can read unset variables.
  set +u
  source "$WR_MAPPING_ROS_SETUP"
  if [[ -n "${WR_MAPPING_ROS_OVERLAY:-}" ]]; then
    source "$WR_MAPPING_ROS_OVERLAY"
  fi
  set -u
fi
export PYTHONPATH="$record_repo/src${PYTHONPATH:+:$PYTHONPATH}"
record_args=()
check_args=()
if [[ -n "${WR_MAPPING_TELEMETRY_CONFIG:-}" ]]; then
  record_args+=(--telemetry-config "$WR_MAPPING_TELEMETRY_CONFIG")
  check_args+=(--telemetry-config "$WR_MAPPING_TELEMETRY_CONFIG")
fi
if [[ -n "${WR_MAPPING_DEVICE_ID:-}" ]]; then
  record_args+=(--device-id "$WR_MAPPING_DEVICE_ID")
  check_args+=(--device-id "$WR_MAPPING_DEVICE_ID")
fi
if [[ -n "${WR_MAPPING_GNSS_PROFILE:-}" ]]; then
  check_args+=(--gnss-profile "$WR_MAPPING_GNSS_PROFILE")
fi
record_name="$(date -u +%Y%m%dT%H%M%SZ)-$(cat /proc/sys/kernel/random/uuid)"
if [[ -d "$record_root" ]]; then
  check_args+=(--report "$record_root/$record_name-hardware.json")
fi
"$record_python" -m wallering_mapping.cli hardware-check --require-jetson \
  --config "$record_config" --output-root "$record_root" --require-mount "$record_mount" \
  "${check_args[@]}"
exec "$record_python" -m wallering_mapping.cli record \
  --config "$record_config" --output "$record_root/$record_name" "${record_args[@]}"
