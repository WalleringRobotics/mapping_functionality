#!/usr/bin/env bash
set -euo pipefail
record_repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export WR_MAPPING_ROOT="${WR_MAPPING_ROOT:-/mnt/nvme/mapping}"
export WR_MAPPING_MOUNT="${WR_MAPPING_MOUNT:-/mnt/nvme}"
record_config="${WR_MAPPING_CONFIG:-$record_repo/configs/oakd-ros.yaml}"
if [[ -n "${WR_MAPPING_ROS_SETUP:-}" ]]; then
  set +u
  source "$WR_MAPPING_ROS_SETUP"
  set -u
fi
record_args=()
[[ -z "${WR_MAPPING_DEVICE_ID:-}" ]] || record_args+=(--device-id "$WR_MAPPING_DEVICE_ID")
[[ "${WR_MAPPING_START_MAVROS:-false}" != true ]] || record_args+=(--start-mavros)
[[ "${WR_MAPPING_CAMERA_ONLY:-false}" != true ]] || record_args+=(--camera-only)
record_name="$(date -u +%Y%m%dT%H%M%SZ)-$(cat /proc/sys/kernel/random/uuid)"
exec bash "$record_repo/deploy/run-ros.sh" bash "$record_repo/deploy/record-rosbag.sh" \
  --config "$record_config" --output "$WR_MAPPING_ROOT/$record_name" \
  --require-mount "$WR_MAPPING_MOUNT" --duration "${WR_MAPPING_DURATION:-0}" \
  --warmup "${WR_MAPPING_WARMUP:-60}" --fcu-url "${WR_MAPPING_FCU_URL:-/dev/ttyUSB0:921600}" \
  "${record_args[@]}"
