#!/usr/bin/env bash
# One-shot readiness and metadata capture, managed as an action by ROS launch.
set -euo pipefail
bag_output="$1"
bag_camera_only="$2"
bag_px4_rate="${3:-0}"
# First messages, PX4 connection, 5 s of continuous camera and IMU data, driver
# parameters and the graph, in one ROS process: a CLI call per check took about
# 50 s on the Orin, against 6 s of device start-up.
bag_ready_args=()
[[ "$bag_camera_only" == false ]] || bag_ready_args=(--camera-only)
python3 "$bag_output/recording.py" --ready "$bag_output" "${bag_ready_args[@]}" \
  --timeout "${WR_MAPPING_READY_TIMEOUT:-60}" --solid-seconds "${WR_MAPPING_SOLID_SECONDS:-5}"
if [[ "$bag_camera_only" == false ]]; then
  # Record the mission PX4 holds for a checked survey plan (read-only download).
  if [[ -f "$bag_output/survey.plan" ]]; then
    python3 "$bag_output/recording.py" --pull-mission --report "$bag_output/mission-pull.json" ||
      echo "Mission download failed; survey legs will stay unverified" >&2
  fi
  if [[ "$bag_px4_rate" != 0 ]]; then
    python3 "$bag_output/recording.py" --rate "$bag_px4_rate" --report "$bag_output/px4-rate-request.json"
  fi
fi
# Preserve the vendor calibration dump without exposing identity in source files.
for calibration in /tmp/*_calibration.json; do
  if [[ -f "$calibration" && "$calibration" -nt "$bag_output/started-utc.txt" ]]; then
    cp "$calibration" "$bag_output/"
  fi
done
