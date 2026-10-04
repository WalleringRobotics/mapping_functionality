#!/usr/bin/env bash
# Run only after ROS launch and rosbag2 have exited. Never stop processes here.
set -euo pipefail
bag_output="$1"
bag_result="${2:-0}"
trap 'printf "%s\n" failed > "$bag_output/state"' EXIT
[[ "$bag_result" == 0 && ! -e "$bag_output/failure.txt" &&
   -s "$bag_output/acquisition-start-ns.txt" && -s "$bag_output/acquisition-end-ns.txt" &&
   -f "$bag_output/bag/metadata.yaml" &&
   "$(cat "$bag_output/recorder-exit-code.txt")" == 0 ]] || {
  echo "Recording failed; preserved at $bag_output" >&2; exit 2;
}
printf '%s\n' finalizing > "$bag_output/state"
date -u --iso-8601=ns > "$bag_output/finished-utc.txt"
ros2 bag info "$bag_output/bag" > "$bag_output/bag-info.txt"
(cd "$bag_output" && find . -type f ! -name state ! -name SHA256SUMS -print0 |
  sort -z | xargs -0 sha256sum > SHA256SUMS)
printf '%s\n' complete > "$bag_output/state"
trap - EXIT
echo "Recording complete: $bag_output"
