#!/usr/bin/env bash
# One-shot readiness and metadata capture, managed as an action by ROS launch.
set -euo pipefail
bag_output="$1"
bag_camera_only="$2"
bag_px4_rate="${3:-0}"
for topic in /oak/rgb/camera_info /oak/left/camera_info /oak/right/camera_info /oak/imu/data; do
  bag_type=sensor_msgs/msg/CameraInfo
  [[ "$topic" != /oak/imu/data ]] || bag_type=sensor_msgs/msg/Imu
  timeout 30 ros2 topic echo "$topic" "$bag_type" --once > "$bag_output/$(echo "$topic" | tr / _)-first.yaml"
done
# Resolve the current session's nodes directly instead of sharing persistent
# ROS CLI daemon discovery state with other commands in the same DDS domain.
timeout 15 ros2 param dump /oak --no-daemon > "$bag_output/oak-parameters.yaml"
if [[ "$bag_camera_only" == false ]]; then
  timeout 30 ros2 topic echo /mavros/state mavros_msgs/msg/State --once --filter 'm.connected' > "$bag_output/mavros-state.yaml"
  grep -q 'connected: true' "$bag_output/mavros-state.yaml" || { echo "PX4 is not connected" >&2; exit 2; }
  timeout 15 ros2 param dump /mavros/time --no-daemon > "$bag_output/mavros-time.yaml"
  if [[ "$bag_px4_rate" != 0 ]]; then
    python3 "$bag_output/recording.py" --rate "$bag_px4_rate" --report "$bag_output/px4-rate-request.json"
  fi
fi
timeout 15 ros2 topic list -t --no-daemon > "$bag_output/graph.txt"
# Preserve the vendor calibration dump without exposing identity in source files.
for calibration in /tmp/*_calibration.json; do
  if [[ -f "$calibration" && "$calibration" -nt "$bag_output/started-utc.txt" ]]; then
    cp "$calibration" "$bag_output/"
  fi
done
