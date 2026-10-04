#!/usr/bin/env bash
# Supervises vendor nodes and rosbag2. ROS messages are written only by rosbag2.
set -euo pipefail
bag_repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
bag_output=""
bag_config="$bag_repo/configs/oakd-ros.yaml"
bag_duration=60
bag_warmup=5
bag_camera_only=false
bag_start_mavros=false
bag_device=""
bag_fcu_url=/dev/ttyUSB0:921600
bag_mount=""
bag_min_free=5368709120
while (($#)); do
  case "$1" in
    --output) bag_output="$2"; shift 2 ;;
    --config) bag_config="$2"; shift 2 ;;
    --duration) bag_duration="$2"; shift 2 ;;
    --warmup) bag_warmup="$2"; shift 2 ;;
    --device-id) bag_device="$2"; shift 2 ;;
    --require-mount) bag_mount="$2"; shift 2 ;;
    --fcu-url) bag_fcu_url="$2"; shift 2 ;;
    --camera-only) bag_camera_only=true; shift ;;
    --start-mavros) bag_start_mavros=true; shift ;;
    --help) echo "Usage: $0 --output NEW_DIRECTORY [--duration SECONDS] [--warmup SECONDS] [--config OAK_YAML] [--device-id ID] [--camera-only | --start-mavros] [--fcu-url URL] [--require-mount MOUNT]"; exit 0 ;;
    *) echo "Unknown option: $1" >&2; exit 2 ;;
  esac
done
[[ -n "$bag_output" && "$bag_duration" =~ ^[0-9]+$ && "$bag_warmup" =~ ^[0-9]+$ ]] || {
  echo "A new output directory and integer duration/warmup are required" >&2; exit 2;
}
[[ "$bag_camera_only:$bag_start_mavros" != true:true ]] || exit 2
[[ -f "$bag_config" && "$bag_config" == *.yaml ]] || { echo "Use an official-driver YAML profile" >&2; exit 2; }
command -v ros2 >/dev/null || { echo "Source ROS or use deploy/run-ros.sh" >&2; exit 2; }
bag_output="$(realpath -m "$bag_output")"
bag_parent="$(dirname "$bag_output")"
[[ -d "$bag_parent" && ! -e "$bag_output" ]] || { echo "Output must be new, with an existing parent" >&2; exit 2; }
if [[ -n "$bag_mount" ]]; then
  mountpoint -q "$bag_mount" || { echo "Required mount is absent" >&2; exit 2; }
  [[ "$bag_output/" == "$(realpath "$bag_mount")/"* && "$(stat -c %d "$bag_parent")" == "$(stat -c %d "$bag_mount")" ]] || exit 2
fi
bag_free() { df -B1 --output=avail "$bag_parent" | tail -1 | tr -d ' '; }
(( $(bag_free) > bag_min_free )) || { echo "Insufficient free storage" >&2; exit 2; }
mkdir "$bag_output"
cp "$bag_config" "$bag_output/oak-requested.yaml"
cp "$bag_repo/configs/rosbag-mcap.yaml" "$bag_output/mcap.yaml"
cp "$bag_repo/deploy/record-rosbag.sh" "$bag_output/recorder-script.sh"
cp "$bag_repo/deploy/run-ros.sh" "$bag_output/runtime-wrapper.sh"
printf '%s\n' starting > "$bag_output/state"
date -u --iso-8601=ns > "$bag_output/started-utc.txt"
dpkg-query -W -f='${Package} ${Version} ${Architecture}\n' 'ros-humble-depthai*' 'ros-humble-rosbag2*' 'ros-humble-mavros*' > "$bag_output/packages.txt" 2>/dev/null || true
git -C "$bag_repo" rev-parse HEAD > "$bag_output/git-commit.txt"
git -C "$bag_repo" diff --stat > "$bag_output/git-diff-stat.txt"
bag_pids=()
bag_recorder=""
bag_stopped=false
bag_failed=false
bag_cleanup() {
  local original=$?
  trap - EXIT INT TERM
  if [[ -n "$bag_recorder" ]] && kill -0 "$bag_recorder" 2>/dev/null; then
    kill -INT -- "-$bag_recorder" 2>/dev/null || true
    # Finalize chunks/indexes before stopping publishers.
    for ((i=0; i<150; i++)); do
      kill -0 "$bag_recorder" 2>/dev/null || break
      sleep .2
    done
    if kill -0 "$bag_recorder" 2>/dev/null; then
      kill -TERM -- "-$bag_recorder" 2>/dev/null || true
      bag_failed=true
    fi
    for ((i=0; i<25; i++)); do
      kill -0 "$bag_recorder" 2>/dev/null || break
      sleep .2
    done
    kill -KILL -- "-$bag_recorder" 2>/dev/null || true
  fi
  if [[ -n "$bag_recorder" ]]; then wait "$bag_recorder" || bag_failed=true; fi
  for pid in "${bag_pids[@]}"; do kill -INT -- "-$pid" 2>/dev/null || true; done
  for ((i=0; i<50; i++)); do
    local alive=false
    for pid in "${bag_pids[@]}"; do kill -0 "$pid" 2>/dev/null && alive=true; done
    [[ "$alive" == false ]] && break
    sleep .2
  done
  for pid in "${bag_pids[@]}"; do
    if kill -0 "$pid" 2>/dev/null; then kill -TERM -- "-$pid" 2>/dev/null || true; fi
  done
  sleep 1
  for pid in "${bag_pids[@]}"; do
    kill -KILL -- "-$pid" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
  done
  date -u --iso-8601=ns > "$bag_output/finished-utc.txt"
  printf '%s\n' finalizing > "$bag_output/state"
  if (( original == 0 )) && [[ "$bag_failed" == false && -f "$bag_output/bag/metadata.yaml" ]]; then
    ros2 bag info "$bag_output/bag" > "$bag_output/bag-info.txt" || bag_failed=true
    if [[ "$bag_failed" == false ]]; then
      if (cd "$bag_output" && find . -type f ! -name state ! -name SHA256SUMS -print0 | sort -z | xargs -0 sha256sum > SHA256SUMS); then
        printf '%s\n' complete > "$bag_output/state"
        echo "Recording complete: $bag_output"
      else
        bag_failed=true
      fi
    fi
  else
    bag_failed=true
  fi
  if [[ "$bag_failed" == true ]]; then
    printf '%s\n' failed > "$bag_output/state"
    echo "Recording failed; preserved at $bag_output" >&2
    original=2
  fi
  exit "$original"
}
trap bag_cleanup EXIT
trap 'bag_stopped=true' INT TERM
# Refuse duplicate owners. A failed graph query is also a failed preflight.
timeout 15 ros2 node list --no-daemon > "$bag_output/nodes-before.txt"
if grep -qx /oak "$bag_output/nodes-before.txt"; then echo "OAK driver is already running" >&2; exit 2; fi
if [[ "$bag_start_mavros" == true ]]; then
  if grep -q '^/mavros' "$bag_output/nodes-before.txt"; then echo "Reuse the existing MAVROS owner" >&2; exit 2; fi
  setsid ros2 launch mavros px4.launch "fcu_url:=$bag_fcu_url" > "$bag_output/mavros.log" 2>&1 &
  bag_pids+=("$!")
fi
bag_device_args=()
[[ -z "$bag_device" ]] || bag_device_args=(-p "camera.i_mx_id:=$bag_device")
setsid ros2 run depthai_ros_driver camera_node --ros-args -r __node:=oak \
  --params-file "$bag_config" "${bag_device_args[@]}" > "$bag_output/oak.log" 2>&1 &
bag_pids+=("$!")
bag_topics=(/oak/rgb/image_raw /oak/rgb/camera_info /oak/left/image_raw /oak/left/camera_info
  /oak/right/image_raw /oak/right/camera_info /oak/imu/data /tf_static /diagnostics)
for topic in /oak/rgb/camera_info /oak/left/camera_info /oak/right/camera_info /oak/imu/data; do
  bag_type=sensor_msgs/msg/CameraInfo
  [[ "$topic" != /oak/imu/data ]] || bag_type=sensor_msgs/msg/Imu
  timeout 30 ros2 topic echo "$topic" "$bag_type" --once > "$bag_output/$(echo "$topic" | tr / _)-first.yaml"
done
timeout 15 ros2 param dump /oak > "$bag_output/oak-parameters.yaml"
if [[ "$bag_camera_only" == false ]]; then
  timeout 30 ros2 topic echo /mavros/state mavros_msgs/msg/State --once --filter 'm.connected' > "$bag_output/mavros-state.yaml"
  grep -q 'connected: true' "$bag_output/mavros-state.yaml" || { echo "PX4 is not connected" >&2; exit 2; }
  timeout 15 ros2 param dump /mavros/time > "$bag_output/mavros-time.yaml"
  bag_topics+=(/mavros/state /mavros/imu/data_raw /mavros/imu/data /mavros/local_position/pose
    /mavros/global_position/global /mavros/global_position/raw/fix /mavros/timesync_status
    /mavros/time_reference /mavros/gpsstatus/gps1/raw /mavros/gpsstatus/gps1/rtk /uas1/mavlink_source)
fi
printf '%s\n' "${bag_topics[@]}" > "$bag_output/topics.txt"
timeout 15 ros2 topic list -t --no-daemon > "$bag_output/graph.txt"
# Preserve the vendor calibration dump without exposing identity in source files.
for calibration in /tmp/*_calibration.json; do
  if [[ -f "$calibration" && "$calibration" -nt "$bag_output/started-utc.txt" ]]; then
    cp "$calibration" "$bag_output/"
  fi
done
echo "Warming up for $bag_warmup seconds; recording $bag_duration seconds to $bag_output"
for ((i=0; i<bag_warmup; i++)); do
  [[ "$bag_stopped" == false ]] || exit 2
  for pid in "${bag_pids[@]}"; do kill -0 "$pid" || exit 2; done
  sleep 1 || [[ "$bag_stopped" == true ]]
done
[[ "$bag_stopped" == false ]] || exit 2
setsid ros2 bag record -s mcap --max-cache-size 104857600 --max-bag-size 1073741824 \
  --storage-config-file "$bag_output/mcap.yaml" -o "$bag_output/bag" \
  "${bag_topics[@]}" > "$bag_output/recorder.log" 2>&1 &
bag_recorder=$!
printf '%s\n' recording > "$bag_output/state"
for ((i=0; bag_duration == 0 || i<bag_duration; i++)); do
  [[ "$bag_stopped" == false ]] || break
  kill -0 "$bag_recorder" || exit 2
  for pid in "${bag_pids[@]}"; do kill -0 "$pid" || exit 2; done
  if (( i % 5 == 0 )); then
    (( $(bag_free) > bag_min_free )) || { echo "Disk reserve reached" >&2; exit 2; }
    echo "Recording $i/$bag_duration seconds"
  fi
  sleep 1 || [[ "$bag_stopped" == true ]]
done
