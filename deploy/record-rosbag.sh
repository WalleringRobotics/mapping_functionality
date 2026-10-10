#!/usr/bin/env bash
# Prepare a session, run ROS launch in the foreground, then seal the stopped bag.
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
bag_kind=survey
bag_px4_rate=0
bag_survey=""
bag_announce=false
while (($#)); do
  case "$1" in
    --output) bag_output="$2"; shift 2 ;;
    --config) bag_config="$2"; shift 2 ;;
    --duration) bag_duration="$2"; shift 2 ;;
    --warmup) bag_warmup="$2"; shift 2 ;;
    --device-id) bag_device="$2"; shift 2 ;;
    --require-mount) bag_mount="$2"; shift 2 ;;
    --fcu-url) bag_fcu_url="$2"; shift 2 ;;
    --kind) bag_kind="$2"; shift 2 ;;
    --px4-imu-rate) bag_px4_rate="$2"; shift 2 ;;
    --survey-check) bag_survey="$2"; shift 2 ;;
    --announce) bag_announce=true; shift ;;
    --camera-only) bag_camera_only=true; shift ;;
    --start-mavros) bag_start_mavros=true; shift ;;
    --help) echo "Usage: $0 --output NEW_DIRECTORY [--duration SECONDS] [--warmup SECONDS] [--config OAK_YAML] [--device-id ID] [--camera-only | --start-mavros] [--fcu-url URL] [--require-mount MOUNT] [--kind survey|calibration] [--px4-imu-rate 0|100] [--survey-check CHECKED_DIR] [--announce]"; exit 0 ;;
    *) echo "Unknown option: $1" >&2; exit 2 ;;
  esac
done
[[ -n "$bag_output" && "$bag_duration" =~ ^[0-9]+$ && "$bag_warmup" =~ ^[0-9]+$ ]] || {
  echo "A new output directory and integer duration/warmup are required" >&2; exit 2;
}
[[ "$bag_camera_only:$bag_start_mavros" != true:true ]] || exit 2
[[ "$bag_kind" == survey || "$bag_kind" == calibration ]] || exit 2
[[ "$bag_px4_rate" == 0 || "$bag_px4_rate" == 100 ]] || exit 2
# Keep MAVROS alive through cleanup, including launch interruption/failure.
if [[ "$bag_px4_rate" != 0 && ( "$bag_camera_only" == true || "$bag_start_mavros" == true ) ]]; then
  echo "A PX4 rate request requires an existing MAVROS owner; start MAVROS separately" >&2; exit 2
fi
# A checked QGroundControl plan and operator announcements both need PX4 through MAVROS.
bag_survey_sha=null
if [[ -n "$bag_survey" ]]; then
  [[ "$bag_camera_only" == false && "$bag_kind" == survey ]] || {
    echo "A survey plan needs PX4 telemetry and a survey recording" >&2; exit 2; }
  bag_survey="$(realpath "$bag_survey")"
  bag_survey_sha="\"$(python3 "$bag_repo/src/wallering_mapping/recording.py" --verify-survey "$bag_survey" --profile "$bag_config")\"" || {
    echo "Use a plan that passed wr-map survey-check" >&2; exit 2; }
fi
if [[ "$bag_announce" == true && "$bag_camera_only" == true ]]; then
  echo "Announcements go through MAVROS; remove --camera-only" >&2; exit 2
fi
if [[ "$bag_kind" == calibration ]]; then
  [[ "$bag_duration" == 110 ]] || { echo "Calibration guidance requires --duration 110" >&2; exit 2; }
fi
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
cp "$bag_repo/configs/rosbag-qos.yaml" "$bag_output/rosbag-qos.yaml"
cp "$bag_repo/configs/fastdds-profile.xml" "$bag_output/fastdds-profile.xml"
cp "$bag_repo/deploy/record-rosbag.sh" "$bag_output/recorder-script.sh"
cp "$bag_repo/deploy/run-ros.sh" "$bag_output/runtime-wrapper.sh"
cp "$bag_repo/deploy/record.launch.py" "$bag_output/record.launch.py"
cp "$bag_repo/deploy/record-rosbag-checks.sh" "$bag_output/readiness-script.sh"
cp "$bag_repo/src/wallering_mapping/recording.py" "$bag_output/recording.py"
cp "$bag_repo/deploy/record-resources.py" "$bag_output/record-resources.py"
cp "$bag_repo/deploy/seal-rosbag.sh" "$bag_output/seal-script.sh"
if [[ -n "$bag_survey" ]]; then
  cp "$bag_survey/survey-check.json" "$bag_survey/survey.plan" "$bag_output/"
fi
printf '%s\n' starting > "$bag_output/state"
printf '{"schema_version": 1, "kind": "%s", "px4_imu_requested_hz": %s, "survey_plan_sha256": %s, "announce": %s}\n' \
  "$bag_kind" "$bag_px4_rate" "$bag_survey_sha" "$bag_announce" > "$bag_output/session.json"
trap 'printf "%s\n" failed > "$bag_output/state"' EXIT
date -u --iso-8601=ns > "$bag_output/started-utc.txt"
dpkg-query -W -f='${Package} ${Version} ${Architecture}\n' 'ros-humble-depthai*' 'ros-humble-rosbag2*' 'ros-humble-mavros*' > "$bag_output/packages.txt" 2>/dev/null || true
# Containers usually run as a different user than the checkout owner.
git -c safe.directory="$bag_repo" -C "$bag_repo" rev-parse HEAD > "$bag_output/git-commit.txt"
git -c safe.directory="$bag_repo" -C "$bag_repo" diff --stat > "$bag_output/git-diff-stat.txt"
# Identify the container image (deploy/run-ros.sh) and its pinned package set.
[[ -z "${WR_MAPPING_IMAGE_ID:-}" ]] || printf '%s\n' "$WR_MAPPING_IMAGE_ID" > "$bag_output/container-image.txt"
[[ ! -d /opt/wallering/manifest ]] || cp -r /opt/wallering/manifest "$bag_output/image-manifest"
# Refuse duplicate owners. A failed graph query is also a failed preflight.
timeout 15 ros2 node list --no-daemon > "$bag_output/nodes-before.txt"
if grep -qx /oak "$bag_output/nodes-before.txt"; then echo "OAK driver is already running" >&2; exit 2; fi
if [[ "$bag_start_mavros" == true ]]; then
  if grep -q '^/mavros' "$bag_output/nodes-before.txt"; then echo "Reuse the existing MAVROS owner" >&2; exit 2; fi
fi
bag_topics=(/oak/rgb/image_raw /oak/rgb/camera_info /oak/left/image_raw /oak/left/camera_info
  /oak/right/image_raw /oak/right/camera_info /oak/imu/data /tf_static /diagnostics)
if [[ "$bag_camera_only" == false ]]; then
  bag_topics+=(/mavros/state /mavros/imu/data_raw /mavros/imu/data /mavros/local_position/pose
    /mavros/global_position/global /mavros/global_position/raw/fix /mavros/timesync_status
    /mavros/time_reference /mavros/gpsstatus/gps1/raw /mavros/gpsstatus/gps1/rtk /uas1/mavlink_source
    /mavros/mission/reached /mavros/mission/waypoints)
fi
printf '%s\n' "${bag_topics[@]}" > "$bag_output/topics.txt"
# ROS launch handles child signals and escalation. Terminal Ctrl+C, Docker's
# init process and systemd deliver signals to this foreground process group.
# Keep this shell alive long enough to seal files after launch has returned.
trap ':' INT TERM
export ROS_LOG_DIR="$bag_output/ros-log"
# Large shared-memory segments for the driver, recorder and checks (#18).
export FASTRTPS_DEFAULT_PROFILES_FILE="$bag_output/fastdds-profile.xml"
# Launch imports the copied record.launch.py; keep bytecode out of the sealed session.
export PYTHONDONTWRITEBYTECODE=1
bag_result=0
bag_device_args=()
[[ -z "$bag_device" ]] || bag_device_args=("device_id:=$bag_device")
ros2 launch "$bag_output/record.launch.py" \
  "output:=$bag_output" "duration:=$bag_duration" "warmup:=$bag_warmup" \
  "camera_only:=$bag_camera_only" "start_mavros:=$bag_start_mavros" \
  "fcu_url:=$bag_fcu_url" "kind:=$bag_kind" "px4_imu_rate:=$bag_px4_rate" \
  "announce:=$bag_announce" "${bag_device_args[@]}" || bag_result=$?
# The existing MAVROS process outlives launch. Restore defaults even after a
# partial request or launch failure; record a failed cleanup instead of hiding it.
if [[ -f "$bag_output/px4-rate-request.json" ]]; then
  python3 "$bag_output/recording.py" --rate 0 --report "$bag_output/px4-rate-restore.json" || {
    printf '%s\n' 'PX4 rate restoration failed; inspect px4-rate-restore.json' > "$bag_output/failure.txt"
    bag_result=2
  }
fi
trap - EXIT INT TERM
bash "$bag_output/seal-script.sh" "$bag_output" "$bag_result"
