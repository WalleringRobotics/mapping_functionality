#!/usr/bin/env bash
# Run official ROS tools natively when sourced, otherwise in this repository's
# image (deploy/build-image.sh). No custom acquisition process.
set -euo pipefail
ros_repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if command -v ros2 >/dev/null && [[ "${WR_MAPPING_CONTAINER:-auto}" != always ]]; then
  exec "$@"
fi
ros_image="${WR_MAPPING_IMAGE:-${WR_PLATFORM_IMAGE:-wallering-mapping:humble}}"
ros_image_id="$(docker image inspect -f '{{.Id}}' "$ros_image" 2>/dev/null)" || {
  echo "Image $ros_image is absent; build it with deploy/build-image.sh" >&2; exit 2;
}
ros_devices=()
ros_mounts=()
if [[ -n "${WR_MAPPING_ROOT:-}" ]]; then
  [[ -d "$WR_MAPPING_ROOT" ]] || { echo "Recording root is absent" >&2; exit 2; }
  ros_mounts+=(--mount "type=bind,src=$WR_MAPPING_ROOT,dst=$WR_MAPPING_ROOT")
fi
if [[ -n "${WR_MAPPING_MOUNT:-}" && "${WR_MAPPING_MOUNT:-}" != "${WR_MAPPING_ROOT:-}" ]]; then
  mountpoint -q "$WR_MAPPING_MOUNT" || { echo "Required storage mount is absent" >&2; exit 2; }
  ros_mounts+=(--mount "type=bind,src=$WR_MAPPING_MOUNT,dst=$WR_MAPPING_MOUNT")
fi
if [[ -n "${WR_MAPPING_SERIAL_DEVICE:-}" ]]; then
  [[ -c "$WR_MAPPING_SERIAL_DEVICE" ]] || { echo "Serial device is absent" >&2; exit 2; }
  ros_devices+=(--device "$WR_MAPPING_SERIAL_DEVICE:/dev/ttyUSB0"
    --group-add "$(stat -Lc %g "$WR_MAPPING_SERIAL_DEVICE")")
fi
# Run as the caller so sessions stay theirs; plugdev grants OAK USB access.
ros_plugdev="$(getent group plugdev | cut -d: -f3)" || { echo "Host plugdev group is absent" >&2; exit 2; }
exec docker run --rm --init --network host --ipc host --stop-signal SIGINT --stop-timeout -1 \
  --user "$(id -u):$(id -g)" --group-add "$ros_plugdev" \
  --entrypoint /bin/bash --device-cgroup-rule='c 189:* rmw' \
  --mount type=bind,src=/dev/bus/usb,dst=/dev/bus/usb \
  --mount "type=bind,src=$ros_repo,dst=$ros_repo" --workdir "$ros_repo" \
  --env "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-1}" --env PYTHONNOUSERSITE=1 \
  --env "PYTHONPATH=$ros_repo/src" --env "WR_MAPPING_IMAGE_ID=$ros_image_id" \
  --env TINI_KILL_PROCESS_GROUP=1 \
  "${ros_devices[@]}" "${ros_mounts[@]}" "$ros_image" -c '
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    exec "$@"
  ' wr-ros "$@"
