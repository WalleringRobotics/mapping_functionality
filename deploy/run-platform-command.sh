#!/usr/bin/env bash
# Run a mapping command with the existing platform's ROS and USB environment.
# This does not open the UART; use the existing MAVROS connector.
set -euo pipefail
platform_repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
platform_image="${WR_PLATFORM_IMAGE:-drone_autonomy_platform:orin}"
exec docker run --rm --network host --ipc host \
  --entrypoint /bin/bash \
  --device-cgroup-rule='c 189:* rmw' \
  --mount type=bind,src=/dev/bus/usb,dst=/dev/bus/usb \
  --mount type=bind,src=/etc/nv_tegra_release,dst=/etc/nv_tegra_release,readonly \
  --mount type=bind,src=/proc/device-tree,dst=/run/wr-mapping/device-tree,readonly \
  --mount "type=bind,src=$platform_repo,dst=$platform_repo" \
  --workdir "$platform_repo" \
  --env "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-1}" \
  --env PYTHONNOUSERSITE=1 \
  "$platform_image" -c '
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    if [[ -f /ros2_ws/install/setup.bash ]]; then source /ros2_ws/install/setup.bash; fi
    exec "$@"
  ' wr-mapping-command "$@"
