#!/usr/bin/env bash
# Stop the UART's existing owner before running this receive-only diagnostic.
set -euo pipefail
serial_repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
serial_device="${WR_SERIAL_DEVICE:-/dev/ttyUSB0}"
serial_device="$(readlink -f -- "$serial_device")"
test -c "$serial_device"
serial_group="$(stat -c %g -- "$serial_device")"
serial_image="${WR_PLATFORM_IMAGE:-drone_autonomy_platform:orin}"
exec docker run --rm --network none --entrypoint /bin/bash \
  --device "$serial_device:/dev/wr-serial" --group-add "$serial_group" \
  --mount "type=bind,src=$serial_repo,dst=$serial_repo" --workdir "$serial_repo" \
  --env PYTHONNOUSERSITE=1 \
  "$serial_image" "$serial_repo/deploy/check-serial.sh" "$@" --device /dev/wr-serial
