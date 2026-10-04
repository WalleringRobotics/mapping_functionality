#!/usr/bin/env bash
# Reuse drone_autonomy_platform's already-built Humble environment.
# The existing connector owns UART/MAVLink; this container only subscribes.
set -euo pipefail
platform_repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
exec bash "$platform_repo/deploy/run-platform-command.sh" \
  /bin/bash "$platform_repo/deploy/check-hardware.sh" "$@"
