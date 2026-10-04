#!/usr/bin/env bash
set -euo pipefail
record_repo=/opt/wallering/mapping_functionality
record_root=/mnt/nvme/mapping
/usr/bin/mountpoint -q /mnt/nvme
test -d "$record_root"
record_name="$(date -u +%Y%m%dT%H%M%SZ)-$(cat /proc/sys/kernel/random/uuid)"
exec "$record_repo/.venv/bin/wr-map" record \
  --config "$record_repo/configs/oakd-survey.json" \
  --output "$record_root/$record_name"

