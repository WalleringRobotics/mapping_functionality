#!/usr/bin/env bash
# Diagnostic only: inspect ownership, then send heartbeats/read requests/TIMESYNC.
set -euo pipefail
serial_repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
serial_path="${WR_SERIAL_DEVICE:?Set WR_SERIAL_DEVICE to the adapter /dev/serial/by-id path}"
case "$serial_path" in
  /dev/serial/by-id/*) ;;
  *) echo 'WR_SERIAL_DEVICE must use /dev/serial/by-id/' >&2; exit 2 ;;
esac
test -L "$serial_path"
test -c "$serial_path"
serial_group="$(stat -Lc %g -- "$serial_path")"
serial_major="$(stat -Lc %t -- "$serial_path")"
serial_minor="$(stat -Lc %T -- "$serial_path")"
serial_image="${WR_PLATFORM_IMAGE:-drone_autonomy_platform:orin}"

# Inspect descriptors by device number so aliases and container paths also match.
# Read/inspection capabilities stay in this container, which has no devices.
docker run --rm --network none --pid host --user 0 --cap-drop ALL \
  --cap-add SYS_PTRACE --cap-add DAC_READ_SEARCH \
  --security-opt no-new-privileges --read-only \
  --entrypoint python3 "$serial_image" -c '
import json, os, stat, sys
device = os.makedev(int(sys.argv[1], 16), int(sys.argv[2], 16))
owners, denied = [], []
for pid in os.listdir("/proc"):
    if not pid.isdigit():
        continue
    directory = "/proc/" + pid + "/fd"
    try:
        descriptors = os.listdir(directory)
    except FileNotFoundError:
        continue
    except PermissionError:
        denied.append(int(pid))
        continue
    for descriptor in descriptors:
        try:
            info = os.stat(directory + "/" + descriptor)
        except FileNotFoundError:
            continue
        except PermissionError:
            denied.append(int(pid))
            break
        if stat.S_ISCHR(info.st_mode) and info.st_rdev == device:
            owners.append({"pid": int(pid), "fd": descriptor})
print(json.dumps({"port_owners": owners, "inspection_denied_pids": denied}))
sys.exit(2 if owners or denied else 0)
' "$serial_major" "$serial_minor"

# Keep the inspection capability out of the process that opens the device.
exec docker run --rm --network none --user "$(id -u):$(id -g)" \
  --cap-drop ALL --security-opt no-new-privileges --read-only \
  --device "$serial_path:$serial_path" --group-add "$serial_group" \
  --mount "type=bind,src=$serial_repo,dst=$serial_repo" --workdir "$serial_repo" \
  --env PYTHONDONTWRITEBYTECODE=1 --entrypoint "$serial_repo/.venv/bin/python" \
  "$serial_image" -m wallering_mapping.px4_link_diagnostics "$@" --device "$serial_path"
