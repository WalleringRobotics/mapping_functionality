#!/usr/bin/env bash
# Run only after ROS launch and rosbag2 have exited. Never stop processes here.
set -euo pipefail
bag_output="$1"
bag_result="${2:-0}"
[[ ! -e "$bag_output/SHA256SUMS" && ! -L "$bag_output/SHA256SUMS" ]] || {
  echo "Recording seal already exists; refusing to change $bag_output" >&2; exit 2;
}
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
# Reuse the auditor's OpenSSL-backed implementation. The distribution's GNU
# sha256sum is much slower on the Orin for field-duration, raw-image bags.
# Keep the standard two-space SHA256SUMS format and refuse to replace a seal.
python3 - "$bag_output" <<'PY'
from pathlib import Path
import sys

from wallering_mapping.dataset import sha256_file

root = Path(sys.argv[1])
with (root / "SHA256SUMS").open("x") as output:
    for path in sorted(root.rglob("*")):
        if path.name in {"state", "SHA256SUMS"} or path.is_symlink() or not path.is_file():
            continue
        relative = path.relative_to(root).as_posix()
        # The audit's line-based seal format cannot represent these names.
        if any(character in relative for character in "\\\r\n"):
            raise ValueError(f"Filename cannot be represented in SHA256SUMS: {relative!r}")
        output.write(f"{sha256_file(path)}  ./{relative}\n")
PY
printf '%s\n' complete > "$bag_output/state"
trap - EXIT
echo "Recording complete: $bag_output"
