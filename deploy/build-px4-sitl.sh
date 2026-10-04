#!/usr/bin/env bash
# Build pinned upstream PX4; do not run during camera/storage qualification soaks.
set -euo pipefail
if [[ $# -lt 2 || $# -gt 3 ]]; then
  echo "Usage: $0 PX4_SOURCE PYTHON [JOBS=2]" >&2
  exit 2
fi
sitl_source="$(realpath "$1")"
sitl_python="$(realpath --no-symlinks "$2")"
sitl_jobs="${3:-2}"
if [[ ! "$sitl_jobs" =~ ^[1-4]$ ]]; then
  echo 'JOBS must be between 1 and 4.' >&2
  exit 2
fi
sitl_revision=d6f12ad1c4f70ad3230afd7d86e971421e02fef4
if [[ "$(git -C "$sitl_source" rev-parse HEAD)" != "$sitl_revision" ]]; then
  echo "Expected reviewed PX4 v1.17.0 commit $sitl_revision" >&2
  exit 2
fi
git -C "$sitl_source" diff --quiet
git -C "$sitl_source" diff --cached --quiet
if [[ ! -x "$sitl_python" ]]; then
  echo 'PYTHON must be the executable in the dedicated PX4 build environment.' >&2
  exit 2
fi
export PATH="$(dirname "$sitl_python"):$PATH"
sitl_build="$sitl_source/build/px4_sitl_default"
cmake -S "$sitl_source" -B "$sitl_build" -G Ninja \
  -DCONFIG=px4_sitl_default -DPYTHON_EXECUTABLE="$sitl_python"
cmake --build "$sitl_build" --parallel "$sitl_jobs"
printf 'Built upstream PX4 SIH-capable simulator: %s/bin/px4\n' "$sitl_build"
