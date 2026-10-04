#!/usr/bin/env bash
# Run with the same interpreter, account and ROS environment as the recorder.
set -euo pipefail
check_repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
check_python="${WR_MAPPING_PYTHON:-$check_repo/.venv/bin/python}"
export PYTHONPATH="$check_repo/src${PYTHONPATH:+:$PYTHONPATH}"
exec "$check_python" -m wallering_mapping.cli hardware-check "$@"
