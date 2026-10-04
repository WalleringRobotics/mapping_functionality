#!/usr/bin/env bash
set -euo pipefail
serial_repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
serial_python="${WR_MAPPING_PYTHON:-$serial_repo/.venv/bin/python}"
export PYTHONPATH="$serial_repo/src${PYTHONPATH:+:$PYTHONPATH}"
exec "$serial_python" -m wallering_mapping.cli serial-check "$@"
