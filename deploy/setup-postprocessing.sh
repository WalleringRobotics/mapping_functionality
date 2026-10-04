#!/usr/bin/env bash
# Native x86-64 workstation setup. Engines come from the official ODM image.
set -euo pipefail
processing_repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
processing_venv="$processing_repo/.venv-postprocess"
processing_image="opendronemap/odm:3.6.2"
if [[ $# -gt 1 || ( $# -eq 1 && "$1" != --check ) ]]; then
  echo "Usage: bash deploy/setup-postprocessing.sh [--check]" >&2
  exit 2
fi
if [[ "$(uname -s)" != Linux || "$(uname -m)" != x86_64 ]]; then
  echo "Offline target: native x86-64 Linux (including x86-64 WSL2). Run this on the processing workstation." >&2
  exit 2
fi
command -v docker >/dev/null || { echo "Install Docker Engine first." >&2; exit 2; }
processing_arch="$(docker info --format '{{.Architecture}}')"
if [[ "$processing_arch" != x86_64 && "$processing_arch" != amd64 ]]; then
  echo "Docker must use a native x86-64 daemon on this workstation." >&2
  exit 2
fi
if [[ $# -eq 0 ]]; then
  python3 -c 'import sys; assert (3, 10) <= sys.version_info[:2] <= (3, 12), "Use Python 3.10–3.12"'
  python3 -m venv "$processing_venv"
  "$processing_venv/bin/python" -m pip install -e "$processing_repo[bags,terrain]"
  docker pull "$processing_image"
fi
"$processing_venv/bin/python" -c 'import cv2, numpy, pyproj, rosbags; import wallering_mapping.cli'
[[ "$(docker image inspect "$processing_image" --format '{{.Os}}/{{.Architecture}}')" == linux/amd64 ]] || {
  echo "The installed ODM image must be linux/amd64." >&2; exit 2;
}
docker run --rm --network none "$processing_image" --version
docker image inspect "$processing_image" --format 'ODM image: {{.Id}}'
echo "Ready. Activate with: source $processing_venv/bin/activate"
echo "Then use configs/process-opensfm.json or configs/process-terrain.json."
