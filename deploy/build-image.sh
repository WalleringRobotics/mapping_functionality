#!/usr/bin/env bash
# Build the pinned recording/processing image from this checkout.
# First build on an Orin compiles COLMAP and takes a long time; later builds reuse
# the layer cache. Extra arguments go to `docker buildx build`.
set -euo pipefail
build_repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
build_image="${WR_MAPPING_IMAGE:-wallering-mapping:humble}"
docker buildx version >/dev/null || { echo "Docker buildx is required; run deploy/prepare-orin.sh" >&2; exit 2; }
exec docker buildx build --load -f "$build_repo/docker/Dockerfile" -t "$build_image" \
  --label "org.opencontainers.image.revision=$(git -C "$build_repo" rev-parse HEAD)" \
  "$@" "$build_repo"
