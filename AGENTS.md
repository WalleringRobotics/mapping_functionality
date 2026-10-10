# Working in this repository

## Delivery workflow

Use a feature branch for repository changes. Local changes alone are not a
completed handoff: at minimum, commit the work and push the branch so it is
remotely available. Open or update a pull request for review and include the
validation results and any remaining acceptance work. Follow the PR review and
required-check process before merging. Do not stop at a local-only result unless
the user explicitly requests it; if publishing is blocked, report the blocker
and the work that remains unpublished.

## Runtime and hardware

For Jetson/OAK/PX4 hardware verification, startup checks, or serial diagnostics,
read [the repository hardware skill](skills/jetson-mapping-checks/SKILL.md).
Its commands and supporting references are kept in this checkout.

The application lives in `src/wallering_mapping`; entry points are in `cli.py`.
The reproducible runtime (ROS Humble, drivers, Python dependencies, PyCOLMAP) is
the image in `docker/`, built by `deploy/build-image.sh` on a host prepared by
`deploy/prepare-orin.sh`; `deploy/run-ros.sh` and `deploy/run-platform-command.sh`
run checkout code inside it. `.venv/bin/python` suits quick portable checks only. Keep bench datasets and device-specific reports under ignored
`runs/`, and put reproducible procedures in `docs/` and `deploy/`.
