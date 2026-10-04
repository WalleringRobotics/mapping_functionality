# Working in this repository

For Jetson/OAK/PX4 hardware verification, startup checks, or serial diagnostics,
read [the repository hardware skill](skills/jetson-mapping-checks/SKILL.md).
Its commands and supporting references are kept in this checkout.

The application lives in `src/wallering_mapping`; entry points are in `cli.py`.
Use `.venv/bin/python` for local checks. ROS may be available only in the sibling
`drone_autonomy_platform` repository's existing container image; the skill explains
how to reuse it. Keep bench datasets and device-specific reports under ignored
`runs/`, and put reproducible procedures in `docs/` and `deploy/`.
