"""Survey design rules: optics, aircraft and target to height, spacing, speed and margins.

No vehicle IO and no geometry. Formulas follow ``survey_plan.check_survey`` so a
design that satisfies these rules also satisfies the checker.
"""

import json
import math
from pathlib import Path

from .survey_plan import DEFAULT_LIMITS, MULTIROTOR_TYPES, finite

AIRCRAFT_SCHEMA = "wallering.px4-multirotor/v1"
# Field: (PX4 parameter it mirrors, lowest accepted, highest accepted).
AIRCRAFT_FIELDS = {"max_survey_speed_m_s": ("MPC_XY_CRUISE", 0.5, 20.0),
                   "accel_m_s2": ("MPC_ACC_HOR", 0.5, 15.0),
                   "acceptance_radius_m": ("NAV_ACC_RAD", 0.1, 20.0),
                   "climb_m_s": ("MPC_Z_V_AUTO_UP", 0.2, 10.0),
                   "descent_m_s": ("MPC_Z_V_AUTO_DN", 0.2, 10.0),
                   "return_height_m": ("RTL_RETURN_ALT", 0.0, 500.0)}


def read_aircraft(path):
    """A reviewed PX4 multirotor definition; template nulls and out-of-range values fail."""
    data = json.loads(Path(path).read_text())
    if data.get("schema") != AIRCRAFT_SCHEMA or data.get("schema_version") != 1:
        raise ValueError(f"Aircraft definition must be {AIRCRAFT_SCHEMA}, schema_version 1")
    for key in ("aircraft_id", "source"):
        if not isinstance(data.get(key), str) or not data[key].strip():
            raise ValueError(f"Aircraft definition needs {key}: name the vehicle and where its values came from")
    if data.get("vehicle_type") not in MULTIROTOR_TYPES:
        raise ValueError("Aircraft vehicle_type must be a multirotor MAV_TYPE: "
                         + ", ".join(f"{k} ({v})" for k, v in sorted(MULTIROTOR_TYPES.items())))
    for key, (parameter, lower, upper) in AIRCRAFT_FIELDS.items():
        if data.get(key) is None:
            raise ValueError(f"Aircraft definition needs {key} (PX4 {parameter})")
        if not lower <= finite(data[key], key) <= upper:
            raise ValueError(f"Aircraft {key} must be in [{lower}, {upper}]")
    return data
