"""Offline transport/safety checks. These are not PX4 SITL acceptance evidence."""

import copy
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from wallering_mapping.calibration_mission import generate_mission

spec = importlib.util.spec_from_file_location(
    "calibration_sitl", Path(__file__).parents[1] / "deploy/verify-calibration-sitl.py")
sitl = importlib.util.module_from_spec(spec)
spec.loader.exec_module(sitl)


@pytest.fixture
def plan(tmp_path):
    path = tmp_path / "calibration.plan"
    generate_mission((47.397742, 8.545594), 489.4, 8, 2, 5, 15, 8, 15,
                     path, altitude_step=1, yaw_hold=2)
    return json.loads(path.read_text())


def test_transport_preserves_unspecified_yaw_and_actual_coordinates(plan):
    item = sitl.mission_items(plan)[0]
    wire = SimpleNamespace(seq=0, frame=6, command=22, autocontinue=1,
        param1=0., param2=0., param3=0., param4=float("nan"),
        x=473977420, y=85455940, z=8., get_type=lambda: "MISSION_ITEM_INT")
    decoded = sitl.comparable_download(wire)
    assert decoded["params"] == [0., 0., 0., None, 47.397742, 8.545594, 8.]
    sitl.check_download([item], [decoded])


@pytest.mark.parametrize("field,value", [("frame", 0), ("command", 21), ("seq", 4)])
def test_download_refuses_changed_identity(plan, field, value):
    items = sitl.mission_items(plan)
    changed = copy.deepcopy(items)
    changed[0][field] = value
    with pytest.raises(ValueError, match="identity"):
        sitl.check_download(items, changed)


@pytest.mark.parametrize("value", [None, 0., 1.])
def test_download_refuses_coordinate_or_unspecified_yaw_changes(plan, value):
    items = sitl.mission_items(plan)
    changed = copy.deepcopy(items)
    changed[0]["params"][4 if value is None else 3] = value
    with pytest.raises(ValueError, match="parameter"):
        sitl.check_download(items, changed)


def test_download_coordinates_require_integer_mavlink_precision(plan):
    items = sitl.mission_items(plan)
    changed = copy.deepcopy(items)
    changed[0]["params"][4] += 1e-6  # about11cm: cannot use a generic float tolerance
    with pytest.raises(ValueError, match="parameter"):
        sitl.check_download(items, changed)


@pytest.mark.parametrize("mutation", [
    lambda p: p["mission"]["items"][0].update(command=400),  # arbitrary arm command
    lambda p: p["mission"]["items"][0].update(frame=0),  # unexpected altitude reference
    lambda p: p["mission"]["items"][0]["params"].__setitem__(4, None),
    lambda p: p["mission"]["items"][0]["params"].__setitem__(6, 1000),
    lambda p: p["mission"]["items"][0]["params"].__setitem__(4, float("nan")),
])
def test_non_generator_mission_cannot_reach_simulator_transport(plan, mutation):
    mutation(plan)
    with pytest.raises(ValueError):
        sitl.mission_items(plan)


@pytest.mark.parametrize("container,interfaces,devices", [
    (False, {"lo"}, []), (True, {"lo", "eth0"}, []), (True, {"lo"}, ["ttyACM0"]),
])
def test_physical_or_networked_environment_rejected_before_connection(
        monkeypatch, container, interfaces, devices):
    class FakePath:
        def __init__(self, value):
            self.value = value

        def exists(self):
            return container

        def iterdir(self):
            return [SimpleNamespace(name=name) for name in interfaces]

        def glob(self, pattern):
            return devices

    monkeypatch.setattr(sitl, "Path", FakePath)
    with pytest.raises(ValueError):
        sitl.require_isolation()
