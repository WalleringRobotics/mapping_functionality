import copy
import json
from pathlib import Path

import pytest

from wallering_mapping.capabilities import camera_definition, check_handoff, derive_handoff, load_handoff
from wallering_mapping.cli import main

FIXTURE = Path(__file__).parent / "fixtures/survey-wing/mapping_handoff.json"


def handoff():
    return json.loads(FIXTURE.read_text())


def test_actual_design_export_is_consumed_without_treating_design_target_as_endurance(tmp_path):
    data, derived = load_handoff(FIXTURE)
    assert data["mission"]["survey_duration_s"] == 3000
    assert derived["max_camera_groundspeed_m_s"] == pytest.approx(16.44)
    assert derived["resources"]["raw_bytes_per_s"] == 64_520_960
    report = check_handoff(FIXTURE, tmp_path / "checked")
    assert report["inputs_valid"] and not report["passed"]
    assert report["blocking_reasons"] == ["storage_capacity_pass: unmeasured",
                                            "storage_throughput_pass: unmeasured"]
    assert not report["flight_ready"] and not report["capture_ready"] and not report["survey_ready"]
    assert (tmp_path / "checked/mapping_handoff.json").read_bytes() == FIXTURE.read_bytes()
    with pytest.raises(FileExistsError):
        check_handoff(FIXTURE, tmp_path / "checked")


def test_camera_definition_uses_selected_sku_dimensions_and_requested_rate():
    data = handoff()
    camera = camera_definition(data)
    spec = camera["qgc_custom_camera"]
    assert spec["ImageWidth"] == 5320 and spec["ImageHeight"] == 3032
    assert spec["SensorWidth"] == pytest.approx(14.5768)
    assert spec["MinTriggerInterval"] == .25  # requested 4 Hz, not maximum 7 Hz
    assert camera["name"] == data["hardware"]["camera"]["part"]
    assert camera["calibration_state"] == "nominal_design_only"


def test_unpacked_16_bit_at_four_hz_fails_one_gigabit_link():
    data = handoff()
    data["capture"].update(pixel_format="Bayer16_candidate", bytes_per_pixel=2)
    resources = data["resources"]
    for key in ("raw_bytes_per_frame", "raw_bytes_per_s", "minimum_sustained_write_bytes_per_s"):
        resources[key] *= 2
    reserve = data["capture"]["free_space_reserve_bytes"]
    resources["minimum_free_storage_bytes"] = 2 * (resources["minimum_free_storage_bytes"] - reserve) + reserve
    resources["link_budget_pass"] = False
    derived = derive_handoff(data)
    assert derived["resources"]["raw_bytes_per_s"] * 8 == 1_032_335_360
    assert derived["resources"]["link_budget_pass"] is False


@pytest.mark.parametrize("change", [
    lambda d: d["geometry"].update(blur_pass=False),
    lambda d: d["capture"].update(requested_fps=5),
    lambda d: d["capture"].update(requested_fps=8),
    lambda d: d["capture"].update(bytes_per_pixel=2),
    lambda d: d["capture"].update(requested_fps=float("nan")),
    lambda d: d["hardware"]["camera"]["geometry"].update(pixels_across=True),
    lambda d: d.update(survey_ready=True),
    lambda d: d["navigation"].update(flight_gnss_independent=False),
    lambda d: d.update(source_files={"../source": "a" * 64}),
])
def test_stale_invalid_or_unqualified_inputs_cannot_pass(change):
    data = copy.deepcopy(handoff())
    change(data)
    with pytest.raises(ValueError):
        derive_handoff(data)


def test_handoff_cli_reports_unknown_storage_with_failing_exit_code(tmp_path, capsys):
    assert main(["handoff-check", str(FIXTURE), "--output", str(tmp_path / "check")]) == 2
    assert json.loads(capsys.readouterr().out)["inputs_valid"]
