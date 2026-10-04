import copy
import json

import numpy as np
import pytest
from pyproj import Transformer

from test_mavros import PARAMETERS, clock, header, telemetry_fixture
from test_rtcm import base_frame, frame
from wallering_mapping.association import associate
from wallering_mapping.dataset import jsonl, sha256_file, write_json
from wallering_mapping.gnss_accuracy import (
    PROFILE,
    ecef_and_enu,
    gps_source_ros_ns,
    gps_bracket,
    image_accuracy,
    position_budget,
    read_profile,
)
from wallering_mapping.mavros import TelemetryBuffer
from wallering_mapping.telemetry_config import TelemetryConfig, rtk_topics
from wallering_mapping.validate import validate
from wallering_mapping.cli import main
from wallering_mapping.export import export


def rtk_fixture(tmp_path, age=20, mode="px4_boot_via_mavros"):
    root, _ = telemetry_fixture(tmp_path / "capture")
    profile = copy.deepcopy(PROFILE)
    base = profile["base"]
    base.update(
        station_id=42,
        wgs84_lat_lon_h=[52, 13, 100],
        covariance_enu_m2=np.diag([1e-6, 1e-6, 4e-6]).tolist(),
        survey_evidence="synthetic independent survey",
        datum_epoch_evidence="synthetic WGS84 epoch alignment",
        rover_uncertainty_includes_base=False,
    )
    profile["receiver"].update(
        model_firmware_evidence="synthetic receiver",
        position_reference="ARP",
        gpsraw_time_mode=mode,
        timestamp_validation_evidence="synthetic clock truth",
        horizontal_accuracy_model="axis_1sigma",
        vertical_accuracy_model="axis_1sigma",
        accuracy_validation_evidence="synthetic covariance truth",
        ellipsoidal_height_verified=True,
        correction_age_field_verified=True,
    )
    profile["rig"].update(
        antenna_to_camera_flu_m=[1, 0, 0],
        lever_covariance_flu_m2=(np.eye(3) * 1e-6).tolist(),
        attitude_sigma_rad=0.001,
        calibration_evidence="synthetic known rig",
        orientation_in_base_tangent_enu_verified=True,
        lever_from_receiver_reference_verified=True,
    )
    profile["motion"].update(
        speed_bound_m_s=1,
        acceleration_bound_m_s2=2,
        angular_rate_bound_rad_s=0.1,
        receiver_latency_bound_ms=0,
        camera_latency_bound_ms=0,
        bound_validation_evidence="synthetic exact measurements",
    )
    profile["policy"]["output_crs"] = "EPSG:32633"
    config = TelemetryConfig(
        topics={**TelemetryConfig().topics, **rtk_topics()}, min_sync_samples=2
    )
    buffer = TelemetryBuffer(config, PARAMETERS)
    xyz = ecef_and_enu(base["wgs84_lat_lon_h"])[0]
    offset = 1700000000000000000
    for index in range(20):
        mono, ros = 10 * 10**9 + index * 10**8, offset + 10**9 + index * 10**8
        encoded = ros
        if mode == "unix_via_mavros":
            encoded = ros + offset
            if encoded >= 2**31 * 10**9:
                encoded -= 2**32 * 10**9
        fields = {
            **header(encoded),
            "fix_type": 6,
            "lat": 520000000 + (index % 2) * 100,
            "lon": 130000000 + (index // 2) * 100,
            "alt_ellipsoid": 100000,
            "alt": 55000,
            "h_acc": 10,
            "v_acc": 20,
            "eph": 500,
            "epv": 800,
            "satellites_visible": 20,
            "dgps_age": age,
        }
        buffer.ingest(
            "gps_raw", fields, "mavros_msgs/msg/GPSRAW", b"cdr", clock(mono + 5000, ros + 5000), ros
        )
        corrections = base_frame(xyz=xyz) + frame(bytes([0x43, 0x20, 42]) + bytes(10))
        buffer.ingest(
            "rtcm",
            {**header(ros), "data": list(corrections)},
            "mavros_msgs/msg/RTCM",
            b"cdr",
            clock(mono + 6000, ros + 6000),
            ros,
        )
    rows = buffer.drain(check=False)
    with (root / "telemetry.jsonl").open("a") as file:
        for row in rows:
            file.write(json.dumps(row) + "\n")
    manifest = json.loads((root / "manifest.json").read_text())
    manifest["telemetry"]["config"] = config.to_dict()
    manifest["counts"].update(telemetry_gps_raw=20, telemetry_rtcm=20)
    manifest["journals_sha256"]["telemetry"] = sha256_file(root / "telemetry.jsonl")
    write_json(root / "manifest.json", manifest)
    assert validate(root)["valid"]
    alignment = tmp_path / "alignment"
    assert associate(root, alignment)["passed"]
    path = tmp_path / "profile.json"
    write_json(path, profile)
    return root, alignment, path


def test_covariance_propagation_and_shared_base_not_averaged():
    budget = position_budget(
        np.eye(3) * 0.0001,
        np.eye(3) * 0.0004,
        np.eye(3) * 0.000001,
        np.eye(3),
        [1, 0, 0],
        0.01,
        0.003,
    )
    c = np.array(budget["covariance_enu_m2"])
    np.testing.assert_allclose(np.diag(c), [0.000501, 0.000601, 0.000601])
    assert budget["conditional_gaussian_horizontal_95_m"] == pytest.approx(
        np.sqrt(5.991464547107979 * 0.000601)
    )
    assert budget["deterministic_motion_allowance_m"] == 0.003
    np.testing.assert_allclose(
        budget["components_covariance_enu_m2"]["shared_base"], np.eye(3) * 0.0004
    )


def test_moving_receiver_interpolates_ecef_without_halving_correlated_error(tmp_path):
    root, alignment, path = rtk_fixture(tmp_path)
    rows = list(jsonl(alignment / "associations.jsonl"))
    rows[0]["exposure_monotonic_ns"] += 50_000_000
    with (alignment / "associations.jsonl").open("w") as file:
        for row in rows:
            file.write(json.dumps(row) + "\n")
    report = json.loads((alignment / "report.json").read_text())
    report["output_hashes"]["associations.jsonl"] = sha256_file(alignment / "associations.jsonl")
    write_json(alignment / "report.json", report)
    image_accuracy(root, alignment, path, tmp_path / "accuracy")
    result = list(jsonl(tmp_path / "accuracy/images.jsonl"))[0]
    interpolation = result["receiver_interpolation"]
    assert interpolation["after_weight"] == pytest.approx(0.5)
    assert len(interpolation["samples"]) == 2
    endpoints = [
        ecef_and_enu([s["fields"]["lat"] / 1e7, s["fields"]["lon"] / 1e7, 100])[0]
        for s in interpolation["samples"]
    ]
    _, base_axes = ecef_and_enu([52, 13, 100])
    expected = (endpoints[0] + endpoints[1]) / 2 + base_axes.T @ np.array([1, 0, 0])
    np.testing.assert_allclose(result["camera_ecef_m"], expected, atol=1e-8, rtol=0)
    rover_cov = result["budget"]["components_covariance_enu_m2"]["rover"]
    assert np.trace(rover_cov) == pytest.approx(2 * 0.01**2 + 0.02**2)
    assert result["interpolation_curvature_allowance_m"] == pytest.approx(0.0025)


def test_gps_interpolation_rejects_extrapolation_and_distant_endpoints():
    rows = [{"mapped_monotonic_ns": t, "clock_budget_ns": 1} for t in [100, 200]]
    assert gps_bracket(rows, [100, 200], 99, 1000) is None
    assert gps_bracket(rows, [100, 200], 201, 1000) is None
    assert gps_bracket(rows, [100, 200], 150, 49) is None
    assert len(gps_bracket(rows, [100, 200], 100, 1)["samples"]) == 1


def test_geodesy_matches_independent_pyproj_and_wrapped_unix_clock_conversion():
    xyz, rotation = ecef_and_enu([52, 13, 100])
    expected = Transformer.from_crs(4979, 4978, always_xy=True).transform(13, 52, 100)
    np.testing.assert_allclose(xyz, expected, atol=1e-7)
    np.testing.assert_allclose(rotation @ rotation.T, np.eye(3), atol=1e-12)
    epoch, offset = 1700000001500000000, 1700000000000000000
    row = {"source_stamp_ros_ns": epoch + offset - 2**32 * 10**9}
    assert gps_source_ros_ns(row, "unix_via_mavros", offset) == epoch
    with pytest.raises(ValueError, match="unverified"):
        gps_source_ros_ns(row, "unknown", offset)


@pytest.mark.parametrize("mode", ["px4_boot_via_mavros", "unix_via_mavros"])
def test_image_position_budget_uses_corrected_rover_ellipsoid_and_calibrated_lever(tmp_path, mode):
    root, alignment, path = rtk_fixture(tmp_path, mode=mode)
    output = tmp_path / "accuracy"
    report = image_accuracy(root, alignment, path, output)
    assert report["passed"] and report["qualified_images"] == 3 and report["geo_available"]
    rows = list(jsonl(output / "images.jsonl"))
    assert all(row["qualified"] for row in rows)
    assert rows[0]["budget"]["coordinate_quantization_allowance_horizontal_vertical_m"][0] > 0.007
    assert rows[0]["budget"]["conditional_horizontal_95_plus_allowances_m"] < 0.1
    assert rows[0]["camera_base_enu_m"][0] > 2
    # 100m ellipsoidal input, not the deliberately different 55m MSL field.
    assert rows[0]["projected_camera_xyz_m"][2] == pytest.approx(100, abs=0.001)
    assert "Shared base" in report["base_correlation"]


def test_primary_mavros_unknown_correction_age_is_not_silently_accepted(tmp_path):
    root, alignment, path = rtk_fixture(tmp_path, age=2**32 - 1)
    report = image_accuracy(root, alignment, path, tmp_path / "accuracy")
    assert not report["passed"] and not report["geo_available"]
    rows = list(jsonl(tmp_path / "accuracy/images.jsonl"))
    assert all("Receiver correction age is required" in ";".join(row["reasons"]) for row in rows)
    assert all(row["receiver_correction_age_ms"] is None for row in rows)


def test_cli_qualified_camera_geolocation_reaches_calibrated_odm_inputs(tmp_path, capsys):
    root, alignment, profile = rtk_fixture(tmp_path)
    project, accuracy, run = (tmp_path / p for p in ("project", "accuracy", "terrain-run"))
    metadata = export(root, project, interval=0.5)
    assert (
        main(
            [
                "image-accuracy",
                str(root),
                "--alignment",
                str(alignment),
                "--profile",
                str(profile),
                "--project",
                str(project),
                "--output",
                str(accuracy),
            ]
        )
        == 0
    )
    capsys.readouterr()
    recipe = tmp_path / "terrain.json"
    write_json(recipe, {"product": "terrain", "interval_seconds": 0.5, "gps_accuracy_m": 0.1})
    assert (
        main(
            [
                "process",
                str(root),
                "--config",
                str(recipe),
                "--output",
                str(run),
                "--geo",
                str(accuracy / "camera-geo.txt"),
                "--vertical-datum",
                "WGS84 ellipsoidal height",
                "--prepare-only",
            ]
        )
        == 0
    )
    capsys.readouterr()
    workflow = json.loads((run / "workflow.json").read_text())
    prepared = run / workflow["stages"]["terrain-input"][-1]["path"]
    original_geo = (accuracy / "camera-geo.txt").read_text().splitlines()
    derived_geo = (prepared / "site/geo.txt").read_text().splitlines()
    assert original_geo[0] == derived_geo[0] == "EPSG:32633"
    assert len(derived_geo) == len(metadata["images"]) + 1
    for before, after in zip(original_geo[1:], derived_geo[1:], strict=True):
        assert after.split()[0].endswith(".png")
        np.testing.assert_allclose(
            list(map(float, before.split()[1:])),
            list(map(float, after.split()[1:])),
            atol=1e-8,
            rtol=0,
        )
    assert workflow["inputs"]["config"]["gps_accuracy_m"] == 0.1


def test_explicit_transport_only_policy_keeps_receiver_age_limitation(tmp_path):
    root, alignment, path = rtk_fixture(tmp_path, age=2**32 - 1)
    profile = json.loads(path.read_text())
    profile["policy"]["require_receiver_correction_age"] = False
    write_json(path, profile)
    report = image_accuracy(root, alignment, path, tmp_path / "accuracy")
    assert report["passed"] and not report["fully_documented_receiver_age"]
    assert all(row["limitations"] for row in jsonl(tmp_path / "accuracy/images.jsonl"))


def test_missing_evidence_and_unknown_covariance_produce_partial_report(tmp_path):
    root, alignment, path = rtk_fixture(tmp_path)
    profile = json.loads(path.read_text())
    profile["rig"]["lever_covariance_flu_m2"] = None
    write_json(path, profile)
    report = image_accuracy(root, alignment, path, tmp_path / "accuracy")
    assert not report["passed"] and report["qualified_images"] == 0
    assert all(row["budget"] is None for row in jsonl(tmp_path / "accuracy/images.jsonl"))
    profile["base"]["covariance_enu_m2"] = [[1, 2, 0], [2, 1, 0], [0, 0, 1]]
    write_json(path, profile)
    with pytest.raises(ValueError, match="semidefinite"):
        read_profile(path)
