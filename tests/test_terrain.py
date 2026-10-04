import json

import cv2
import numpy as np
import pytest

from wallering_mapping.control import gcps, geolocation
from wallering_mapping.export import export
from wallering_mapping.odm import camera_geometry, docker_command, prepare, undistort_pixel
from wallering_mapping.process import process
from wallering_mapping.process_config import ProcessConfig
from wallering_mapping.simulate import simulate


@pytest.mark.parametrize(
    "model,distortion",
    [
        ("FULL_OPENCV", [0.05, -0.01, 0.001, -0.002, 0.002, 0.005, -0.001, 0.0002]),
        ("OPENCV_FISHEYE", [0.01, -0.002, 0.001, -0.0001]),
    ],
)
def test_distortion_conversion_preserves_calibrated_ray(model, distortion):
    camera = {
        "model": model,
        "width": 640,
        "height": 480,
        "params": [500, 510, 319, 241, *distortion],
    }
    k, d, _, mask, normalized = camera_geometry(camera)
    point = np.array([[[0.2, -0.1, 1]]], float)
    if model == "FULL_OPENCV":
        pixel = cv2.projectPoints(point, np.zeros(3), np.zeros(3), k, d)[0]
    else:
        pixel = cv2.fisheye.projectPoints(point, np.zeros(3), np.zeros(3), k, d)[0]
    np.testing.assert_allclose(undistort_pixel(pixel, camera), [419, 190], atol=1e-7)
    assert mask[190, 419] == 255
    assert normalized["c_x"] == pytest.approx(-0.5 / 640)
    assert normalized["c_y"] == pytest.approx(1.5 / 640)


def control_file(path, names):
    lines = ["EPSG:32632"]
    for index, (x, y) in enumerate([(0, 0), (10, 0), (0, 10), (10, 10), (5, 5)]):
        lines += [f"{500000 + x} {6000000 + y} 100 320 240 {name} p{index}" for name in names[:3]]
    path.write_text("\n".join(lines) + "\n")
    return path


def test_terrain_preparation_and_control_coordinate_trace(tmp_path):
    source, project, prepared = (tmp_path / p for p in ("source", "project", "prepared"))
    simulate(source, 6)
    metadata = export(source, project)
    names = [i["name"] for i in metadata["images"]]
    controls = control_file(tmp_path / "controls.txt", names)
    result = prepare(project, prepared, gcp_path=controls, vertical_datum="EGM2008")
    assert result["status"] == "ready" and result["gcp_points"] == 5
    assert result["camera_id"] == "v2   640 480 brown 0.85"
    assert len(list((prepared / "site/images").glob("*_mask.png"))) == 3
    original = (project / "images" / names[0]).read_bytes()
    row = (prepared / "site/gcp_list.txt").read_text().splitlines()[1].split()
    np.testing.assert_allclose(list(map(float, row[3:5])), [320, 240], atol=1e-8)
    assert row[5].endswith(".png")
    assert (project / "images" / names[0]).read_bytes() == original
    root = tmp_path / "run"
    result = process(
        source,
        root,
        ProcessConfig(product="terrain", interval_seconds=1),
        prepare_only=True,
        gcp=controls,
        vertical_datum="EGM2008",
    )
    assert result["status"] == "prepared"
    assert set(result["stages"]) == {"export", "terrain-input"}
    resumed = process(
        source,
        root,
        ProcessConfig(product="terrain", interval_seconds=1),
        prepare_only=True,
        resume=True,
        gcp=controls,
        vertical_datum="EGM2008",
    )
    assert len(resumed["stages"]["terrain-input"]) == 1


def test_controls_reject_missing_views_bad_pixels_and_unknown_images(tmp_path):
    names = ["a.jpg", "b.jpg", "c.jpg"]
    file = control_file(tmp_path / "gcp.txt", names)
    assert gcps(file, set(names), 640, 480)["points"] == 5
    with pytest.raises(ValueError, match="not selected"):
        gcps(file, {"a.jpg"}, 640, 480)
    with pytest.raises(ValueError, match="outside"):
        gcps(file, set(names), 200, 200)
    file.write_text("\n".join(file.read_text().splitlines()[:4]))
    with pytest.raises(ValueError, match="5 distinct"):
        gcps(file, set(names), 640, 480)


def test_geolocation_and_explicit_docker_arguments(tmp_path):
    geo = tmp_path / "geo.txt"
    geo.write_text(
        "EPSG:32632\na.png 500000 6000000 100\nb.png 500010 6000000 100\nc.png 500000 6000010 100\n"
    )
    assert len(geolocation(geo, {"a.png", "b.png", "c.png"})["records"]) == 3
    (tmp_path / "site").mkdir()
    (tmp_path / "site/geo.txt").write_text(geo.read_text())
    command = docker_command(
        "sha256:fixture", tmp_path, "owned-run", ProcessConfig(product="terrain"), "dataset"
    )
    assert command[-1] == "site"
    assert command[command.index("--end-with") + 1] == "dataset"
    assert "--use-fixed-camera-params" in command and "--geo" in command
    assert command[command.index("--network") + 1] == "none"


def test_odm_failed_run_retains_evidence_and_cleans_owned_container(tmp_path, monkeypatch):
    import wallering_mapping.odm as odm

    prepared = tmp_path / "prepared"
    (prepared / "site").mkdir(parents=True)
    (prepared / "preparation.json").write_text(
        json.dumps({"status": "ready", "source_type": "test"})
    )
    monkeypatch.setattr(odm, "resolve_image", lambda _: {"id": "sha256:fixture"})
    cleaned = []
    monkeypatch.setattr(odm.subprocess, "run", lambda command, **_: cleaned.append(command))

    def fail(*_):
        raise InterruptedError("controlled cancellation")

    monkeypatch.setattr(odm, "run_logged", fail)
    with pytest.raises(InterruptedError):
        odm.execute(prepared, tmp_path / "run", ProcessConfig(product="terrain"))
    state = json.loads((tmp_path / "run/run.json").read_text())
    assert state["status"] == "failed"
    assert cleaned[0][:3] == ["docker", "rm", "-f"]
    assert cleaned[0][3].startswith("wr-mapping-")


@pytest.mark.parametrize("drift", [False, True])
def test_odm_engine_contract_and_final_camera_drift_gate(tmp_path, monkeypatch, drift):
    """Controlled engine output contract; this does not run real ODM reconstruction."""
    import wallering_mapping.odm as odm

    source, project, prepared, output = (
        tmp_path / name for name in ("source", "project", "prepared", "run")
    )
    simulate(source, 6)
    metadata = export(source, project)
    names = [item["name"] for item in metadata["images"]]
    controls = control_file(tmp_path / "control.txt", names)
    metadata = prepare(project, prepared, gcp_path=controls, vertical_datum="EGM2008")
    metadata["source_type"] = "test-orchestration"
    (prepared / "preparation.json").write_text(json.dumps(metadata))
    monkeypatch.setattr(odm, "resolve_image", lambda _: {"id": "sha256:fixture"})
    monkeypatch.setattr(odm.subprocess, "run", lambda *a, **kw: None)
    calls = []

    def engine(command, log):
        calls.append(command)
        log.write_text("Controlled engine fixture, not a reconstruction")
        if len(calls) == 1:
            photos = [
                {
                    "filename": name,
                    "camera_make": "",
                    "camera_model": "",
                    "width": 640,
                    "height": 480,
                    "camera_projection": "brown",
                    "focal_ratio": 0.85,
                }
                for name in metadata["image_mapping"].values()
            ]
            (output / "site/images.json").write_text(json.dumps(photos))
        elif len(calls) == 2:
            root = output / "site/opensfm"
            root.mkdir()
            cameras = {metadata["camera_id"]: metadata["camera"]}
            (root / "camera_models.json").write_text(json.dumps(cameras))
            fitted = json.loads(json.dumps(cameras))
            if drift:
                fitted[metadata["camera_id"]]["focal_x"] += 0.01
            reconstruction = [
                {
                    "cameras": fitted,
                    "shots": {name: {} for name in metadata["image_mapping"].values()},
                    "points": {str(i): {} for i in range(100)},
                }
            ]
            (root / "reconstruction.json").write_text(json.dumps(reconstruction))
        else:
            for name in (
                "odm_orthophoto/odm_orthophoto.tif",
                "odm_dem/dsm.tif",
                "odm_georeferencing/odm_georeferenced_model.laz",
                "odm_texturing/odm_textured_model_geo.obj",
                "odm_texturing/odm_textured_model_geo.mtl",
                "odm_texturing/texture.jpg",
            ):
                path = output / "site" / name
                path.parent.mkdir(exist_ok=True)
                path.write_text("fixture; no raster or geometry semantics")

    monkeypatch.setattr(odm, "run_logged", engine)
    if drift:
        with pytest.raises(ValueError, match="calibration mismatch"):
            odm.execute(prepared, output, ProcessConfig(product="terrain"))
        assert len(calls) == 2  # Stop before dense work with incorrect calibration.
    else:
        result = odm.execute(prepared, output, ProcessConfig(product="terrain"))
        assert result["status"] == "complete" and len(result["products"]) == 6
        assert len(calls) == 3


def test_explicit_rtk_prior_accuracy_is_passed_to_pinned_odm(tmp_path):
    from wallering_mapping.odm import docker_command
    from wallering_mapping.process_config import ProcessConfig

    site = tmp_path / "site"
    site.mkdir()
    (site / "geo.txt").write_text("EPSG:32633\n")
    config = ProcessConfig(product="terrain", gps_accuracy_m=0.08)
    command = docker_command(config.odm_image, tmp_path, "rtk-test", config)
    assert command[command.index("--gps-accuracy") + 1] == "0.08"
    with pytest.raises(ValueError, match="gps_accuracy_m"):
        ProcessConfig(gps_accuracy_m=0)
