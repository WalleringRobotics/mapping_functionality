import copy
import json
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from wallering_mapping import opensfm
from wallering_mapping.dataset import write_json
from wallering_mapping.export import export
from wallering_mapping.process import process
from wallering_mapping.process_config import ProcessConfig
from wallering_mapping.simulate import simulate


@pytest.fixture
def prepared(tmp_path):
    session, project, staged = (tmp_path / name for name in ("capture", "project", "prepared"))
    simulate(session, 5)
    export(session, project, interval=0)
    # The mock engine tests orchestration, not the synthetic fixture's geometry.
    metadata = json.loads((project / "project.json").read_text())
    metadata["source_type"] = "test-orchestration"
    write_json(project / "project.json", metadata)
    config = ProcessConfig(backend="opensfm", dense=True, mesh=False, min_sparse_points=3)
    metadata = opensfm.prepare(project, staged, config)
    return staged, metadata, config


def reconstruction(metadata):
    return {
        "cameras": {metadata["camera_id"]: metadata["camera"]},
        "shots": {name: {"camera": metadata["camera_id"], "rotation": [0, 0, 0],
                          "translation": [index, 0, 0]}
                  for index, name in enumerate(metadata["image_mapping"].values())},
        "points": {str(index): {"coordinates": [index, index + 1, 3], "color": [1, 2, 3]}
                   for index in range(3)},
    }


def test_native_preparation_intrinsics_masks_and_provenance(prepared):
    staged, metadata, config = prepared
    dataset = staged / "dataset"
    assert metadata["status"] == "ready"
    exif = json.loads((dataset / "exif_overrides.json").read_text())
    native_config = json.loads((dataset / "config.yaml").read_text())
    assert not native_config["optimize_camera_parameters"]
    assert not native_config["bundle_use_gps"]
    assert native_config["matching_order_neighbors"] == 0
    assert native_config["processes"] == config.max_concurrency
    for name, values in exif.items():
        assert "gps" not in values and "capture_time" not in values
        assert values["camera"] == metadata["camera_id"]
        mask = cv2.imread(str(dataset / "masks" / (name + ".png")), cv2.IMREAD_GRAYSCALE)
        assert mask.shape == (metadata["camera"]["height"], metadata["camera"]["width"])
        assert 0 < np.mean(mask > 0) < 1
    opensfm.verify_inventory(staged, metadata["artifacts"])
    assert (staged / "source-selection.jsonl").is_file()


def test_prepare_process_and_resume_without_engine(tmp_path):
    session = tmp_path / "capture"
    simulate(session, 5)
    config = ProcessConfig(backend="opensfm", mesh=False, interval_seconds=0)
    output = tmp_path / "run"
    planned = process(session, output, config)
    assert planned["backend"] == "opensfm"
    assert planned["stages"] == ["export", "opensfm-input", "opensfm"]
    assert not output.exists()
    result = process(session, output, config, prepare_only=True)
    assert result["status"] == "prepared"
    resumed = process(session, output, config, prepare_only=True, resume=True)
    assert len(resumed["stages"]["opensfm-input"]) == 1
    path = next((output / "stages/opensfm-input-001/dataset/masks").iterdir())
    path.write_bytes(b"changed")
    with pytest.raises(ValueError, match="changed"):
        process(session, output, config, prepare_only=True, resume=True)


@pytest.mark.parametrize("change,expected", [
    ("unknown-image", "unexpected images"),
    ("calibration", "calibration mismatch"),
    ("camera", "unexpected camera"),
    ("points", "quality gates"),
    ("registration", "quality gates"),
    ("pose", "invalid camera poses"),
    ("coordinates", "invalid sparse coordinates"),
])
def test_quality_failures(prepared, change, expected):
    staged, metadata, config = prepared
    model = reconstruction(metadata)
    name = next(iter(model["shots"]))
    if change == "unknown-image":
        model["shots"]["unknown.png"] = model["shots"].pop(name)
    elif change == "calibration":
        model = copy.deepcopy(model)
        model["cameras"][metadata["camera_id"]]["focal_x"] += 0.01
    elif change == "camera":
        model["shots"][name]["camera"] = "wrong"
    elif change == "points":
        model["points"].clear()
    elif change == "registration":
        model["shots"].pop(name)
    elif change == "pose":
        model["shots"][name]["translation"] = [0, 0]
    elif change == "coordinates":
        model["points"]["0"]["coordinates"] = [0, 0]
    write_json(staged / "dataset/reconstruction.json", [model])
    with pytest.raises(ValueError, match=expected):
        opensfm.assess_sparse(staged / "dataset", metadata, config)


def test_component_selection_is_explicit_and_preserves_native_result(prepared):
    staged, metadata, config = prepared
    models = [reconstruction(metadata), reconstruction(metadata)]
    dataset = staged / "dataset"
    write_json(dataset / "reconstruction.json", models)
    with pytest.raises(ValueError, match="Multiple sparse"):
        opensfm.assess_sparse(dataset, metadata, config)
    quality = opensfm.assess_sparse(dataset, metadata, replace(config, model_index=1))
    assert quality["passed"] and quality["components"] == 2 and quality["model_index"] == 1
    assert json.loads((dataset / "reconstruction-all.json").read_text()) == models
    assert json.loads((dataset / "reconstruction.json").read_text()) == [models[1]]


def mock_engine(monkeypatch, metadata, fail_quality=False):
    calls = []
    monkeypatch.setattr(opensfm, "resolve_engine", lambda image: {"id": "sha256:test"})

    def run(command, log):
        action = command[command.index(opensfm.ENGINE_ROOT + "/bin/opensfm_main.py") + 1]
        calls.append(action)
        log.write_text("mock engine")
        dataset = log.parent.parent / "dataset"
        if action == "extract_metadata":
            write_json(dataset / "camera_models.json", {metadata["camera_id"]: metadata["camera"]})
        elif action == "reconstruct":
            model = reconstruction(metadata)
            if fail_quality:
                model["points"].clear()
            write_json(dataset / "reconstruction.json", [model])
        elif action == "create_tracks":
            (dataset / "tracks.csv").write_text("upstream tracks placeholder\n")
        elif action in {"export_ply", "compute_depthmaps"}:
            cloud = dataset / ("reconstruction.ply" if action == "export_ply"
                               else "undistorted/depthmaps/merged.ply")
            cloud.parent.mkdir(parents=True, exist_ok=True)
            cloud.write_text("ply\nformat ascii 1.0\nelement vertex 1\nproperty float x\n"
                             "property float y\nproperty float z\nend_header\n0 0 1\n")

    monkeypatch.setattr(opensfm, "run_logged", run)
    return calls


def test_native_command_sequence_and_sealed_products(prepared, tmp_path, monkeypatch):
    staged, metadata, config = prepared
    calls = mock_engine(monkeypatch, metadata)
    result = opensfm.execute(staged, tmp_path / "run", config)
    assert calls == ["extract_metadata", "detect_features", "match_features", "create_tracks",
                     "reconstruct", "export_ply", "undistort", "compute_depthmaps"]
    assert result["status"] == "complete" and result["quality"]["passed"]
    opensfm.verify_inventory(tmp_path / "run", result["products"])
    assert result["commands"][0][-1] == "/datasets/dataset"
    assert all(command[command.index("--network") + 1] == "none" for command in result["commands"])


def test_process_executes_opensfm_and_reuses_sealed_run(prepared, tmp_path, monkeypatch):
    _, metadata, config = prepared
    session = tmp_path / "capture"
    manifest = json.loads((session / "manifest.json").read_text())
    manifest["source"] = "test-orchestration"
    write_json(session / "manifest.json", manifest)
    calls = mock_engine(monkeypatch, metadata)
    output = tmp_path / "workflow"
    result = process(session, output, config, execute=True)
    assert result["status"] == "complete"
    assert result["quality"]["registered_fraction"] == 1
    assert len(calls) == 8
    process(session, output, config, execute=True, resume=True)
    assert len(calls) == 8


def test_failed_registration_stops_dense_and_retains_evidence(prepared, tmp_path, monkeypatch):
    staged, metadata, config = prepared
    calls = mock_engine(monkeypatch, metadata, fail_quality=True)
    cleanup = []
    monkeypatch.setattr(opensfm.subprocess, "run", lambda command, **kw: cleanup.append(command))
    with pytest.raises(ValueError, match="quality gates"):
        opensfm.execute(staged, tmp_path / "run", config)
    assert "undistort" not in calls and "export_ply" not in calls
    assert cleanup[0][:3] == ["docker", "rm", "-f"]
    assert cleanup[0][3].startswith("wr-opensfm-")
    state = json.loads((tmp_path / "run/run.json").read_text())
    assert state["status"] == "failed"
    assert not json.loads((tmp_path / "run/quality.json").read_text())["passed"]


def test_input_tampering_rejected_before_engine(prepared, tmp_path, monkeypatch):
    staged, _, config = prepared
    monkeypatch.setattr(opensfm, "resolve_engine", lambda image: pytest.fail("Engine must not run"))
    (staged / "dataset/config.yaml").write_text("{}")
    with pytest.raises(ValueError, match="changed"):
        opensfm.execute(staged, tmp_path / "run", config)


def test_engine_interface_pin(prepared, monkeypatch):
    monkeypatch.setattr(opensfm.odm, "resolve_image", lambda image: {"id": "sha256:test"})
    result = {"source_sha256": opensfm.SOURCE_SHA256, "native_sha256": {"pymap": "hash"}}
    monkeypatch.setattr(opensfm.subprocess, "run", lambda *a, **kw: SimpleNamespace(stdout=json.dumps(result)))
    assert opensfm.resolve_engine("image")["opensfm_revision"] == opensfm.OPENSFM_REVISION
    result["source_sha256"] = {}
    with pytest.raises(ValueError, match="source differs"):
        opensfm.resolve_engine("image")


def test_no_empty_cloud_success(tmp_path):
    cloud = tmp_path / "empty.ply"
    cloud.write_text("ply\nformat ascii 1.0\nelement vertex 0\nend_header\n")
    with pytest.raises(ValueError, match="empty/invalid"):
        opensfm.require_cloud(cloud)


@pytest.mark.parametrize("values", [
    {"backend": "unknown"}, {"backend": "odm"}, {"backend": "opensfm"},
    {"backend": "colmap", "product": "terrain"},
])
def test_incompatible_recipes_rejected(values):
    with pytest.raises(ValueError):
        ProcessConfig(**values)


def test_old_recipe_serialization_retained():
    assert "backend" not in ProcessConfig().to_dict()
    assert ProcessConfig().resolved_backend == "colmap"
    assert ProcessConfig(product="terrain").resolved_backend == "odm"
    assert ProcessConfig.read(Path("configs/process-opensfm.json")).resolved_backend == "opensfm"
