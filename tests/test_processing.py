import json
from dataclasses import replace

import numpy as np
import pycolmap
import pytest

from wallering_mapping.dataset import write_json
from wallering_mapping.export import export
from wallering_mapping.model import assess_model
from wallering_mapping.process import Workflow, process
from wallering_mapping.process_config import ProcessConfig
from wallering_mapping.simulate import simulate


@pytest.fixture
def session(tmp_path):
    path = tmp_path / "capture"
    simulate(path, 10)
    return path


def synthesized_model(project, output):
    """Real COLMAP binary geometry; unrelated to the IO fixture's pixel content."""
    reconstruction = pycolmap.synthesize_dataset(pycolmap.SyntheticDatasetOptions())
    names = [item["name"] for item in json.loads((project / "project.json").read_text())["images"]]
    for image, name in zip(reconstruction.images.values(), names, strict=True):
        image.name = name
    reconstruction.update_point_3d_errors()
    output.mkdir(parents=True)
    reconstruction.write(str(output))
    return reconstruction


def test_plan_preparation_resume_and_tamper(session, tmp_path):
    root = tmp_path / "run"
    config = ProcessConfig()
    assert process(session, root, config)["status"] == "planned"
    assert not root.exists()
    first = process(session, root, config, prepare_only=True)
    assert first["status"] == "prepared"
    resumed = process(session, root, config, prepare_only=True, resume=True)
    assert len(resumed["stages"]["export"]) == 1
    with pytest.raises(ValueError, match="inputs/config changed"):
        process(session, root, replace(config, min_sharpness=1), prepare_only=True, resume=True)
    image = next((root / "stages/export-001/images").iterdir())
    image.write_bytes(b"altered")
    with pytest.raises(ValueError, match="changed"):
        process(session, root, config, prepare_only=True, resume=True)
    assert json.loads((root / "workflow.json").read_text())["status"] == "failed"


def test_synthetic_execution_and_input_nesting_rejected(session, tmp_path):
    with pytest.raises(ValueError, match="Synthetic"):
        process(session, tmp_path / "run", ProcessConfig(), execute=True)
    with pytest.raises(ValueError, match="separate"):
        process(session, session / "output", ProcessConfig(), prepare_only=True)
    with pytest.raises(ValueError, match="Terrain needs"):
        process(session, tmp_path / "run", ProcessConfig(product="terrain"))


def test_attempt_failure_retained_and_retry_is_fresh(tmp_path):
    workflow = Workflow(tmp_path / "run", {"fixture": True}, False)

    def fail(path):
        path.mkdir()
        (path / "failure.log").write_text("evidence")
        raise RuntimeError("engine interrupted")

    with pytest.raises(RuntimeError):
        workflow.stage("engine", fail)
    assert (tmp_path / "run/stages/engine-001/failure.log").read_text() == "evidence"

    def success(path):
        path.mkdir()
        (path / "product").write_text("result")
        return {"done": True}

    output, result = workflow.stage("engine", success)
    assert output.name == "engine-002" and result["done"]
    assert workflow.stage("engine", fail)[0] == output


def test_real_binary_model_quality_and_poses(session, tmp_path):
    project = tmp_path / "project"
    export(session, project, interval=0)
    reconstruction = synthesized_model(project, tmp_path / "sparse/0")
    result = assess_model(project, tmp_path / "sparse", ProcessConfig(), tmp_path / "quality")
    assert result["passed"] and result["registered_fraction"] == 1
    assert result["sparse_points"] == 100
    assert result["reprojection_error_px"]["p95"] < 1e-8
    rows = np.genfromtxt(tmp_path / "quality/camera-poses.csv", delimiter=",", skip_header=1,
                         usecols=(2, 3, 4))
    expected = [-(i.cam_from_world().rotation.matrix().T @ i.cam_from_world().translation)
                for i in sorted(reconstruction.images.values(), key=lambda i: i.name)]
    np.testing.assert_allclose(rows, expected, atol=1e-10)
    with pytest.raises(ValueError, match="quality gate"):
        assess_model(project, tmp_path / "sparse", ProcessConfig(min_sparse_points=101),
                     tmp_path / "rejected")
    assert not json.loads((tmp_path / "rejected/quality.json").read_text())["passed"]
    (tmp_path / "sparse/1").mkdir()
    reconstruction.write(str(tmp_path / "sparse/1"))
    with pytest.raises(ValueError, match="Multiple sparse"):
        assess_model(project, tmp_path / "sparse", ProcessConfig(), tmp_path / "ambiguous")


def test_full_building_orchestration_and_resume(session, tmp_path, monkeypatch):
    # Engines are simulated; model IO and quality assessment use real pycolmap.
    import wallering_mapping.process as workflow_module
    metadata = json.loads((session / "manifest.json").read_text())
    metadata["source"] = "test-orchestration"
    write_json(session / "manifest.json", metadata)
    calls = []

    def engine(plan):
        from pathlib import Path
        calls.append(plan["kind"])
        output = Path(plan["output"])
        if plan["kind"] == "sparse":
            synthesized_model(Path(plan["source"]), output / "sparse/0")
        else:
            product = output / ("dense/fused.ply" if plan["kind"] == "dense" else "mesh.ply")
            product.parent.mkdir(parents=True)
            product.write_text("mock engine product; no metric accuracy claim")
        return {"status": "complete"}

    monkeypatch.setattr(workflow_module.reconstruct, "execute", engine)
    root = tmp_path / "run"
    result = process(session, root, ProcessConfig(), execute=True)
    assert result["status"] == "complete"
    assert calls == ["sparse", "dense", "mesh"]
    assert any(item["path"].endswith("mesh.ply") for item in result["products"])
    process(session, root, ProcessConfig(), execute=True, resume=True)
    assert calls == ["sparse", "dense", "mesh"]


@pytest.mark.parametrize("values", [
    {"product": "unknown"}, {"dense": False}, {"min_registered_fraction": float("nan")},
    {"max_image_size": 128.5}, {"odm_image": "opendronemap/odm:latest"},
    {"odm_image": "opendronemap/odm@sha256:bad"}, {"cpu": 1},
])
def test_recipe_rejects_invalid_values(values):
    with pytest.raises(ValueError):
        ProcessConfig(**values)
