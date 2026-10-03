import json
import subprocess
import sys

import pytest

from wallering_mapping.accuracy import checkpoints
from wallering_mapping.export import export
from wallering_mapping.reconstruct import dense_plan, execute, sparse_plan
from wallering_mapping.simulate import simulate


@pytest.fixture
def project(tmp_path):
    source, project = tmp_path / "source", tmp_path / "project with spaces"
    simulate(source, 6)
    export(source, project)
    return project


def test_plan_passes_calibration_and_keeps_argv_paths(project, tmp_path):
    output = tmp_path / "result"
    plan = sparse_plan(project, output, cpu=True)
    assert not output.exists()
    feature = plan["commands"][0]
    assert str(project / "images") in feature
    assert feature[feature.index("--ImageReader.camera_model") + 1] == "FULL_OPENCV"
    assert feature[feature.index("--SiftExtraction.use_gpu") + 1] == "0"
    assert plan["commands"][1][1] == "exhaustive_matcher"


def test_changed_export_rejected(project, tmp_path):
    next((project / "images").iterdir()).write_bytes(b"changed")
    with pytest.raises(ValueError, match="changed"):
        sparse_plan(project, tmp_path / "result")


def test_execute_failure_is_logged(project, tmp_path, monkeypatch):
    # Command runner integration with controlled engine failure, not an SfM accuracy test.
    manifest = project / "project.json"
    metadata = json.loads(manifest.read_text())
    metadata["source_type"] = "test"
    manifest.write_text(json.dumps(metadata))
    output = tmp_path / "result"
    engine = tmp_path / "fake-colmap"
    engine.write_text(f"#!{sys.executable}\nimport sys\n"
                      "if sys.argv[-1] == '-h': print('COLMAP 3.12.6')\n"
                      "else: print('controlled failure'); sys.exit(1)\n")
    engine.chmod(0o755)
    plan = sparse_plan(project, output, executable=str(engine))

    with pytest.raises(subprocess.CalledProcessError):
        execute(plan)
    assert json.loads((output / "run.json").read_text())["status"] == "failed"


def test_dense_requires_explicit_real_model(project, tmp_path):
    with pytest.raises(ValueError, match="existing binary"):
        dense_plan(project, tmp_path / "missing", tmp_path / "result")


def test_known_checkpoint_errors(tmp_path):
    path = tmp_path / "checks.csv"
    path.write_text("id,role,reference_x_m,reference_y_m,reference_z_m,model_x_m,model_y_m,model_z_m\n"
                    "1,check,0,0,0,0.003,0.004,0\n"
                    "2,check,1,0,0,1.003,0.004,0\n"
                    "3,check,0,1,0,0.003,1.004,0\n")
    assert checkpoints(path)["rmse_3d_m"] == pytest.approx(0.005)
    path.write_text(path.read_text().replace("2,check", "2,control"))
    with pytest.raises(ValueError, match="withheld"):
        checkpoints(path)

