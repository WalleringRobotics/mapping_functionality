import json

import pytest

from wallering_mapping.cli import main
from wallering_mapping.dataset import jsonl, sha256_file
from wallering_mapping.export import colmap_camera, export
from wallering_mapping.simulate import simulate
from wallering_mapping.validate import nearest_offsets, validate


@pytest.fixture
def capture(tmp_path):
    root = tmp_path / "capture"
    simulate(root, 6)
    return root


def reseal_test_journal(root, name):
    """Intentionally construct an internally consistent fixture with a semantic fault."""
    path = root / "manifest.json"
    manifest = json.loads(path.read_text())
    manifest["journals_sha256"][name] = sha256_file(root / f"{name}.jsonl")
    path.write_text(json.dumps(manifest))


def test_valid_and_selected_originals(capture, tmp_path):
    report = validate(capture)
    assert report["valid"], report
    assert report["streams"]["rgb"]["observed_fps"] == 2
    result = export(capture, tmp_path / "project", interval=1)
    assert len(result["images"]) == 3
    assert result["camera"]["model"] == "FULL_OPENCV"
    for image in result["images"]:
        assert (tmp_path / "project/images" / image["name"]).read_bytes() == (
            capture / "images/rgb" / image["name"]).read_bytes()
    decisions = list(jsonl(tmp_path / "project/selection.jsonl"))
    assert [x["sequence"] for x in decisions if x["decision"] == "selected"] == [0, 2, 4]


def test_corruption_blocks_export(capture, tmp_path):
    first = next((capture / "images/rgb").glob("*.jpg"))
    first.write_bytes(b"corrupt")
    assert not validate(capture)["valid"]
    with pytest.raises(ValueError, match="validation"):
        export(capture, tmp_path / "project")
    assert not (tmp_path / "project").exists()


def test_torn_log_and_missing_file(capture):
    with (capture / "frames.jsonl").open("a") as file:
        file.write('{"stream":')
    result = validate(capture)
    assert not result["valid"]
    assert any("incomplete/invalid" in error for error in result["errors"])


def test_incomplete_session(capture):
    path = capture / "manifest.json"
    manifest = json.loads(path.read_text())
    manifest["status"] = "recording"
    path.write_text(json.dumps(manifest))
    assert not validate(capture)["valid"]
    assert main(["validate", str(capture)]) == 2


def test_frame_gap_requires_explicit_acceptance(capture, tmp_path):
    file = capture / "frames.jsonl"
    rows = list(jsonl(file))
    for row in rows:
        if row["stream"] == "rgb" and row["sequence"] >= 3:
            row["sequence"] += 1
    file.write_text("".join(json.dumps(row) + "\n" for row in rows))
    reseal_test_journal(capture, "frames")
    with pytest.raises(ValueError, match="gaps"):
        export(capture, tmp_path / "project")
    assert export(capture, tmp_path / "accepted", allow_gaps=True)["status"] == "ready"


def test_focus_change_rejected(capture, tmp_path):
    file = capture / "frames.jsonl"
    rows = list(jsonl(file))
    rows[0]["lens_position"] += 1
    file.write_text("".join(json.dumps(row) + "\n" for row in rows))
    reseal_test_journal(capture, "frames")
    with pytest.raises(ValueError, match="focus"):
        export(capture, tmp_path / "project")


def test_unsupported_lens_terms():
    camera = {"K": [[1, 0, 0], [0, 1, 0], [0, 0, 1]], "model": "CameraModel.Perspective",
              "distortion": [0] * 8 + [0.1]}
    with pytest.raises(ValueError, match="thin prism"):
        colmap_camera(camera)


def test_failed_selection_marked(capture, tmp_path):
    with pytest.raises(ValueError, match="Fewer than"):
        export(capture, tmp_path / "project", min_sharpness=1e12)
    assert json.loads((tmp_path / "project/project.json").read_text())["status"] == "failed"


def test_nearest_timing_report():
    assert nearest_offsets([0, 10_000_000], [1_000_000, 11_000_000]) == [1, 1]


def test_metadata_transfer_corruption_detected(capture):
    path = capture / "clock.jsonl"
    path.write_text('{"host_utc_ns":123}\n')
    result = validate(capture)
    assert not result["valid"]
    assert "Journal checksum mismatch: clock" in result["errors"]


def test_export_does_not_mutate_source(capture):
    with pytest.raises(ValueError, match="outside"):
        export(capture, capture / "derived")
    assert not (capture / "derived").exists()
