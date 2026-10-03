import json

import numpy as np
import pytest

from test_processing import synthesized_model
from wallering_mapping.dataset import jsonl, sha256_file, write_json
from wallering_mapping.export import export
from wallering_mapping.georeference import georeference, similarity, transform_ply
from wallering_mapping.model import pycolmap
from wallering_mapping.simulate import simulate


def rotation():
    a = 0.3
    return np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1]])


def test_weighted_similarity_recovers_metric_scale_rotation_and_origin():
    source = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [2, 3, 0], [1, 1, 2]], float)
    r = rotation()
    target = 1.7 * source @ r.T + [400000, 5700000, 100]
    s, recovered, t = similarity(source, target, [1, 2, 3, 4, 5])
    assert s == pytest.approx(1.7)
    np.testing.assert_allclose(recovered, r, atol=1e-9)
    np.testing.assert_allclose(s * source @ recovered.T + t, target, atol=1e-8)
    with pytest.raises(ValueError, match="collinear"):
        similarity(np.arange(5)[:, None] * np.ones((1, 3)), target, np.ones(5))


@pytest.mark.parametrize("endian", ["<", ">"])
def test_binary_ply_double_vertices_rotated_normals_colors_and_faces_preserved(tmp_path, endian):
    dtype = np.dtype(
        [(a, endian + "f4") for a in ("x", "y", "z", "nx", "ny", "nz")] + [("red", "u1")]
    )
    vertices = np.zeros(3, dtype=dtype)
    vertices["x"] = [1, 2, 3]
    vertices["y"] = [4, 5, 6]
    vertices["z"] = [7, 8, 9]
    vertices["nx"] = 1
    vertices["red"] = [10, 20, 30]
    fmt = "binary_little_endian" if endian == "<" else "binary_big_endian"
    header = f"ply\nformat {fmt} 1.0\nelement vertex 3\n"
    header += "".join(f"property float {a}\n" for a in ("x", "y", "z", "nx", "ny", "nz"))
    header += (
        "property uchar red\nelement face 1\nproperty list uchar int vertex_indices\nend_header\n"
    )
    faces = bytes([3]) + np.array([0, 1, 2], dtype=endian + "i4").tobytes()
    src = tmp_path / "source.ply"
    out = tmp_path / "derived.ply"
    src.write_bytes(header.encode() + vertices.tobytes() + faces)
    transform_ply(src, out, 2, rotation(), np.array([0.1, 0.2, 0.3]))
    with out.open("rb") as file:
        lines = []
        while True:
            line = file.readline()
            lines.append(line)
            if line == b"end_header\n":
                break
        newdtype = np.dtype(
            [
                (a, endian + ("f8" if a in "xyz" else "f4"))
                for a in ("x", "y", "z", "nx", "ny", "nz")
            ]
            + [("red", "u1")]
        )
        points = np.frombuffer(file.read(3 * newdtype.itemsize), dtype=newdtype)
        assert file.read() == faces
    expected = 2 * np.column_stack([vertices[a] for a in "xyz"]) @ rotation().T + [0.1, 0.2, 0.3]
    np.testing.assert_allclose(np.column_stack([points[a] for a in "xyz"]), expected, atol=1e-12)
    np.testing.assert_allclose(
        [points["nx"][0], points["ny"][0], points["nz"][0]], rotation() @ [1, 0, 0], atol=1e-7
    )
    assert points["red"].tolist() == [10, 20, 30]
    assert b"property double x\n" in lines


def test_real_colmap_binary_and_ascii_product_alignment_keeps_local_precision(tmp_path):
    source, project = tmp_path / "capture", tmp_path / "project"
    simulate(source, 10)
    export(source, project, interval=0)
    model = tmp_path / "model"
    recon = synthesized_model(project, model)
    metadata = json.loads((project / "project.json").read_text())
    accuracy = tmp_path / "image-accuracy"
    accuracy.mkdir()
    r, s = rotation(), 1.7
    offset = np.array([400000, 5700000, 100])
    rows = []
    source_centres = {}
    for image in recon.images.values():
        pose = image.cam_from_world()
        centre = -(pose.rotation.matrix().T @ pose.translation)
        source_centres[image.name] = centre
        rows.append(
            {
                "name": image.name,
                "qualified": True,
                "projected_camera_xyz_m": (s * r @ centre + offset).tolist(),
                "budget": {
                    "components_covariance_enu_m2": {
                        "rover": (np.eye(3) * 0.0001).tolist(),
                        "shared_base": (np.eye(3) * 0.0004).tolist(),
                    },
                    "deterministic_motion_allowance_m": 0.001,
                },
            }
        )
    (accuracy / "images.jsonl").write_text("".join(json.dumps(row) + "\n" for row in rows))
    write_json(
        accuracy / "report.json",
        {
            "status": "complete",
            "passed": True,
            "stream": "rgb",
            "source_manifest_sha256": metadata["source_manifest_sha256"],
            "output_hashes": {"images.jsonl": sha256_file(accuracy / "images.jsonl")},
            "profile": {"policy": {"output_crs": "EPSG:32633"}},
            "vertical_datum": "WGS84 ellipsoidal height",
        },
    )
    cloud = tmp_path / "cloud.ply"
    cloud.write_text(
        "ply\nformat ascii 1.0\nelement vertex 3\nproperty float x\nproperty float y\nproperty float z\nelement face 1\nproperty list uchar int vertex_indices\nend_header\n1 2 3\n4 5 6\n7 8 9\n3 0 1 2\n"
    )
    output = tmp_path / "georeference"
    report = georeference(project, model, accuracy, output, [cloud], 0.01)
    assert report["status"] == "complete" and report["scale"] == pytest.approx(s)
    transformed = pycolmap().Reconstruction(str(output / "model"))
    origin = np.array(report["coordinate_origin_xyz_m"])
    for image in transformed.images.values():
        pose = image.cam_from_world()
        local = -(pose.rotation.matrix().T @ pose.translation)
        expected = s * r @ source_centres[image.name] + offset
        np.testing.assert_allclose(local + origin, expected, atol=1e-8)
        assert np.linalg.norm(local) < 100
    data = (output / "products/cloud.ply").read_text().split("end_header\n")[1].splitlines()
    first = np.array(list(map(float, data[0].split())))
    np.testing.assert_allclose(first + origin, s * r @ [1, 2, 3] + offset, atol=1e-8)
    assert data[-1] == "3 0 1 2"
    assert all(
        row["role"] == "navigation_control"
        for row in __import__("csv").DictReader((output / "camera-controls.csv").open())
    )
    assert list(jsonl(accuracy / "images.jsonl")) == rows
