"""Sparse reconstruction quality and portable camera-pose exports."""

import csv

import numpy as np

from .dataset import write_json
from .reconstruct import load_project


def pycolmap():
    import pycolmap as engine
    if engine.__version__ != "3.12.6":
        raise RuntimeError("Install .[processing] for pycolmap 3.12.6")
    return engine


def assess_model(project, sparse_root, config, output):
    engine = pycolmap()
    metadata = load_project(project)
    expected = {item["name"] for item in metadata["images"]}
    components = sorted(p for p in sparse_root.iterdir() if p.is_dir() and (p / "images.bin").is_file())
    if not components:
        raise ValueError("No sparse reconstruction components")
    if config.model_index is None:
        if len(components) != 1:
            raise ValueError("Multiple sparse components; inspect and explicitly set model_index")
        selected = components[0]
    else:
        selected = sparse_root / str(config.model_index)
        if selected not in components:
            raise ValueError("Requested model_index does not exist")
    reconstruction = engine.Reconstruction(str(selected))
    names = {image.name for image in reconstruction.images.values()}
    if not names <= expected:
        raise ValueError("Sparse model references images outside this export")
    errors = np.array([point.error for point in reconstruction.points3D.values()])
    tracks = np.array([point.track.length() for point in reconstruction.points3D.values()])
    fraction = len(names) / len(expected)
    failures = []
    if fraction < config.min_registered_fraction:
        failures.append(f"Registered fraction {fraction:.3f} < {config.min_registered_fraction}")
    if len(errors) < config.min_sparse_points:
        failures.append(f"Sparse points {len(errors)} < {config.min_sparse_points}")
    if len(errors) and (not np.isfinite(errors).all() or (errors < 0).any()):
        failures.append("Sparse point errors are invalid")
    result = {
        "model": str(selected), "components": [p.name for p in components],
        "registered_images": len(names), "selected_images": len(expected),
        "registered_fraction": fraction, "unregistered": sorted(expected - names),
        "sparse_points": len(errors), "passed": not failures, "failures": failures,
        "reprojection_error_px": {"median": float(np.median(errors)),
                                  "p95": float(np.percentile(errors, 95))} if len(errors) else {},
        "track_length_median": float(np.median(tracks)) if len(tracks) else None,
        "coordinate_frame": "COLMAP world; arbitrary scale until independently aligned",
        "accuracy_claim": "Registration quality is not metric survey accuracy",
    }
    output.mkdir(parents=True, exist_ok=False)
    write_json(output / "quality.json", result)
    with (output / "camera-poses.csv").open("x", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["image", "camera_id", "center_x", "center_y", "center_z",
                         "q_w", "q_x", "q_y", "q_z", "t_x", "t_y", "t_z"])
        for image in sorted(reconstruction.images.values(), key=lambda item: item.name):
            pose = image.cam_from_world()
            q = pose.rotation.quat
            center = -(pose.rotation.matrix().T @ pose.translation)
            writer.writerow([image.name, image.camera_id, *center, q[3], *q[:3], *pose.translation])
    if failures:
        raise ValueError("Sparse quality gate failed: " + "; ".join(failures))
    return result

