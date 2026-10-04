"""Align COLMAP and PLY products with qualified camera GNSS priors."""

import csv
import json
import math
import shutil
from pathlib import Path

import numpy as np

from .dataset import jsonl, safe_path, sha256_file, write_json
from .model import pycolmap
from .process_utils import inventory
from .reconstruct import load_project


def similarity(source, target, weights):
    source, target, weights = (
        np.asarray(source, float),
        np.asarray(target, float),
        np.asarray(weights, float),
    )
    if (
        source.shape != target.shape
        or source.ndim != 2
        or source.shape[1] != 3
        or len(source) < 5
        or weights.shape != (len(source),)
        or not np.isfinite(source).all()
        or not np.isfinite(target).all()
        or not np.isfinite(weights).all()
        or (weights <= 0).any()
    ):
        raise ValueError("At least five finite positive-weight camera position pairs are required")
    weights = weights / weights.sum()
    a, b = np.sum(source * weights[:, None], axis=0), np.sum(target * weights[:, None], axis=0)
    centred_source, centred_target = source - a, target - b
    if min(np.linalg.matrix_rank(centred_source), np.linalg.matrix_rank(centred_target)) < 2:
        raise ValueError("Camera prior geometry is collinear; alignment is underconstrained")
    u, s, vt = np.linalg.svd((centred_target * weights[:, None]).T @ centred_source)
    signs = np.array([1, 1, np.linalg.det(u @ vt)])
    rotation = u @ np.diag(signs) @ vt
    scale = float(np.sum(s * signs) / np.sum(weights * np.sum(centred_source**2, axis=1)))
    translation = b - scale * rotation @ a
    if not math.isfinite(scale) or scale <= 0:
        raise ValueError("Invalid metric similarity scale")
    return scale, rotation, translation


PLY_TYPES = {
    "char": "i1",
    "int8": "i1",
    "uchar": "u1",
    "uint8": "u1",
    "short": "i2",
    "int16": "i2",
    "ushort": "u2",
    "uint16": "u2",
    "int": "i4",
    "int32": "i4",
    "uint": "u4",
    "uint32": "u4",
    "float": "f4",
    "float32": "f4",
    "double": "f8",
    "float64": "f8",
}


def transform_ply(source, output, scale, rotation, translation):
    """Stream scalar vertex properties; preserve faces and other trailing elements.

    XYZ becomes float64, while a nearby local origin avoids projected-coordinate
    float32 precision loss. Normals rotate without translation/scale.
    """
    with source.open("rb") as src, output.open("xb") as dst:
        lines = []
        while True:
            line = src.readline()
            if not line or sum(map(len, lines)) + len(line) > 65536:
                raise ValueError("Invalid/oversized PLY header")
            lines.append(line)
            if line.strip() == b"end_header":
                break
        if lines[0].strip() != b"ply":
            raise ValueError("Expected PLY product")
        fmt = None
        current = None
        count = None
        properties = []
        header = []
        first_element = True
        for line in lines:
            words = line.decode("ascii").split()
            if words[:1] == ["format"]:
                if len(words) != 3 or words[2] != "1.0":
                    raise ValueError("Unsupported PLY version")
                fmt = words[1]
            if words[:1] == ["element"]:
                current = words[1]
                if first_element and current != "vertex":
                    raise ValueError("PLY vertices must be the first element")
                first_element = False
                if current == "vertex":
                    if count is not None:
                        raise ValueError("Duplicate PLY vertex element")
                    count = int(words[2])
            if words[:1] == ["property"] and current == "vertex":
                if len(words) != 3 or words[1] not in PLY_TYPES:
                    raise ValueError("Unsupported/list-valued PLY vertex property")
                kind, name = words[1:]
                properties.append((name, kind))
                if name in "xyz" and len(name) == 1:
                    line = f"property double {name}\n".encode()
            header.append(line)
        names = [p[0] for p in properties]
        if (
            count is None
            or count <= 0
            or len(set(names)) != len(names)
            or not {"x", "y", "z"} <= set(names)
        ):
            raise ValueError("PLY requires nonempty unique XYZ vertices")
        normals = {"nx", "ny", "nz"} <= set(names)
        if set(names) & {"nx", "ny", "nz"} and not normals:
            raise ValueError("PLY normal fields are incomplete")
        if normals and any(
            not PLY_TYPES[kind].startswith("f")
            for name, kind in properties
            if name in ("nx", "ny", "nz")
        ):
            raise ValueError("PLY normal properties must be floating point")
        dst.writelines(header)
        if fmt == "ascii":
            indices = [names.index(a) for a in "xyz"]
            normal_indices = [names.index(a) for a in ("nx", "ny", "nz")] if normals else None
            for _ in range(count):
                values = src.readline().decode("ascii").split()
                if len(values) != len(properties):
                    raise ValueError("Truncated/malformed ASCII PLY vertex")
                xyz = np.array([float(values[i]) for i in indices])
                if not np.isfinite(xyz).all():
                    raise ValueError("Nonfinite PLY vertex")
                transformed = scale * rotation @ xyz + translation
                for i, value in zip(indices, transformed, strict=True):
                    values[i] = format(value, ".17g")
                if normals:
                    normal = np.array([float(values[i]) for i in normal_indices])
                    if not np.isfinite(normal).all():
                        raise ValueError("Nonfinite PLY normal")
                    for i, value in zip(normal_indices, rotation @ normal, strict=True):
                        values[i] = format(value, ".9g")
                dst.write((" ".join(values) + "\n").encode())
        elif fmt in ("binary_little_endian", "binary_big_endian"):
            endian = "<" if fmt == "binary_little_endian" else ">"
            original = np.dtype([(name, endian + PLY_TYPES[kind]) for name, kind in properties])
            derived = np.dtype(
                [
                    (name, endian + ("f8" if name in ("x", "y", "z") else PLY_TYPES[kind]))
                    for name, kind in properties
                ]
            )
            remaining = count
            while remaining:
                n = min(remaining, 65536)
                data = src.read(n * original.itemsize)
                if len(data) != n * original.itemsize:
                    raise ValueError("Truncated binary PLY vertices")
                vertices = np.frombuffer(data, dtype=original)
                target = np.empty(n, dtype=derived)
                for name in names:
                    target[name] = vertices[name]
                xyz = np.column_stack([vertices[a] for a in "xyz"])
                if not np.isfinite(xyz).all():
                    raise ValueError("Nonfinite PLY vertex")
                transformed = scale * xyz @ rotation.T + translation
                for index, name in enumerate("xyz"):
                    target[name] = transformed[:, index]
                if normals:
                    normal = np.column_stack([vertices[a] for a in ("nx", "ny", "nz")])
                    if not np.isfinite(normal).all():
                        raise ValueError("Nonfinite PLY normal")
                    normal = normal @ rotation.T
                    for index, name in enumerate(("nx", "ny", "nz")):
                        target[name] = normal[:, index]
                dst.write(target.tobytes())
                remaining -= n
        else:
            raise ValueError("Unsupported PLY encoding")
        shutil.copyfileobj(src, dst, 1024 * 1024)


def georeference(project, model, image_accuracy, output, artifacts=(), max_residual_m=0.15):
    project, model, image_accuracy, output = (
        p.resolve() for p in (project, model, image_accuracy, output)
    )
    if not math.isfinite(max_residual_m) or max_residual_m <= 0:
        raise ValueError("max_residual_m must be finite and positive")
    for source in (project, model, image_accuracy):
        if output.is_relative_to(source) or source.is_relative_to(output):
            raise ValueError("Georeference output must be separate from immutable inputs")
    metadata = load_project(project)
    report = json.loads((image_accuracy / "report.json").read_text())
    if report["status"] != "complete" or not report["passed"]:
        raise ValueError("Qualified image accuracy report required")
    if (
        report["source_manifest_sha256"] != metadata["source_manifest_sha256"]
        or report["stream"] != metadata["stream"]
    ):
        raise ValueError("Image accuracy and camera model refer to different captures/streams")
    for name, expected in report["output_hashes"].items():
        if sha256_file(safe_path(image_accuracy, name)) != expected:
            raise ValueError("Image accuracy output integrity mismatch")
    priors = {
        r["name"]: r
        for r in jsonl(image_accuracy / "images.jsonl")
        if r["qualified"] and r["projected_camera_xyz_m"] is not None
    }
    engine = pycolmap()
    reconstruction = engine.Reconstruction(str(model))
    expected = {r["name"] for r in metadata["images"]}
    if not {i.name for i in reconstruction.images.values()} <= expected:
        raise ValueError("COLMAP model references images outside selected project")
    names, source, target, weights = [], [], [], []
    for image in sorted(reconstruction.images.values(), key=lambda i: i.name):
        if image.name not in priors:
            continue
        prior = priors[image.name]
        pose = image.cam_from_world()
        source.append(-(pose.rotation.matrix().T @ pose.translation))
        target.append(prior["projected_camera_xyz_m"])
        parts = prior["budget"]["components_covariance_enu_m2"]
        relative = sum(np.array(v) for k, v in parts.items() if k != "shared_base")
        weight_sigma = (
            math.sqrt(float(np.trace(relative)))
            + prior["budget"]["deterministic_motion_allowance_m"]
        )
        weights.append(1 / max(weight_sigma, 1e-6) ** 2)
        names.append(image.name)
    target = np.asarray(target)
    if len(target) < 5:
        raise ValueError("At least five registered qualified camera GNSS priors required")
    origin = target.mean(axis=0)
    scale, rotation, translation = similarity(source, target - origin, weights)
    residuals = scale * np.array(source) @ rotation.T + translation - (target - origin)
    errors = np.linalg.norm(residuals, axis=1)
    if errors.max() > max_residual_m:
        raise ValueError(
            "Camera prior alignment residual exceeds limit; inspect time, lever arm and reconstruction"
        )
    artifacts = [Path(p).resolve() for p in artifacts]
    if len({p.name for p in artifacts}) != len(artifacts) or any(
        p.suffix.lower() != ".ply" for p in artifacts
    ):
        raise ValueError("Provide distinct named PLY products from this same COLMAP world")
    model_names = ["cameras.bin", "images.bin", "points3D.bin"]
    model_names += [name for name in ("rigs.bin", "frames.bin") if (model / name).exists()]
    model_sources = {name: sha256_file(model / name) for name in model_names}
    product_sources = [{"path": str(p), "sha256": sha256_file(p)} for p in artifacts]
    output.mkdir(parents=True, exist_ok=False)
    result = {
        "schema_version": 1,
        "status": "running",
        "source_manifest_sha256": metadata["source_manifest_sha256"],
        "image_accuracy_report_sha256": sha256_file(image_accuracy / "report.json"),
        "source_model_hashes": model_sources,
        "source_model_path": str(model),
        "source_artifacts": product_sources,
        "crs": report["profile"]["policy"]["output_crs"],
        "vertical_datum": report["vertical_datum"],
        "coordinate_origin_xyz_m": origin.tolist(),
        "scale": scale,
        "rotation": rotation.tolist(),
        "translation_local_m": translation.tolist(),
        "camera_controls": names,
        "accuracy_claim": "Navigation-prior alignment, not independent map accuracy; assess withheld checkpoints",
        "coordinate_rule": "absolute projected XYZ = stored local XYZ + coordinate_origin_xyz_m",
        "weight_model": "Relative receiver/rig uncertainty plus motion allowance; shared base excluded from relative weights",
        "camera_prior_residual_rmse_m": float(np.sqrt(np.mean(errors**2))),
        "camera_prior_residual_max_m": float(errors.max()),
    }
    write_json(output / "report.json", result)
    try:
        reconstruction.transform(engine.Sim3d(scale, engine.Rotation3d(rotation), translation))
        (output / "model").mkdir()
        reconstruction.write(str(output / "model"))
        if artifacts:
            (output / "products").mkdir()
            for source_product in artifacts:
                transform_ply(
                    source_product,
                    output / "products" / source_product.name,
                    scale,
                    rotation,
                    translation,
                )
        for name, expected_hash in model_sources.items():
            if sha256_file(model / name) != expected_hash:
                raise ValueError("Source model changed during alignment")
        for item in product_sources:
            if sha256_file(Path(item["path"])) != item["sha256"]:
                raise ValueError("Source PLY changed during alignment")
        with (output / "camera-controls.csv").open("x", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(["image", "role", "dx_m", "dy_m", "dz_m", "error_3d_m"])
            for name, error, norm in zip(names, residuals, errors, strict=True):
                writer.writerow([name, "navigation_control", *error, norm])
        result.update(
            status="complete",
            output_artifacts=inventory(
                output, [p for p in output.rglob("*") if p.is_file() and p.name != "report.json"]
            ),
        )
    except BaseException as error:
        result.update(status="failed", error=str(error))
        raise
    finally:
        write_json(output / "report.json", result)
    return result
