"""Select one camera's original images with traceable quality decisions."""

import json
import math
import shutil

import cv2
import numpy as np

from .dataset import jsonl, safe_path, sha256_file, write_json
from .validate import validate


def colmap_camera(camera):
    k = np.asarray(camera["K"], dtype=float)
    distortion = list(camera["distortion"])
    if not np.isfinite(distortion).all():
        raise ValueError("Nonfinite distortion coefficients")
    base = [float(k[0, 0]), float(k[1, 1]), float(k[0, 2]), float(k[1, 2])]
    if camera["model"].endswith("Perspective"):
        if len(distortion) < 4 or any(abs(x) > 1e-12 for x in distortion[8:]):
            raise ValueError("Unsupported distortion: thin prism/tilt requires a separate conversion")
        distortion = (distortion + [0.0] * 8)[:8]
        return {"model": "FULL_OPENCV", "params": base + distortion}
    if camera["model"].endswith("Fisheye") and len(distortion) == 4:
        return {"model": "OPENCV_FISHEYE", "params": base + distortion}
    raise ValueError(f"Unsupported camera model {camera['model']}")


def export(root, output, stream="rgb", interval=1.0, min_sharpness=0.0, allow_gaps=False):
    if not math.isfinite(interval) or interval < 0 or not math.isfinite(min_sharpness) or min_sharpness < 0:
        raise ValueError("interval and min_sharpness must be finite and nonnegative")
    report = validate(root)
    if not report["valid"]:
        raise ValueError(f"Dataset failed validation: {report['errors']}")
    if stream not in report["streams"]:
        raise ValueError(f"No {stream} stream in session")
    if report["streams"][stream]["sequence_gaps"] and not allow_gaps:
        raise ValueError("Selected stream has gaps; inspect validation and use --allow-gaps explicitly")
    rows = [row for row in jsonl(root / "frames.jsonl") if row["stream"] == stream]
    first = rows[0]
    camera = colmap_camera(first["camera"])
    geometry = (first["width"], first["height"], first["lens_position"], first["camera"])
    if any((row["width"], row["height"], row["lens_position"], row["camera"]) != geometry for row in rows):
        raise ValueError("Camera geometry/focus changed; split into calibration groups before export")
    output.mkdir(parents=True, exist_ok=False)
    (output / "images").mkdir()
    metadata = {
        "schema_version": 1, "status": "building", "stream": stream,
        "source_manifest_sha256": sha256_file(root / "manifest.json"),
        "source_calibration_sha256": sha256_file(root / "calibration.json"),
        "source_type": json.loads((root / "manifest.json").read_text())["source"],
        "camera": {**camera, "width": first["width"], "height": first["height"]},
        "selection": {"interval_seconds": interval, "min_sharpness": min_sharpness,
                      "allow_gaps": allow_gaps},
        "coordinate_frame": "unscaled monocular reconstruction; no CRS or metric scale",
    }
    write_json(output / "project.json", metadata)
    write_json(output / "validation.json", report)
    selected, last = [], None
    try:
        with (output / "selection.jsonl").open("x") as decisions:
            for row in rows:
                source = safe_path(root, row["path"])
                image = cv2.imread(str(source), cv2.IMREAD_GRAYSCALE)
                if image is None:
                    raise ValueError(f"Could not decode {source}")
                score = float(cv2.Laplacian(image, cv2.CV_64F).var())
                spacing_ok = last is None or row["device_ns"] - last >= interval * 1e9
                reason = "selected" if spacing_ok and score >= min_sharpness else (
                    "interval" if not spacing_ok else "sharpness")
                decision = {"source": row["path"], "sequence": row["sequence"],
                            "device_ns": row["device_ns"], "sharpness": score,
                            "dark_fraction": float(np.mean(image <= 3)),
                            "bright_fraction": float(np.mean(image >= 252)), "decision": reason}
                if reason == "selected":
                    destination = output / "images" / source.name
                    shutil.copy2(source, destination)
                    if sha256_file(destination) != row["sha256"]:
                        raise OSError("Source changed or export copy failed checksum")
                    selected.append({"name": source.name, "sha256": row["sha256"]})
                    last = row["device_ns"]
                decisions.write(json.dumps(decision) + "\n")
        if len(selected) < 3:
            raise ValueError("Fewer than three images selected; reduce interval/quality threshold")
        metadata.update(status="ready", images=selected)
        shutil.copy2(root / "calibration.json", output / "source-calibration.json")
        write_json(output / "project.json", metadata)
    except BaseException as error:
        metadata.update(status="failed", error=str(error))
        write_json(output / "project.json", metadata)
        raise
    return metadata

