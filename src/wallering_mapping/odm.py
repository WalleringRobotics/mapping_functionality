"""Calibrated ODM 3.6.2 staging and managed Docker execution."""

import json
import os
import re
import shutil
import subprocess
import time
import uuid
from pathlib import Path

import cv2
import numpy as np

from .control import gcps, geolocation
from .dataset import atomic_bytes, safe_path, sha256_file, write_json
from .process_utils import inventory, run_logged
from .reconstruct import load_project


def camera_geometry(camera):
    params = camera["params"]
    k = np.array([[params[0], 0, params[2]], [0, params[1], params[3]], [0, 0, 1]], float)
    distortion = np.array(params[4:], float)
    width, height = camera["width"], camera["height"]
    if camera["model"] == "FULL_OPENCV" and len(distortion) == 8:
        maps = cv2.initUndistortRectifyMap(k, distortion, None, k, (width, height), cv2.CV_32FC1)
    elif camera["model"] == "OPENCV_FISHEYE" and len(distortion) == 4:
        maps = cv2.fisheye.initUndistortRectifyMap(k, distortion, np.eye(3), k,
                                                (width, height), cv2.CV_32FC1)
    else:
        raise ValueError("Unsupported camera model for ODM preprocessing")
    x, y = maps
    valid = ((x >= 0) & (x <= width - 1) & (y >= 0) & (y <= height - 1)).astype(np.uint8) * 255
    mask = cv2.erode(valid, np.ones((3, 3), np.uint8), borderType=cv2.BORDER_CONSTANT, borderValue=0)
    if np.mean(mask > 0) < 0.25:
        raise ValueError("Undistortion retains less than 25% valid image area; inspect calibration")
    scale = max(width, height)
    normalized = {"projection_type": "brown", "width": width, "height": height,
                  "focal_x": params[0] / scale, "focal_y": params[1] / scale,
                  "c_x": (params[2] - (width - 1) / 2) / scale,
                  "c_y": (params[3] - (height - 1) / 2) / scale,
                  "k1": 0, "k2": 0, "k3": 0, "p1": 0, "p2": 0}
    return k, distortion, maps, mask, normalized


def undistort_pixel(pixel, camera):
    params = camera["params"]
    k = np.array([[params[0], 0, params[2]], [0, params[1], params[3]], [0, 0, 1]], float)
    points = np.array(pixel, float).reshape(1, 1, 2)
    distortion = np.array(params[4:], float)
    if camera["model"] == "OPENCV_FISHEYE":
        result = cv2.fisheye.undistortPoints(points, k, distortion, P=k)
    else:
        result = cv2.undistortPointsIter(points, k, distortion, None, k,
                                       (cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 50, 1e-10))
    return result.reshape(2)


def prepare(project, output, gcp_path=None, geo_path=None, vertical_datum=None):
    metadata = load_project(project)
    if output.resolve().is_relative_to(project.resolve()):
        raise ValueError("ODM staging must be outside the source project")
    if not gcp_path and not geo_path:
        raise ValueError("Terrain products require real GCPs or image geolocation; no coordinates are invented")
    if not vertical_datum or not vertical_datum.strip():
        raise ValueError("State the height reference with --vertical-datum (e.g. EGM2008 or ellipsoidal WGS84)")
    camera = metadata["camera"]
    names = {item["name"] for item in metadata["images"]}
    controls = gcps(gcp_path, names, camera["width"], camera["height"]) if gcp_path else None
    positions = geolocation(geo_path, names) if geo_path else None
    _, _, maps, mask, normalized = camera_geometry(camera)
    mapping = {name: Path(name).stem + ".png" for name in names}
    if len(set(mapping.values())) != len(mapping):
        raise ValueError("Image filenames collide after PNG conversion")
    # OpenCV writes derived PNGs without EXIF/XMP. ODM 3.6.2 therefore uses the
    # empty make/model, brown projection and 0.85 fallback to form its *identifier*.
    # The actual intrinsics override below replaces that fallback numerically.
    camera_id = f"v2   {camera['width']} {camera['height']} brown 0.85"
    transformed_controls = []
    if controls:
        for row in controls["records"]:
            pixel = undistort_pixel(row["pixel"], camera)
            u, v = pixel
            if (not np.isfinite(pixel).all() or not 0 <= u < camera["width"] - 1
                    or not 0 <= v < camera["height"] - 1 or mask[int(round(v)), int(round(u))] == 0):
                raise ValueError(f"GCP falls outside valid undistorted area: {row['label']}/{row['image']}")
            transformed_controls.append({**row, "pixel": pixel.tolist(), "image": mapping[row["image"]]})
    output.mkdir(parents=True, exist_ok=False)
    images = output / "site/images"
    images.mkdir(parents=True)
    result = {"status": "preparing", "source_project_sha256": sha256_file(project / "project.json"),
              "source_type": metadata["source_type"], "camera_id": camera_id,
              "camera": normalized, "image_mapping": mapping,
              "processing": "OpenCV undistortion into original K and dimensions; lossless derived PNG",
              "valid_pixel_fraction": float(np.mean(mask > 0)),
              "vertical_datum": vertical_datum,
              "control_crs": controls["crs"] if controls else None,
              "geolocation_crs": positions["crs"] if positions else None,
              "reference": "surveyed_controls" if controls else "camera_position_priors",
              "reference_accuracy": "Not verified; assess independent checkpoints"}
    write_json(output / "preparation.json", result)
    try:
        for item in metadata["images"]:
            source = safe_path(project / "images", item["name"])
            image = cv2.imread(str(source), cv2.IMREAD_COLOR)
            transformed = cv2.remap(image, *maps, interpolation=cv2.INTER_LINEAR,
                                    borderMode=cv2.BORDER_CONSTANT)
            name = mapping[item["name"]]
            for path, array in ((images / name, transformed),
                                (images / (Path(name).stem + "_mask.png"), mask)):
                ok, encoded = cv2.imencode(".png", array, [cv2.IMWRITE_PNG_COMPRESSION, 1])
                if not ok:
                    raise OSError(f"PNG encoding failed: {path.name}")
                atomic_bytes(path, encoded.tobytes())
        write_json(output / "site/cameras.json", {camera_id: normalized})
        if controls:
            rows = [controls["crs"]] + [" ".join([*(format(v, ".17g") for v in row["xyz"] + row["pixel"]),
                                                   row["image"], row["label"]]) for row in transformed_controls]
            atomic_bytes(output / "site/gcp_list.txt", ("\n".join(rows) + "\n").encode())
        if positions:
            rows = [positions["crs"]] + [" ".join([mapping[row["image"]],
                                                    *(format(v, ".17g") for v in row["values"])])
                                        for row in positions["records"]]
            atomic_bytes(output / "site/geo.txt", ("\n".join(rows) + "\n").encode())
        result.update(status="ready", images=len(metadata["images"]),
                      gcp_points=controls["points"] if controls else 0)
        write_json(output / "preparation.json", result)
    except BaseException as error:
        result.update(status="failed", error=str(error))
        write_json(output / "preparation.json", result)
        raise
    return result


def resolve_image(image):
    result = subprocess.run(["docker", "image", "inspect", image], capture_output=True,
                            text=True, check=True)
    info = json.loads(result.stdout)[0]
    identity = info["Id"]
    if not re.fullmatch(r"sha256:[0-9a-f]{64}", identity):
        raise ValueError("Docker did not return an immutable image ID")
    version = subprocess.run(["docker", "run", "--rm", "--network", "none", identity, "--version"],
                             capture_output=True, text=True, check=True)
    text = version.stdout + version.stderr
    if not re.search(r"(?<![\d.])3\.6\.2(?![\d.])", text):
        raise ValueError(f"Expected ODM 3.6.2; engine reports {text.strip()}")
    return {"requested": image, "id": identity, "digests": info.get("RepoDigests", []),
            "version_output": text.strip()}


def docker_command(image, output, name, config, end_with=None):
    if "," in str(output):
        raise ValueError("Docker bind path must not contain commas")
    command = ["docker", "run", "--rm", "--name", name, "--network", "none",
               "--user", f"{os.getuid()}:{os.getgid()}", "--env", "HOME=/tmp",
               "--mount", f"type=bind,src={output.resolve()},dst=/datasets", image,
               "--project-path", "/datasets", "--cameras", "/datasets/site/cameras.json",
               "--camera-lens", "brown", "--use-fixed-camera-params",
               "--matcher-type", "flann", "--max-concurrency", str(config.max_concurrency),
               "--dsm", "--orthophoto-resolution", str(config.orthophoto_cm),
               "--dem-resolution", str(config.dem_cm)]
    if (output / "site/gcp_list.txt").exists():
        command += ["--gcp", "/datasets/site/gcp_list.txt"]
    if (output / "site/geo.txt").exists():
        command += ["--geo", "/datasets/site/geo.txt"]
    if config.dtm:
        command += ["--dtm"]
    if not config.mesh:
        command += ["--skip-3dmodel"]
    if end_with:
        command += ["--end-with", end_with]
    return command + ["site"]


def verify_camera(actual, expected):
    if actual.get("projection_type") != "brown":
        raise ValueError("ODM changed the calibrated camera projection")
    for key, value in expected.items():
        if isinstance(value, (float, int)):
            observed = actual.get(key, float("inf"))
            if not isinstance(observed, (int, float)) or not np.isfinite(observed) or abs(observed - value) > 1e-9:
                raise ValueError(f"ODM calibration mismatch: {key}")


def execute(prepared, output, config):
    metadata = json.loads((prepared / "preparation.json").read_text())
    if metadata["status"] != "ready" or metadata["source_type"] == "synthetic":
        raise ValueError("ODM requires a ready preparation from real imagery")
    engine = resolve_image(config.odm_image)
    output.mkdir(parents=True, exist_ok=False)
    shutil.copytree(prepared / "site", output / "site")
    (output / "logs").mkdir()
    state = {"status": "running", "engine": engine, "started_utc_ns": time.time_ns(), "commands": []}
    write_json(output / "run.json", state)
    name = "wr-mapping-" + uuid.uuid4().hex
    try:
        for index, end in enumerate(("dataset", "opensfm", None)):
            command = docker_command(engine["id"], output, name, config, end)
            state["commands"].append(command)
            write_json(output / "run.json", state)
            run_logged(command, output / "logs" / f"{index:02d}-{end or 'products'}.log")
            if end == "dataset":
                photos = json.loads((output / "site/images.json").read_text())
                if {p["filename"] for p in photos} != set(metadata["image_mapping"].values()):
                    raise ValueError("ODM did not ingest exactly the prepared images")
                for p in photos:
                    key = " ".join(["v2", p["camera_make"].strip(), p["camera_model"].strip(),
                                    str(int(p["width"])), str(int(p["height"])),
                                    p["camera_projection"], str(float(p["focal_ratio"]))[:6]]).lower()
                    if key != metadata["camera_id"]:
                        raise ValueError("ODM camera identity changed; calibration override would not apply")
            if end == "opensfm":
                cameras = json.loads((output / "site/opensfm/camera_models.json").read_text())
                if metadata["camera_id"] not in cameras:
                    raise ValueError("ODM did not apply the calibrated camera override")
                actual = cameras[metadata["camera_id"]]
                verify_camera(actual, metadata["camera"])
                reconstructions = json.loads((output / "site/opensfm/reconstruction.json").read_text())
                if len(reconstructions) != 1:
                    raise ValueError("ODM produced multiple/empty sparse components; inspect before dense mapping")
                reconstruction = reconstructions[0]
                if set(reconstruction["cameras"]) != {metadata["camera_id"]}:
                    raise ValueError("ODM reconstruction has unexpected cameras")
                verify_camera(reconstruction["cameras"][metadata["camera_id"]], metadata["camera"])
                if not set(reconstruction["shots"]) <= set(metadata["image_mapping"].values()):
                    raise ValueError("ODM reconstruction references unexpected images")
                fraction = len(reconstruction["shots"]) / metadata["images"]
                if fraction < config.min_registered_fraction or len(reconstruction["points"]) < config.min_sparse_points:
                    raise ValueError("ODM sparse reconstruction failed registration/point-count quality gates")
                state["quality"] = {"registered_fraction": fraction,
                                    "sparse_points": len(reconstruction["points"])}
        required = [output / "site/odm_orthophoto/odm_orthophoto.tif",
                    output / "site/odm_dem/dsm.tif",
                    output / "site/odm_georeferencing/odm_georeferenced_model.laz"]
        if config.dtm:
            required.append(output / "site/odm_dem/dtm.tif")
        if config.mesh:
            required.append(output / "site/odm_texturing/odm_textured_model_geo.obj")
            textures = [p for p in (output / "site/odm_texturing").iterdir()
                        if p.suffix.lower() in {".mtl", ".png", ".jpg", ".jpeg"}]
            if not any(p.suffix.lower() == ".mtl" for p in textures) or not any(
                    p.suffix.lower() in {".png", ".jpg", ".jpeg"} for p in textures):
                raise ValueError("ODM textured mesh is missing materials/textures")
            required += textures
        state["products"] = inventory(output, required)
        state.update(status="complete", input_reference=metadata["reference"],
                     vertical_datum=metadata["vertical_datum"],
                     accuracy_claim="Unverified until independent checkpoint assessment")
    except BaseException as error:
        state.update(status="failed", error=str(error))
        # Only remove the container created with this run's unique name.
        try:
            subprocess.run(["docker", "rm", "-f", name], stdout=subprocess.DEVNULL,
                           stderr=subprocess.DEVNULL, timeout=30)
        except (OSError, subprocess.TimeoutExpired) as cleanup_error:
            state["cleanup_error"] = str(cleanup_error)
        raise
    finally:
        state["finished_utc_ns"] = time.time_ns()
        write_json(output / "run.json", state)
    return state
