"""Native OpenSfM datasets and execution using ODM's pinned upstream engine.

Only calibration conversion, artifact auditing and acceptance gates live here.
Feature detection, matching, SfM, bundle adjustment and dense stereo are upstream.
"""

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

from . import odm
from .dataset import atomic_bytes, safe_path, sha256_file, write_json
from .process_utils import inventory, run_logged, verify_inventory
from .reconstruct import load_project


# ODM v3.6.2/SuperBuild/cmake/External-OpenSfM.cmake pins this source revision.
OPENSFM_REVISION = "c5328439465e6ace011f39077d1077d7b1cdd65d"
ENGINE_ROOT = "/code/SuperBuild/install/bin/opensfm"
ENGINE_PYTHON = "/code/venv/bin/python3"
SOURCE_SHA256 = {
    "bin/opensfm_main.py": "a1d37a7e4f78c115e3074829a9f52b5bae464e2bef92a3629f29de8ec041db32",
    "opensfm/config.py": "41879326a6353cc13b092dcb6b0660911792bc87d7004b022ef23e99bbcdfbf8",
    "opensfm/dataset.py": "b592b1c0a67390680116050a71631837e73830094c057b6fd5832332c27448b0",
    "opensfm/actions/extract_metadata.py": "93b447b87975aee0504e9d7520e5cd157f5ed1db93aab2ba63673fdfacb06172",
    "opensfm/io.py": "d01397e864c0f86c218cc4372e9e755e0b98582a7b84437a70934dc6728f9d80",
}
CAMERA_ID = "wr-calibrated"


def prepare(project, output, config):
    """Prepare upstream inputs; retain source hashes without fabricating GPS/time."""
    metadata = load_project(project)
    if output.resolve().is_relative_to(project.resolve()):
        raise ValueError("OpenSfM staging must be outside the source project")
    _, _, maps, mask, camera = odm.camera_geometry(metadata["camera"])
    mapping = {item["name"]: Path(item["name"]).stem + ".png" for item in metadata["images"]}
    if len(set(mapping.values())) != len(mapping):
        raise ValueError("Image filenames collide after PNG conversion")
    output.mkdir(parents=True, exist_ok=False)
    dataset = output / "dataset"
    (dataset / "images").mkdir(parents=True)
    (dataset / "masks").mkdir()
    state = {
        "schema_version": 1,
        "status": "preparing",
        "source_project_sha256": sha256_file(project / "project.json"),
        "source_type": metadata["source_type"],
        "camera_id": CAMERA_ID,
        "camera": camera,
        "image_mapping": mapping,
        "images": len(mapping),
        "processing": "OpenCV undistortion into original K and dimensions; lossless derived PNG",
        "valid_pixel_fraction": float(np.mean(mask > 0)),
        "coordinate_frame": "Unscaled OpenSfM world; no geographic reference or metric scale",
        "timestamp_policy": "Selection timestamps retained as provenance; no EXIF epoch inferred",
        "config": config.to_dict(),
    }
    write_json(output / "preparation.json", state)
    try:
        for item in metadata["images"]:
            source = safe_path(project / "images", item["name"])
            image = cv2.imread(str(source), cv2.IMREAD_COLOR)
            if image is None:
                raise ValueError(f"Could not decode {source}")
            derived = cv2.remap(image, *maps, interpolation=cv2.INTER_LINEAR,
                                borderMode=cv2.BORDER_CONSTANT)
            name = mapping[item["name"]]
            # Native OpenSfM automatic mask lookup appends .png to the image name.
            for path, array in ((dataset / "images" / name, derived),
                                (dataset / "masks" / (name + ".png"), mask)):
                ok, encoded = cv2.imencode(".png", array, [cv2.IMWRITE_PNG_COMPRESSION, 1])
                if not ok:
                    raise OSError(f"PNG encoding failed: {path.name}")
                atomic_bytes(path, encoded.tobytes())
        write_json(dataset / "camera_models_overrides.json", {CAMERA_ID: camera})
        write_json(dataset / "exif_overrides.json", {
            name: {"camera": CAMERA_ID, "projection_type": "brown", "orientation": 1,
                   "width": camera["width"], "height": camera["height"]}
            for name in mapping.values()
        })
        native_config = {
            "feature_type": "SIFT", "feature_process_size": config.max_image_size,
            "matcher_type": "FLANN", "matching_gps_distance": 0,
            "matching_gps_neighbors": 0, "matching_time_neighbors": 0,
            "matching_order_neighbors": 8 if config.matcher == "sequential" else 0,
            "matching_bow_neighbors": 0, "matching_vlad_neighbors": 0,
            "optimize_camera_parameters": False, "bundle_use_gps": False,
            "bundle_use_gcp": False, "use_altitude_tag": False,
            "align_method": "naive", "processes": config.max_concurrency,
            "read_processes": config.max_concurrency,
            "undistorted_image_format": "png",
            "undistorted_image_max_size": config.max_image_size,
            "depthmap_resolution": min(640, config.max_image_size),
        }
        # JSON is a YAML subset accepted by upstream, without another local dependency.
        write_json(dataset / "config.yaml", native_config)
        for name in ("project.json", "selection.jsonl", "source-calibration.json"):
            shutil.copy2(project / name, output / ("source-" + name))
        state.update(status="ready", artifacts=inventory(
            output, [p for p in output.rglob("*") if p.is_file() and p.name != "preparation.json"]))
        write_json(output / "preparation.json", state)
    except BaseException as error:
        state.update(status="failed", error=str(error))
        write_json(output / "preparation.json", state)
        raise
    return state


def resolve_engine(image):
    """Verify the supported image and the upstream Python interfaces it supplies."""
    engine = odm.resolve_image(image)
    script = (
        "import hashlib,json,platform; from pathlib import Path; "
        "from opensfm import pymap,pygeometry; "
        f"root=Path({ENGINE_ROOT!r}); names={list(SOURCE_SHA256)!r}; "
        "print(json.dumps({'python':platform.python_version(), "
        "'source_sha256':{p:hashlib.sha256((root/p).read_bytes()).hexdigest() for p in names}, "
        "'native_sha256':{m.__name__:hashlib.sha256(Path(m.__file__).read_bytes()).hexdigest() "
        "for m in (pymap,pygeometry)}}))"
    )
    command = ["docker", "run", "--rm", "--network", "none", "--entrypoint", ENGINE_PYTHON,
               engine["id"], "-c", script]
    probe = subprocess.run(command, capture_output=True, text=True, check=True, timeout=60)
    identity = json.loads(probe.stdout)
    if identity["source_sha256"] != SOURCE_SHA256:
        raise ValueError("OpenSfM source differs from the pinned ODM 3.6.2 engine")
    return {**engine, "opensfm_revision": OPENSFM_REVISION, "opensfm": identity}


def docker_command(image, output, name, action, options=()):
    if "," in str(output):
        raise ValueError("Docker bind path must not contain commas")
    return [
        "docker", "run", "--rm", "--name", name, "--network", "none", "--user",
        f"{os.getuid()}:{os.getgid()}", "--env", "HOME=/tmp",
        "--env", "OPENBLAS_NUM_THREADS=1", "--env", "OMP_NUM_THREADS=1",
        "--mount", f"type=bind,src={output.resolve()},dst=/datasets",
        "--entrypoint", ENGINE_PYTHON, image, ENGINE_ROOT + "/bin/opensfm_main.py",
        action, *options, "/datasets/dataset",
    ]


def assess_sparse(dataset, metadata, config):
    """Audit native models and enforce the same registration/point gates as ODM."""
    models = json.loads((dataset / "reconstruction.json").read_text())
    if not isinstance(models, list) or not models:
        raise ValueError("OpenSfM produced no sparse reconstruction")
    if len(models) > 1 and config.model_index is None:
        raise ValueError("Multiple sparse components; inspect and explicitly set model_index")
    index = config.model_index if config.model_index is not None else 0
    if index >= len(models):
        raise ValueError("Requested model_index does not exist")
    model = models[index]
    if set(model["cameras"]) != {metadata["camera_id"]}:
        raise ValueError("OpenSfM reconstruction has unexpected cameras")
    odm.verify_camera(model["cameras"][metadata["camera_id"]], metadata["camera"])
    expected = set(metadata["image_mapping"].values())
    names = set(model["shots"])
    if not names <= expected:
        raise ValueError("OpenSfM reconstruction references unexpected images")
    for shot in model["shots"].values():
        if shot["camera"] != metadata["camera_id"]:
            raise ValueError("OpenSfM shot references an unexpected camera")
        for key in ("rotation", "translation"):
            values = np.asarray(shot[key], dtype=float)
            if values.shape != (3,) or not np.isfinite(values).all():
                raise ValueError("OpenSfM produced invalid camera poses")
    for point in model["points"].values():
        values = np.asarray(point["coordinates"], dtype=float)
        if values.shape != (3,) or not np.isfinite(values).all():
            raise ValueError("OpenSfM produced invalid sparse coordinates")
    fraction = len(names) / len(expected)
    passed = fraction >= config.min_registered_fraction and len(model["points"]) >= config.min_sparse_points
    quality = {
        "passed": passed, "components": len(models), "model_index": index,
        "registered_images": len(names), "selected_images": len(expected),
        "registered_fraction": fraction, "unregistered_images": sorted(expected - names),
        "sparse_points": len(model["points"]), "camera_parameters_fixed": True,
        "accuracy_claim": "Unverified; unscaled visual reconstruction requires independent control",
    }
    write_json(dataset.parent / "quality.json", quality)
    if not passed:
        raise ValueError("OpenSfM sparse reconstruction failed registration/point-count quality gates")
    if len(models) > 1:
        # Upstream export_ply selects component zero. Preserve its complete native
        # result before staging the operator-selected component for export/dense.
        shutil.copy2(dataset / "reconstruction.json", dataset / "reconstruction-all.json")
        write_json(dataset / "reconstruction.json", [model])
    return quality


def require_cloud(path):
    if not path.is_file():
        raise ValueError(f"OpenSfM produced no point cloud: {path.name}")
    with path.open("rb") as stream:
        header = stream.read(8192)
    match = re.search(rb"(?:^|\n)element vertex (\d+)\r?\n", header)
    end = re.search(rb"\nend_header\r?\n", header)
    if (not header.startswith(b"ply\n") or not match or int(match[1]) == 0
            or not end or path.stat().st_size <= end.end()):
        raise ValueError(f"OpenSfM produced an empty/invalid point cloud: {path.name}")


def execute(prepared, output, config):
    metadata = json.loads((prepared / "preparation.json").read_text())
    if metadata["status"] != "ready" or metadata["source_type"] == "synthetic":
        raise ValueError("OpenSfM requires a ready preparation from real imagery")
    if metadata["config"] != config.to_dict():
        raise ValueError("OpenSfM prepared config changed; use a new run")
    verify_inventory(prepared, metadata["artifacts"])
    engine = resolve_engine(config.odm_image)
    output.mkdir(parents=True, exist_ok=False)
    dataset = output / "dataset"
    shutil.copytree(prepared / "dataset", dataset)
    (output / "logs").mkdir()
    state = {"status": "running", "engine": engine, "started_utc_ns": time.time_ns(),
             "commands": [], "completed_commands": 0,
             "preparation_sha256": sha256_file(prepared / "preparation.json")}
    write_json(output / "run.json", state)
    name = "wr-opensfm-" + uuid.uuid4().hex
    actions = ["extract_metadata", "detect_features", "match_features", "create_tracks",
               "reconstruct", "export_ply"]
    if config.dense:
        actions += ["undistort", "compute_depthmaps"]
    try:
        for index, action in enumerate(actions):
            command = docker_command(engine["id"], output, name, action,
                                     ("--no-cameras",) if action == "export_ply" else ())
            state["commands"].append(command)
            write_json(output / "run.json", state)
            run_logged(command, output / "logs" / f"{index:02d}-{action}.log")
            state["completed_commands"] += 1
            if action == "extract_metadata":
                cameras = json.loads((dataset / "camera_models.json").read_text())
                if set(cameras) != {metadata["camera_id"]}:
                    raise ValueError("OpenSfM did not apply the calibrated camera override")
                odm.verify_camera(cameras[metadata["camera_id"]], metadata["camera"])
            if action == "reconstruct":
                state["quality"] = assess_sparse(dataset, metadata, config)
            write_json(output / "run.json", state)
        clouds = [dataset / "reconstruction.ply"]
        if config.dense:
            clouds.append(dataset / "undistorted/depthmaps/merged.ply")
        for cloud in clouds:
            require_cloud(cloud)
        state.update(status="complete", products=inventory(output, [*clouds,
                     dataset / "reconstruction.json", dataset / "tracks.csv",
                     dataset / "camera_models.json", output / "quality.json"]),
                     coordinate_frame=metadata["coordinate_frame"],
                     accuracy_claim="Unverified; assess independently after measured alignment")
    except BaseException as error:
        state.update(status="failed", error=str(error))
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
