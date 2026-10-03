"""Reviewable COLMAP 3.12 command plans with isolated outputs and execution logs."""

import json
import os
import re
import subprocess
import time
from pathlib import Path

from .dataset import safe_path, sha256_file, write_json
from .process_utils import run_logged, termination_signals


def load_project(project):
    metadata = json.loads((project / "project.json").read_text())
    if metadata.get("schema_version") != 1 or metadata.get("status") != "ready":
        raise ValueError("Export project is not ready")
    expected = set()
    for item in metadata["images"]:
        image = safe_path(project / "images", item["name"])
        expected.add(image)
        if sha256_file(image) != item["sha256"]:
            raise ValueError(f"Export image changed: {item['name']}")
    if set(p.resolve() for p in (project / "images").rglob("*") if p.is_file()) != expected:
        raise ValueError("Image directory differs from the export manifest")
    return metadata


def sparse_plan(project, output, matcher="exhaustive", cpu=False, executable="colmap"):
    project, output = project.resolve(), output.resolve()
    metadata = load_project(project)
    if output.is_relative_to(project):
        raise ValueError("Reconstruction output must be outside the immutable export project")
    if matcher not in {"exhaustive", "sequential"}:
        raise ValueError("Unknown matcher")
    camera = metadata["camera"]
    images, database = str(project / "images"), str(output / "database.db")
    params = ",".join(format(value, ".17g") for value in camera["params"])
    return {
        "engine": "COLMAP 3.12.x", "kind": "sparse", "source": str(project),
        "source_project_sha256": sha256_file(project / "project.json"),
        "source_type": metadata["source_type"], "output": str(output),
        "commands": [
            [executable, "feature_extractor", "--database_path", database,
             "--image_path", images, "--ImageReader.single_camera", "1",
             "--ImageReader.camera_model", camera["model"], "--ImageReader.camera_params", params,
             "--SiftExtraction.use_gpu", "0" if cpu else "1"],
            [executable, matcher + "_matcher", "--database_path", database,
             "--SiftMatching.use_gpu", "0" if cpu else "1"],
            [executable, "mapper", "--database_path", database, "--image_path", images,
             "--output_path", str(output / "sparse"),
             "--Mapper.ba_refine_focal_length", "1", "--Mapper.ba_refine_principal_point", "0",
             "--Mapper.ba_refine_extra_params", "0"],
        ],
        "notes": ["Factory lens distortion is fixed; focal length may refine.",
                  "Model coordinates have arbitrary scale/origin until externally constrained.",
                  "Sequential matching alone does not ensure loop closure; exhaustive is the small-survey default.",
                  "Review all sparse components; dense processing requires explicit model selection."],
    }


def dense_plan(project, model, output, max_image_size=2000, executable="colmap"):
    project, model, output = project.resolve(), model.resolve(), output.resolve()
    metadata = load_project(project)
    if output.is_relative_to(project) or output.is_relative_to(model):
        raise ValueError("Dense output must be outside the source project/model")
    if max_image_size < 128:
        raise ValueError("max_image_size must be at least 128")
    model_files = [model / name for name in ("cameras.bin", "images.bin", "points3D.bin")]
    if not all(file.is_file() for file in model_files):
        raise ValueError("Select an existing binary COLMAP model directory (e.g. sparse/0)")
    return {
        "engine": "COLMAP 3.12.x (CUDA)", "kind": "dense", "source": str(project),
        "source_type": metadata["source_type"],
        "source_project_sha256": sha256_file(project / "project.json"),
        "model": str(model), "model_sha256": {file.name: sha256_file(file) for file in model.iterdir() if file.is_file()},
        "output": str(output),
        "commands": [
            [executable, "model_analyzer", "--path", str(model)],
            [executable, "image_undistorter", "--image_path", str(project / "images"),
             "--input_path", str(model), "--output_path", str(output / "dense"),
             "--output_type", "COLMAP", "--max_image_size", str(max_image_size)],
            [executable, "patch_match_stereo", "--workspace_path", str(output / "dense"),
             "--workspace_format", "COLMAP", "--PatchMatchStereo.geom_consistency", "true"],
            [executable, "stereo_fusion", "--workspace_path", str(output / "dense"),
             "--workspace_format", "COLMAP", "--input_type", "geometric",
             "--output_path", str(output / "dense/fused.ply")],
        ],
    }


def execute(plan):
    executable = plan["commands"][0][0]
    environment = {**os.environ, "QT_QPA_PLATFORM": "offscreen"}
    version = subprocess.run([executable, "-h"], capture_output=True, text=True, env=environment, check=True)
    help_text = version.stdout + version.stderr
    if not re.search(r"COLMAP\s+3\.12(?:\.|\s)", help_text):
        raise RuntimeError("This runner targets COLMAP 3.12.x; verify newer CLI options before upgrading")
    if plan.get("source_type") == "synthetic":
        raise ValueError("Synthetic IO fixtures are not reconstructable scenes; use real captured images")
    output = Path(plan["output"])
    output.mkdir(parents=True, exist_ok=False)
    (output / "logs").mkdir()
    if plan["kind"] == "sparse":
        (output / "sparse").mkdir()
    elif plan["kind"] == "dense":
        (output / "dense").mkdir()
    state = {**plan, "status": "running", "started_utc_ns": time.time_ns(),
             "engine_help": help_text, "completed_commands": 0}
    write_json(output / "run.json", state)
    try:
        for index, command in enumerate(plan["commands"]):
            with termination_signals():
                run_logged(command, output / "logs" / f"{index:02d}-{command[1]}.log", environment)
            state["completed_commands"] += 1
            write_json(output / "run.json", state)
        if plan["kind"] == "sparse":
            components = sorted(str(p.parent.relative_to(output)) for p in (output / "sparse").glob("*/images.bin"))
            if not components:
                raise RuntimeError("COLMAP produced no registered sparse model")
            state["components"] = components
        else:
            product = output / ("mesh.ply" if plan["kind"] == "mesh" else "dense/fused.ply")
            if not product.is_file() or product.stat().st_size == 0:
                raise RuntimeError("COLMAP produced no nonempty output product")
        state["status"] = "complete"
    except BaseException as error:
        state.update(status="failed", error=str(error))
        raise
    finally:
        state["finished_utc_ns"] = time.time_ns()
        write_json(output / "run.json", state)
    return state



def mesh_plan(cloud, output, executable="colmap"):
    cloud, output = cloud.resolve(), output.resolve()
    if not cloud.is_file() or cloud.stat().st_size == 0:
        raise ValueError("Meshing requires a nonempty fused point cloud")
    return {"engine": "COLMAP 3.12.x", "kind": "mesh", "output": str(output),
            "source": str(cloud), "source_sha256": sha256_file(cloud),
            "commands": [[executable, "poisson_mesher", "--input_path", str(cloud),
                          "--output_path", str(output / "mesh.ply")]]}
