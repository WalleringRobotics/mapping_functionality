"""Mode 2: auditable, resumable offline processing with immutable stage outputs."""

import fcntl
import json
import time
from pathlib import Path

from . import odm, opensfm, reconstruct
from .dataset import safe_path, sha256_file, write_json
from .export import export
from .model import assess_model
from .operations import provenance
from .process_utils import inventory, termination_signals, verify_inventory
from .validate import validate


class Workflow:
    def __init__(self, root, inputs, resume):
        self.root = root
        if resume:
            self.state = json.loads((root / "workflow.json").read_text())
            if self.state.get("schema_version") != 1 or self.state["inputs"] != inputs:
                raise ValueError("Run inputs/config changed; use a new output directory")
        else:
            root.mkdir(parents=True, exist_ok=False)
            self.state = {"schema_version": 1, "inputs": inputs, "status": "created",
                          "started_utc_ns": time.time_ns(), "provenance": provenance(),
                          "stages": {}}
            self.save()

    def save(self):
        self.state["updated_utc_ns"] = time.time_ns()
        write_json(self.root / "workflow.json", self.state)

    def stage(self, name, action):
        attempts = self.state["stages"].setdefault(name, [])
        if attempts and attempts[-1]["status"] == "complete":
            attempt = attempts[-1]
            path = safe_path(self.root, attempt["path"])
            verify_inventory(path, attempt["artifacts"])
            return path, attempt["result"]
        if attempts and attempts[-1]["status"] == "running":
            attempts[-1]["status"] = "interrupted"
        path = self.root / "stages" / f"{name}-{len(attempts) + 1:03d}"
        path.parent.mkdir(exist_ok=True)
        attempt = {"path": path.relative_to(self.root).as_posix(), "status": "running",
                   "started_utc_ns": time.time_ns()}
        attempts.append(attempt)
        self.save()
        try:
            attempt["result"] = action(path)
            # Seal all nonempty stage files, including images, calibration, models and logs.
            # Empty logs are allowed; critical products are checked by their stage runner.
            files = [p for p in path.rglob("*") if p.is_file() and p.stat().st_size]
            attempt["artifacts"] = inventory(path, files)
            if not attempt["artifacts"]:
                raise ValueError(f"Stage {name} produced no artifacts")
            attempt["status"] = "complete"
        except BaseException as error:
            attempt.update(status="failed", error=str(error))
            raise
        finally:
            attempt["finished_utc_ns"] = time.time_ns()
            self.save()
        return path, attempt["result"]


def process(session, output, config, execute=False, prepare_only=False, resume=False,
            gcp=None, geo=None, vertical_datum=None):
    session, output = session.resolve(), output.resolve()
    if output.is_relative_to(session) or session.is_relative_to(output):
        raise ValueError("Run output must be separate from the immutable source session")
    audit = validate(session)
    if not audit["valid"]:
        raise ValueError(f"Dataset failed validation: {audit['errors']}")
    source = json.loads((session / "manifest.json").read_text())["source"]
    if execute and prepare_only:
        raise ValueError("Choose execute or prepare-only")
    if execute and source == "synthetic":
        raise ValueError("Synthetic IO fixtures cannot be reconstructed; use --prepare-only")
    if config.product == "terrain":
        if not (gcp or geo) or not vertical_datum or not vertical_datum.strip():
            raise ValueError("Terrain needs --gcp or --geo, and an explicit --vertical-datum")
    elif gcp or geo or vertical_datum:
        raise ValueError("GCP/geo inputs are supported by the terrain recipe only")
    inputs = {"session": str(session), "manifest_sha256": sha256_file(session / "manifest.json"),
              "config": config.to_dict(), "vertical_datum": vertical_datum,
              "gcp_sha256": sha256_file(gcp) if gcp else None,
              "geo_sha256": sha256_file(geo) if geo else None}
    stages = ["export"]
    if config.product == "terrain":
        stages += ["terrain-input", "odm"]
    elif config.resolved_backend == "opensfm":
        stages += ["opensfm-input", "opensfm"]
    else:
        stages += ["sparse", "quality"] + (["dense"] if config.dense else [])
        stages += ["mesh"] if config.mesh else []
    if not execute and not prepare_only:
        return {"status": "planned", "inputs": inputs, "output": str(output), "stages": stages,
                "engine": "COLMAP 3.12.x" if config.resolved_backend == "colmap" else config.odm_image,
                "backend": config.resolved_backend,
                "note": "No outputs written; use --prepare-only or --execute"}
    # O_EXCL creation prevents two new runs sharing a directory. An advisory lock
    # serializes explicit resumes before reading or changing the workflow journal.
    if resume:
        if not (output / "workflow.json").is_file():
            raise ValueError("Resume requires an existing workflow.json")
    else:
        output.mkdir(parents=True, exist_ok=False)
    with (output / ".workflow.lock").open("a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise ValueError("Another process owns this run") from error
        if not resume:
            # Directory creation above owns this new run; initialize without recreating it.
            state = {"schema_version": 1, "inputs": inputs, "status": "created",
                     "started_utc_ns": time.time_ns(), "provenance": provenance(), "stages": {}}
            write_json(output / "workflow.json", state)
        workflow = Workflow(output, inputs, True)
        workflow.state.update(status="running", error=None)
        workflow.save()
        try:
            with termination_signals():
                project, _ = workflow.stage("export", lambda p: export(
                    session, p, config.stream, config.interval_seconds, config.min_sharpness,
                    config.allow_gaps))
                products = []
                quality = None
                if config.product == "terrain":
                    prepared, _ = workflow.stage("terrain-input", lambda p: odm.prepare(
                        project, p, gcp, geo, vertical_datum))
                    if execute:
                        product_root, result = workflow.stage("odm", lambda p: odm.execute(prepared, p, config))
                        products = [product_root / item["path"] for item in result["products"]]
                        quality = result["quality"]
                elif config.resolved_backend == "opensfm":
                    prepared, _ = workflow.stage("opensfm-input", lambda p: opensfm.prepare(
                        project, p, config))
                    if execute:
                        product_root, result = workflow.stage("opensfm", lambda p: opensfm.execute(
                            prepared, p, config))
                        products = [product_root / item["path"] for item in result["products"]]
                        quality = result["quality"]
                elif execute:
                    sparse, _ = workflow.stage("sparse", lambda p: reconstruct.execute(
                        reconstruct.sparse_plan(project, p, config.matcher, config.cpu)))
                    quality_root, quality = workflow.stage("quality", lambda p: assess_model(
                        project, sparse / "sparse", config, p))
                    selected = Path(quality["model"])
                    products = [quality_root / "camera-poses.csv", quality_root / "quality.json"]
                    products += [p for p in selected.iterdir() if p.is_file()]
                    if config.dense:
                        dense, _ = workflow.stage("dense", lambda p: reconstruct.execute(
                            reconstruct.dense_plan(project, selected, p, config.max_image_size)))
                        cloud = dense / "dense/fused.ply"
                        products.append(cloud)
                        if config.mesh:
                            mesh, _ = workflow.stage("mesh", lambda p: reconstruct.execute(
                                reconstruct.mesh_plan(cloud, p)))
                            products.append(mesh / "mesh.ply")
                workflow.state.update(status="complete" if execute else "prepared",
                                      products=inventory(output, products) if products else workflow.state.get("products", []),
                                      quality=quality or workflow.state.get("quality"),
                                      accuracy_claim="Not verified; assess independent checkpoints",
                                      coordinate_frame=f"Unscaled {config.resolved_backend.upper()} world" if config.product == "building"
                                      else "External reference; height convention supplied by operator")
                if execute:
                    workflow.state["completed_utc_ns"] = time.time_ns()
                workflow.save()
                write_json(output / "report.json", {key: value for key, value in workflow.state.items()
                                                    if key != "stages"})
        except BaseException as error:
            workflow.state.update(status="cancelled" if isinstance(error, (KeyboardInterrupt, InterruptedError))
                                  else "failed", error=str(error))
            workflow.save()
            raise
    return workflow.state
