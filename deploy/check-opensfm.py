#!/usr/bin/env python3
"""Exercise the real adapter on three pinned upstream Berlin photographs.

This is software acceptance, not an OAK capture or independent survey test.
"""

import argparse
import fcntl
import json
import os
from pathlib import Path

import cv2

from wallering_mapping import opensfm
from wallering_mapping.dataset import sha256_file, write_json
from wallering_mapping.process import Workflow
from wallering_mapping.process_config import ProcessConfig
from wallering_mapping.process_utils import inventory, run_logged, termination_signals


SAMPLE_SHA256 = {
    "images/01.jpg": "4b3fbfb5d2bda883f7e971535fb71ecb7b45c8fcda35606c6d856e1fb923ffce",
    "images/02.jpg": "d21b2fd5a3b41b9f244a6317400a8e4a44609dd318213598c2e894f9ead9141a",
    "images/03.jpg": "afec918e18fcf5ec6a66c5e3b2185117d3a04b583c31e8a1371f195a38b45501",
    "reconstruction_example.json": "7dfc2f48ffad36092e0f0b21d1e7275b8cf047a3183d874c460a52fa79469425",
}


def sample_project(output, engine):
    output.mkdir(parents=True, exist_ok=False)
    if "," in str(output):
        raise ValueError("Docker bind path must not contain commas")
    script = (
        "import shutil; from pathlib import Path; "
        f"src=Path({opensfm.ENGINE_ROOT!r})/'data/berlin'; dst=Path('/datasets'); "
        "(dst/'images').mkdir(); "
        f"[shutil.copy2(src/p,dst/p) for p in {list(SAMPLE_SHA256)!r}]"
    )
    command = ["docker", "run", "--rm", "--network", "none", "--user",
               f"{os.getuid()}:{os.getgid()}", "--mount",
               f"type=bind,src={output},dst=/datasets", "--entrypoint", opensfm.ENGINE_PYTHON,
               engine["id"], "-c", script]
    run_logged(command, output / "sample-copy.log")
    for name, expected in SAMPLE_SHA256.items():
        if sha256_file(output / name) != expected:
            raise ValueError(f"Upstream Berlin sample changed: {name}")
    reference = json.loads((output / "reconstruction_example.json").read_text())[0]
    if len(reference["cameras"]) != 1:
        raise ValueError("Expected the pinned sample's single camera")
    camera_id, camera = next(iter(reference["cameras"].items()))
    if camera["projection_type"] != "perspective":
        raise ValueError("Expected the pinned sample's perspective model")
    width, height = camera["width"], camera["height"]
    focal = camera["focal"] * max(width, height)
    converted = {
        "model": "FULL_OPENCV", "width": width, "height": height,
        "params": [focal, focal, (width - 1) / 2, (height - 1) / 2,
                   camera["k1"], camera["k2"], 0, 0, 0, 0, 0, 0],
    }
    images = []
    with (output / "selection.jsonl").open("x") as decisions:
        for name in sorted(SAMPLE_SHA256):
            if not name.startswith("images/"):
                continue
            image = cv2.imread(str(output / name))
            if image is None or image.shape[:2] != (height, width):
                raise ValueError(f"Upstream sample dimensions changed: {name}")
            image_name = Path(name).name
            if reference["shots"][image_name]["camera"] != camera_id:
                raise ValueError("Upstream sample uses more than one camera")
            images.append({"name": image_name, "sha256": SAMPLE_SHA256[name]})
            decisions.write(json.dumps({"source": name, "sha256": SAMPLE_SHA256[name],
                                        "decision": "selected", "reason": "all upstream sample photos",
                                        "timestamp_source": "not required for this software test"}) + "\n")
    provenance = {
        "source": "OpenDroneMap/OpenSfM data/berlin",
        "revision": opensfm.OPENSFM_REVISION,
        "source_url": "https://github.com/OpenDroneMap/OpenSfM/tree/" + opensfm.OPENSFM_REVISION + "/data/berlin",
        "files_sha256": SAMPLE_SHA256,
        "camera_source": "Upstream example reconstruction estimated intrinsics; not measured OAK calibration",
        "supplied_camera": {camera_id: camera},
        "converted_camera": converted,
        "pose_policy": "Reference poses/points/GPS are not provided to reconstruction",
        "qualification": "Software smoke test only; no physical survey or metric accuracy evidence",
    }
    write_json(output / "source-calibration.json", provenance)
    write_json(output / "project.json", {
        "schema_version": 1, "status": "ready", "source_type": "upstream-berlin-photos",
        "camera": converted, "images": images, "source_provenance": provenance,
        "coordinate_frame": "Unscaled monocular; no measured geographic reference",
    })
    return provenance


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--image", default="opendronemap/odm:3.6.2")
    parser.add_argument("--execute", action="store_true", help="Run upstream sparse and dense engines")
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--sparse-only", action="store_true")
    args = parser.parse_args()
    config = ProcessConfig(backend="opensfm", mesh=False, dense=not args.sparse_only,
                           max_image_size=1024, max_concurrency=1, interval_seconds=0,
                           odm_image=args.image)
    root = args.output.resolve()
    root.mkdir(parents=True, exist_ok=True)
    with (root / ".sample.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        engine = opensfm.resolve_engine(config.odm_image)
        workflow = Workflow(root / "workflow", {"engine": engine, "config": config.to_dict(),
                            "sample_sha256": SAMPLE_SHA256}, args.resume)
        try:
            with termination_signals():
                project, _ = workflow.stage("sample", lambda p: sample_project(p, engine))
                prepared, _ = workflow.stage("opensfm-input", lambda p: opensfm.prepare(project, p, config))
                result = None
                if args.execute:
                    product_root, result = workflow.stage("opensfm", lambda p: opensfm.execute(prepared, p, config))
                    workflow.state["products"] = inventory(workflow.root, [
                        product_root / item["path"] for item in result["products"]])
                    workflow.state["quality"] = result["quality"]
                workflow.state.update(status="complete" if args.execute else "prepared",
                                      qualification="Upstream sample software test; hardware/survey accuracy unverified")
                workflow.save()
                write_json(root / "report.json", workflow.state)
                print(json.dumps({"status": workflow.state["status"], "report": str(root / "report.json"),
                                  "quality": result["quality"] if result else None}, indent=2))
        except BaseException as error:
            workflow.state.update(status="failed", error=str(error))
            workflow.save()
            raise


if __name__ == "__main__":
    main()
