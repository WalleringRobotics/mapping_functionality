"""Withheld-checkpoint map assessment with explicit reference uncertainty."""

import csv
import html
import json

import numpy as np

from .dataset import safe_path, sha256_file, write_json
from .gnss_accuracy import covariance, finite, projected_crs
from .process_utils import verify_inventory


MAP_PROFILE = {
    "schema_version": 1,
    "crs": None,
    "vertical_datum": None,
    "reference_survey_evidence": None,
    "independence_evidence": None,
    "map_product_evidence": None,
    "control_ids": [],
    "shared_base_with_map": True,
    "shared_base_covariance_map_xyz_m2": None,
    "checkpoint_sigmas_exclude_shared_base": True,
    "minimum_checkpoints": 30,
    "horizontal_target_rmse_m": 0.1,
    "vertical_target_rmse_m": 0.15,
    "map_extent_xy_m": None,
}


def statistics(residuals):
    h = np.linalg.norm(residuals[:, :2], axis=1)
    v = np.abs(residuals[:, 2])
    xyz = np.linalg.norm(residuals, axis=1)
    bias = residuals.mean(axis=0)
    centred = residuals - bias
    return {
        "points": len(residuals),
        "bias_xyz_m": bias.tolist(),
        "rmse_axes_m": np.sqrt(np.mean(residuals**2, axis=0)).tolist(),
        "centred_rmse_axes_m": np.sqrt(np.mean(centred**2, axis=0)).tolist(),
        "rmse_horizontal_m": float(np.sqrt(np.mean(h * h))),
        "rmse_vertical_m": float(np.sqrt(np.mean(v * v))),
        "rmse_3d_m": float(np.sqrt(np.mean(xyz * xyz))),
        "empirical_p95_horizontal_m": float(np.percentile(h, 95)),
        "empirical_p95_absolute_vertical_m": float(np.percentile(v, 95)),
        "empirical_p95_3d_m": float(np.percentile(xyz, 95)),
        "max_horizontal_m": float(h.max()),
        "max_absolute_vertical_m": float(v.max()),
    }


def workflow_evidence(root):
    workflow = json.loads((root / "workflow.json").read_text())
    if workflow["status"] != "complete":
        raise ValueError("Accuracy assessment requires a completed reconstruction workflow")
    controls = set()
    for attempts in workflow["stages"].values():
        if not attempts or attempts[-1]["status"] != "complete":
            raise ValueError("Workflow contains an incomplete stage")
        attempt = attempts[-1]
        path = safe_path(root, attempt["path"])
        verify_inventory(path, attempt["artifacts"])
        gcp = path / "site/gcp_list.txt"
        if gcp.exists():
            controls.update(
                line.split()[-1] for line in gcp.read_text().splitlines()[1:] if line.strip()
            )
    verify_inventory(root, workflow.get("products", []))
    if not workflow.get("products"):
        raise ValueError("Completed workflow has no sealed map/model products")
    return workflow, controls


def assess(checks_path, profile_path, output, image_accuracy=None, workflow=None):
    profile = json.loads(profile_path.read_text())
    if set(profile) != set(MAP_PROFILE) or profile["schema_version"] != 1:
        raise ValueError("Unsupported map accuracy profile")
    crs = projected_crs(profile["crs"])
    for name in (
        "vertical_datum",
        "reference_survey_evidence",
        "independence_evidence",
        "map_product_evidence",
    ):
        if not isinstance(profile[name], str) or not profile[name].strip():
            raise ValueError(
                f"Provide {name}; input coordinates must already share the stated datum"
            )
    if (
        type(profile["minimum_checkpoints"]) is not int
        or profile["minimum_checkpoints"] < 3
        or type(profile["shared_base_with_map"]) is not bool
        or type(profile["checkpoint_sigmas_exclude_shared_base"]) is not bool
        or not isinstance(profile["control_ids"], list)
        or any(not isinstance(v, str) or not v.strip() for v in profile["control_ids"])
    ):
        raise ValueError("Invalid checkpoint count, base-sharing declaration or control IDs")
    if profile["shared_base_with_map"]:
        if (
            not profile["checkpoint_sigmas_exclude_shared_base"]
            or profile["shared_base_covariance_map_xyz_m2"] is None
        ):
            raise ValueError(
                "Same-base checkpoints require a separate shared-base covariance, excluded from point sigmas"
            )
        shared = covariance(profile["shared_base_covariance_map_xyz_m2"], "shared base covariance")
    else:
        if profile["shared_base_covariance_map_xyz_m2"] is not None:
            raise ValueError(
                "Do not add shared-base uncertainty to external checkpoints which already test map/base bias"
            )
        shared = np.zeros((3, 3))
    htarget = finite(profile["horizontal_target_rmse_m"], "horizontal target", positive=True)
    vtarget = finite(profile["vertical_target_rmse_m"], "vertical target", positive=True)
    output = output.resolve()
    for source in (
        checks_path.resolve(),
        profile_path.resolve(),
        image_accuracy.resolve() if image_accuracy else None,
        workflow.resolve() if workflow else None,
    ):
        if source and (output.is_relative_to(source) or source.is_relative_to(output)):
            raise ValueError("Write map accuracy results outside all immutable inputs")
    controls = set(profile["control_ids"])
    workflow_data = None
    if workflow:
        workflow_data, used = workflow_evidence(workflow)
        controls |= used
    with checks_path.open(newline="") as file:
        rows = list(csv.DictReader(file))
    if (
        len(rows) < 3
        or len({r["id"] for r in rows}) != len(rows)
        or any(not r["id"].strip() for r in rows)
    ):
        raise ValueError("At least three uniquely identified withheld checkpoints required")
    if any(r["role"] != "check" for r in rows) or controls & {r["id"] for r in rows}:
        raise ValueError(
            "Checkpoint was used as fitted control; only independent withheld points are allowed"
        )
    reference = np.array([[float(r[f"reference_{a}_m"]) for a in "xyz"] for r in rows])
    model = np.array([[float(r[f"model_{a}_m"]) for a in "xyz"] for r in rows])
    sigmas = np.array([[float(r[f"reference_sigma_{a}_m"]) for a in "xyz"] for r in rows])
    if (
        not np.isfinite(reference).all()
        or not np.isfinite(model).all()
        or not np.isfinite(sigmas).all()
        or (sigmas <= 0).any()
    ):
        raise ValueError("Checkpoint coordinates must be finite and reference axis sigmas positive")
    if np.linalg.matrix_rank(reference[:, :2] - reference[:, :2].mean(axis=0)) < 2:
        raise ValueError("Checkpoint layout is collinear; distributed coverage is required")
    residuals = model - reference
    measured = statistics(residuals)
    reference_variance = np.mean(sigmas**2, axis=0)
    envelope_axes = np.sqrt(np.mean(residuals**2, axis=0) + reference_variance + np.diag(shared))
    indicative_h, indicative_v = float(np.linalg.norm(envelope_axes[:2])), float(envelope_axes[2])
    groups = {}
    for name in sorted({r.get("surface_class", "unspecified") or "unspecified" for r in rows}):
        group = residuals[
            [
                i
                for i, r in enumerate(rows)
                if (r.get("surface_class", "unspecified") or "unspecified") == name
            ]
        ]
        groups[name] = statistics(group)
    images = None
    if image_accuracy:
        images = json.loads((image_accuracy / "report.json").read_text())
        if images["status"] != "complete":
            raise ValueError("Image accuracy report is incomplete")
        for name, expected in images["output_hashes"].items():
            if sha256_file(safe_path(image_accuracy, name)) != expected:
                raise ValueError("Image accuracy output integrity mismatch")
        if (
            workflow_data
            and workflow_data["inputs"]["manifest_sha256"] != images["source_manifest_sha256"]
        ):
            raise ValueError("Image accuracy and reconstruction refer to different captures")
        if str(projected_crs(images["profile"]["policy"]["output_crs"])) != str(crs):
            raise ValueError("Image geolocation and map assessment CRS differ")
        if images["vertical_datum"] != profile["vertical_datum"]:
            raise ValueError("Image geolocation and checkpoint vertical datums differ")
    xy_min, xy_max = reference[:, :2].min(axis=0), reference[:, :2].max(axis=0)
    coverage = {
        "checkpoint_extent_min_xy_m": xy_min.tolist(),
        "checkpoint_extent_max_xy_m": xy_max.tolist(),
        "note": "Extent and point count do not establish spatial representativeness or errors between checkpoints",
    }
    if profile["map_extent_xy_m"] is not None:
        extent = np.array(profile["map_extent_xy_m"], float)
        if (
            extent.shape != (2, 2)
            or not np.isfinite(extent).all()
            or (extent[1] <= extent[0]).any()
        ):
            raise ValueError("map_extent_xy_m must contain [minimum XY, maximum XY]")
        if (reference[:, :2] < extent[0]).any() or (reference[:, :2] > extent[1]).any():
            raise ValueError("Checkpoint lies outside declared map extent")
        coverage["extent_span_fraction_xy"] = ((xy_max - xy_min) / (extent[1] - extent[0])).tolist()
    enough = len(rows) >= profile["minimum_checkpoints"]
    report = {
        "schema_version": 1,
        "status": "complete",
        "profile": profile,
        "checks_sha256": sha256_file(checks_path),
        "profile_sha256": sha256_file(profile_path),
        "workflow_sha256": sha256_file(workflow / "workflow.json") if workflow else None,
        "image_report_sha256": sha256_file(image_accuracy / "report.json")
        if image_accuracy
        else None,
        "measured_checkpoint_comparison": measured,
        "surface_classes": groups,
        "coverage": coverage,
        "reference_axis_sigma_rms_m": np.sqrt(reference_variance).tolist(),
        "shared_base_covariance_map_xyz_m2": shared.tolist(),
        "indicative_reference_aware_rmse_envelope": {
            "axes_m": envelope_axes.tolist(),
            "horizontal_m": indicative_h,
            "vertical_m": indicative_v,
            "meaning": "RSS of observed residual RMS, stated checkpoint sigma RMS and separately shared base; conservative engineering budget, not a confidence bound",
        },
        "sample_size": {
            "actual": len(rows),
            "required_by_profile": profile["minimum_checkpoints"],
            "sufficient": enough,
        },
        "observed_target_passed": measured["rmse_horizontal_m"] <= htarget
        and measured["rmse_vertical_m"] <= vtarget,
        "reference_aware_target_passed": indicative_h <= htarget and indicative_v <= vtarget,
        "passed": enough and indicative_h <= htarget and indicative_v <= vtarget,
        "accuracy_scope": "Observed withheld-checkpoint comparison in supplied CRS/datum; not a guaranteed map-wide or per-pixel bound",
        "standards_compliance": "Not assessed; no ASPRS certification statement is generated",
        "percentile_meaning": "Empirical sample percentiles; not confidence intervals on true map accuracy",
        "same_base_limit": "Common base bias cancels in same-base comparisons; it is retained separately and never divided by sqrt(number of images/checkpoints)",
        "alignment": "No transformation fitted using these checkpoints; all supplied coordinates must be pre-aligned",
        "control_ids_checked": sorted(controls),
        "reconstruction_quality": workflow_data.get("quality") if workflow_data else None,
        "image_navigation_summary": {
            k: images.get(k)
            for k in (
                "images",
                "qualified_images",
                "qualified_fraction",
                "horizontal_budget_m",
                "vertical_budget_m",
                "fully_documented_receiver_age",
            )
        }
        if images
        else None,
    }
    output.mkdir(parents=True, exist_ok=False)
    with (output / "checkpoints.csv").open("x", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(
            [
                "id",
                "surface_class",
                "dx_m",
                "dy_m",
                "dz_m",
                "horizontal_error_m",
                "absolute_vertical_error_m",
            ]
        )
        for row, error in zip(rows, residuals, strict=True):
            writer.writerow(
                [
                    row["id"],
                    row.get("surface_class", "unspecified"),
                    *error,
                    np.linalg.norm(error[:2]),
                    abs(error[2]),
                ]
            )

    def number(value):
        return html.escape(f"{value:.4f}")

    cells = "".join(
        "<tr><td>" + html.escape(label) + "</td><td>" + number(value) + "</td></tr>"
        for label, value in (
            ("Observed horizontal RMSE (m)", measured["rmse_horizontal_m"]),
            ("Observed vertical RMSE (m)", measured["rmse_vertical_m"]),
            ("Reference-aware indicative horizontal budget (m)", indicative_h),
            ("Reference-aware indicative vertical budget (m)", indicative_v),
            ("Empirical horizontal p95 (m)", measured["empirical_p95_horizontal_m"]),
            ("Empirical absolute vertical p95 (m)", measured["empirical_p95_absolute_vertical_m"]),
        )
    )
    page = '<!doctype html><meta charset="utf-8"><title>Map accuracy</title><style>body{font:16px system-ui;max-width:1000px;margin:40px auto;padding:0 20px;line-height:1.5}td,th{padding:8px 14px;border-bottom:1px solid #ddd;text-align:left}table{border-collapse:collapse}pre{white-space:pre-wrap}a{color:#1259a5}</style>'
    page += "<h1>Map accuracy assessment</h1><p>" + html.escape(report["accuracy_scope"]) + "</p>"
    page += (
        "<p>Target gate: <strong>"
        + ("passed" if report["passed"] else "not passed")
        + "</strong>. Checkpoints: "
        + str(len(rows))
        + "; profile requires "
        + str(profile["minimum_checkpoints"])
        + ".</p>"
    )
    page += (
        "<table><tr><th>Quantity</th><th>Metres</th></tr>"
        + cells
        + "</table><h2>How to read this</h2>"
    )
    page += (
        "<p>"
        + html.escape(report["same_base_limit"])
        + "</p><p>"
        + html.escape(report["percentile_meaning"])
        + "</p><p>"
        + html.escape(report["standards_compliance"])
        + "</p>"
    )
    page += "<p>Camera-position uncertainty, reconstruction residuals and measured checkpoint errors are different quantities. Their component terms must not be summed again into an invented total accuracy.</p>"
    page += (
        '<p><a href="checkpoints.csv">Individual checkpoint errors</a> · <a href="report.json">Full breakdown and evidence</a></p><h2>Surface classes</h2><pre>'
        + html.escape(json.dumps(groups, indent=2))
        + "</pre>"
    )
    if images:
        page += (
            "<h2>Image navigation evidence</h2><pre>"
            + html.escape(json.dumps(report["image_navigation_summary"], indent=2))
            + "</pre>"
        )
    (output / "report.html").write_text(page)
    report["output_hashes"] = {
        name: sha256_file(output / name) for name in ("checkpoints.csv", "report.html")
    }
    write_json(output / "report.json", report)
    return report
