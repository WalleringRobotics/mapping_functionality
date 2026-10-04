"""Inspect ODM products with the GDAL/PDAL already installed in its image.

This file also runs directly inside that image; no repository installation or
second geospatial environment is needed there.
"""

import json
import math
import sys
from pathlib import Path


def validate_products(report):
    """Require readable, nonempty products sharing a metric projected CRS."""
    from pyproj import CRS

    if len(report["rasters"]) < 2:
        raise ValueError("ODM needs an orthophoto and DSM raster")
    products = [*report["rasters"], report["cloud"]]
    reference = None
    bounds = []
    for product in products:
        crs = CRS.from_user_input(product["crs_wkt"]).to_2d()
        if not crs.is_projected or any(
            not math.isclose(axis.unit_conversion_factor, 1.0) for axis in crs.axis_info[:2]
        ):
            raise ValueError("ODM products require a projected CRS with metre axes")
        if reference is not None and not reference.equals(crs, ignore_axis_order=True):
            raise ValueError("ODM raster and point-cloud coordinate systems disagree")
        reference = crs
        box = product["bounds_xy"]
        if len(box) != 4 or not all(math.isfinite(v) for v in box):
            raise ValueError("ODM product bounds must be finite")
        if box[0] >= box[2] or box[1] >= box[3]:
            raise ValueError("ODM product has empty spatial extent")
        bounds.append(box)
    if max(b[0] for b in bounds) >= min(b[2] for b in bounds) or max(
        b[1] for b in bounds
    ) >= min(b[3] for b in bounds):
        raise ValueError("ODM product extents do not overlap")
    for raster in report["rasters"]:
        if len(raster["size"]) != 2 or min(raster["size"]) < 1:
            raise ValueError("ODM raster has no pixels")
        pixel_size = raster["pixel_size_m"]
        if len(pixel_size) != 2 or not all(math.isfinite(v) and v > 0 for v in pixel_size):
            raise ValueError("ODM raster has invalid pixel spacing")
        if not raster["bands"] or any(
            not all(math.isfinite(v) for v in band["statistics"])
            or len(band["statistics"]) != 4
            or not 0 < band["valid_percent"] <= 100
            for band in raster["bands"]
        ):
            raise ValueError("ODM raster has no finite valid data")
    cloud = report["cloud"]
    if cloud["points"] <= 0 or cloud["decoded_points"] != cloud["points"]:
        raise ValueError("ODM point cloud is empty or incomplete")
    heights = cloud["bounds_z"]
    if (len(heights) != 2 or not all(math.isfinite(v) for v in heights)
            or heights[0] > heights[1]):
        raise ValueError("ODM point-cloud heights must be finite and ordered")
    return {**report, "valid": True, "horizontal_crs": reference.to_string(),
            "accuracy_claim": "File/CRS consistency only; independent checkpoints still required"}


def inspect_products(site, dtm=False):
    import pdal
    from osgeo import gdal

    gdal.UseExceptions()
    # Keep inspection read-only, including GDAL's sidecar statistics cache.
    gdal.SetConfigOption("GDAL_PAM_ENABLED", "NO")
    gdal.SetCacheMax(128 * 1024 * 1024)
    paths = ["odm_orthophoto/odm_orthophoto.tif", "odm_dem/dsm.tif"]
    if dtm:
        paths.append("odm_dem/dtm.tif")
    rasters = []
    for name in paths:
        dataset = gdal.Open(str(site / name), gdal.GA_ReadOnly)
        transform = dataset.GetGeoTransform(can_return_null=True)
        if transform is None:
            raise ValueError(f"ODM raster has no geotransform: {name}")
        width, height = dataset.RasterXSize, dataset.RasterYSize
        corners = [gdal.ApplyGeoTransform(transform, x, y)
                   for x, y in ((0, 0), (width, 0), (0, height), (width, height))]
        bands = []
        for index in range(1, dataset.RasterCount + 1):
            band = dataset.GetRasterBand(index)
            # Exact upstream statistics decode the raster, respecting validity
            # masks (including alpha), and refuse all-nodata/all-transparent data.
            statistics = band.ComputeStatistics(False)
            valid = float(band.GetMetadataItem("STATISTICS_VALID_PERCENT") or 0)
            bands.append({"statistics": statistics, "valid_percent": valid})
        rasters.append({"path": name, "size": [width, height], "bands": bands,
                        "crs_wkt": dataset.GetProjection(),
                        "pixel_size_m": [math.hypot(transform[1], transform[4]),
                                         math.hypot(transform[2], transform[5])],
                        "bounds_xy": [min(p[0] for p in corners), min(p[1] for p in corners),
                                      max(p[0] for p in corners), max(p[1] for p in corners)]})
        dataset = None
    name = "odm_georeferencing/odm_georeferenced_model.laz"
    pipeline = pdal.Pipeline(json.dumps([
        {"type": "readers.las", "filename": str(site / name)},
        {"type": "filters.stats", "dimensions": "X,Y,Z"},
    ]))
    summary = pipeline.quickinfo["readers.las"]
    # Streaming decoding checks compressed payloads without retaining all points.
    decoded = pipeline.execute_streaming(chunk_size=65536)
    # LAS header bounds are not evidence of the decoded point coordinates.
    statistics = {item["name"]: item for item in
                  pipeline.metadata["metadata"]["filters.stats"]["statistic"]}
    for axis in ("X", "Y", "Z"):
        values = statistics[axis]
        if values["count"] != decoded or not all(
            math.isfinite(values[key]) for key in ("minimum", "maximum", "average")
        ):
            raise ValueError("ODM point cloud contains invalid decoded coordinates")
    cloud = {"path": name, "points": summary["num_points"], "decoded_points": decoded,
             "crs_wkt": summary["srs"]["wkt"],
             "bounds_xy": [statistics["X"]["minimum"], statistics["Y"]["minimum"],
                           statistics["X"]["maximum"], statistics["Y"]["maximum"]],
             "bounds_z": [statistics["Z"]["minimum"], statistics["Z"]["maximum"]]}
    return validate_products({"rasters": rasters, "cloud": cloud,
                              "versions": {"gdal": gdal.VersionInfo(), "pdal": pdal.__version__}})


if __name__ == "__main__":
    print(json.dumps(inspect_products(Path(sys.argv[1]), dtm="--dtm" in sys.argv[2:]),
                     allow_nan=False))
