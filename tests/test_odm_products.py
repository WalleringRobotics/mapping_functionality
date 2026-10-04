import copy

import pytest
from pyproj import CRS

from wallering_mapping.odm_products import validate_products


def products():
    common = {"crs_wkt": CRS.from_epsg(32632).to_wkt(),
              "bounds_xy": [500000, 6000000, 500100, 6000100]}
    raster = {**common, "size": [100, 100], "pixel_size_m": [1, 1],
              "bands": [{"statistics": [100, 110, 105, 2], "valid_percent": 75}]}
    return {"rasters": [copy.deepcopy(raster), copy.deepcopy(raster)],
            "cloud": {**common, "points": 10, "decoded_points": 10}}


def test_consistent_metric_terrain_products():
    result = validate_products(products())
    assert result["valid"] and result["horizontal_crs"] == "EPSG:32632"
    assert "checkpoints" in result["accuracy_claim"]


@pytest.mark.parametrize("failure,reason", [
    ("crs", "coordinate systems disagree"),
    ("degrees", "metre axes"),
    ("feet", "metre axes"),
    ("extent", "do not overlap"),
    ("bounds", "finite"),
    ("empty", "empty spatial extent"),
    ("pixels", "no pixels"),
    ("spacing", "pixel spacing"),
    ("nodata", "finite valid data"),
    ("infinite", "finite valid data"),
    ("truncated", "empty or incomplete"),
])
def test_invalid_terrain_products(failure, reason):
    report = products()
    raster = report["rasters"][0]
    if failure in {"crs", "degrees", "feet"}:
        raster["crs_wkt"] = CRS.from_epsg({"crs": 32633, "degrees": 4326, "feet": 2263}[failure]).to_wkt()
    elif failure == "extent":
        raster["bounds_xy"] = [100, 100, 200, 200]
    elif failure == "bounds":
        raster["bounds_xy"][0] = float("nan")
    elif failure == "empty":
        raster["bounds_xy"][2] = raster["bounds_xy"][0]
    elif failure == "pixels":
        raster["size"][0] = 0
    elif failure == "spacing":
        raster["pixel_size_m"][0] = 0
    elif failure == "nodata":
        raster["bands"][0]["valid_percent"] = 0
    elif failure == "infinite":
        raster["bands"][0]["statistics"][0] = float("inf")
    elif failure == "truncated":
        report["cloud"]["decoded_points"] = 9
    with pytest.raises(ValueError, match=reason):
        validate_products(report)
