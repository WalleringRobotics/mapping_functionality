#!/usr/bin/env python3
"""Exercise the product checks against real GDAL/PDAL APIs in the ODM image."""

import json
import sys
import tempfile
from pathlib import Path

import numpy as np
import pdal
from osgeo import gdal, osr

def main():
    # Preserve the image's PYTHONPATH for its compiled GDAL/PDAL bindings.
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
    from wallering_mapping.odm_products import inspect_products

    gdal.UseExceptions()
    with tempfile.TemporaryDirectory(prefix="wr-odm-products-") as temporary:
        root = Path(temporary)
        reference = osr.SpatialReference()
        reference.ImportFromEPSG(32632)
        for name in ("odm_orthophoto/odm_orthophoto.tif", "odm_dem/dsm.tif"):
            path = root / name
            path.parent.mkdir()
            orthophoto = "orthophoto" in name
            dataset = gdal.GetDriverByName("GTiff").Create(
                str(path), 8, 8, 4 if orthophoto else 1,
                gdal.GDT_Byte if orthophoto else gdal.GDT_Float32)
            dataset.SetGeoTransform([500000, 1, 0, 6000008, 0, -1])
            dataset.SetProjection(reference.ExportToWkt())
            for index in range(1, 4 if orthophoto else 2):
                dataset.GetRasterBand(index).WriteArray(np.arange(64, dtype=np.float32).reshape(8, 8))
            if orthophoto:
                for index, color in enumerate((gdal.GCI_RedBand, gdal.GCI_GreenBand,
                                                gdal.GCI_BlueBand, gdal.GCI_AlphaBand), 1):
                    dataset.GetRasterBand(index).SetColorInterpretation(color)
                dataset.GetRasterBand(4).Fill(255)
            dataset = None
        cloud = root / "odm_georeferencing/odm_georeferenced_model.laz"
        cloud.parent.mkdir()
        points = np.array([(500001, 6000001, 100), (500002, 6000006, 101),
                           (500006, 6000002, 102)],
                          dtype=[("X", "f8"), ("Y", "f8"), ("Z", "f8")])
        pdal.Writer.las(filename=str(cloud), a_srs="EPSG:32632").pipeline(points).execute()
        report = inspect_products(root)
        assert report["valid"] and report["cloud"]["decoded_points"] == 3
        assert report["rasters"][0]["bands"][0]["valid_percent"] == 100
        np.testing.assert_allclose(report["cloud"]["bounds_xy"],
                                   [500001, 6000001, 500006, 6000006])
        np.testing.assert_allclose(report["cloud"]["bounds_z"], [100, 102])
        # Finite RGB behind an entirely transparent alpha is not a usable map.
        orthophoto = root / "odm_orthophoto/odm_orthophoto.tif"
        dataset = gdal.Open(str(orthophoto), gdal.GA_Update)
        dataset.GetRasterBand(4).Fill(0)
        dataset = None
        try:
            inspect_products(root)
        except RuntimeError as error:
            assert "no valid pixels" in str(error)
        else:
            raise AssertionError("Fully transparent orthophoto was accepted")
        dataset = gdal.Open(str(orthophoto), gdal.GA_Update)
        dataset.GetRasterBand(4).Fill(255)
        dataset = None
        # A nonempty file with the wrong reference must not qualify as a map.
        reference.ImportFromEPSG(32633)
        dataset = gdal.Open(str(root / "odm_dem/dsm.tif"), gdal.GA_Update)
        dataset.SetProjection(reference.ExportToWkt())
        dataset = None
        try:
            inspect_products(root)
        except ValueError as error:
            assert "coordinate systems disagree" in str(error)
        else:
            raise AssertionError("Mismatched raster CRS was accepted")
        print(json.dumps({"valid": True, "fixture": "synthetic GDAL/PDAL file contract",
                          "checks": ["GeoTIFF decode", "alpha mask refusal", "LAZ streaming decode",
                                     "decoded XYZ bounds", "CRS mismatch refusal"],
                          "versions": report["versions"]}))


if __name__ == "__main__":
    main()
