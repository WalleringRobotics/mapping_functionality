"""Strict ODM control/geolocation readers using explicit coordinate systems."""

from collections import defaultdict

import numpy as np


def read_lines(path):
    from pyproj import CRS
    lines = [line.strip() for line in path.read_text().splitlines()
             if line.strip() and not line.lstrip().startswith("#")]
    if len(lines) < 2:
        raise ValueError("Control file needs a CRS header and observations")
    crs = CRS.from_user_input(lines[0])
    if not (crs.is_projected or crs.is_geographic):
        raise ValueError("Use an explicit projected or geographic CRS")
    return lines[0], lines[1:]


def gcps(path, names, width, height):
    crs, lines = read_lines(path)
    records, points, views = [], {}, defaultdict(set)
    for line in lines:
        fields = line.split()
        if len(fields) != 7:
            raise ValueError("GCP rows require X Y Z pixel_x pixel_y image_name label")
        values = list(map(float, fields[:5]))
        if not np.isfinite(values).all():
            raise ValueError("GCP coordinates must be finite")
        x, y, z, u, v = values
        image, label = fields[5:]
        if image not in names:
            raise ValueError(f"GCP image is not selected: {image}; retain it by reducing the export interval")
        if not (0 <= u <= width - 1 and 0 <= v <= height - 1):
            raise ValueError(f"GCP pixel outside original image: {image}")
        if label in points and points[label] != (x, y, z):
            raise ValueError(f"Conflicting coordinates for GCP {label}")
        if image in views[label]:
            raise ValueError(f"Duplicate GCP/image observation: {label}/{image}")
        points[label] = x, y, z
        views[label].add(image)
        records.append({"xyz": [x, y, z], "pixel": [u, v], "image": image, "label": label})
    if len(points) < 5 or any(len(images) < 3 for images in views.values()):
        raise ValueError("Terrain GCP input requires at least 5 distinct points, each in 3 selected images")
    xy = np.array(list(points.values()))[:, :2]
    if np.linalg.matrix_rank(xy - xy.mean(axis=0)) < 2:
        raise ValueError("GCP layout is collinear; distribute controls over the survey")
    return {"crs": crs, "points": len(points), "records": records}


def geolocation(path, names):
    crs, lines = read_lines(path)
    rows, seen = [], set()
    for line in lines:
        fields = line.split()
        if len(fields) not in (4, 9):
            raise ValueError("Geo rows require image X Y Z, optionally yaw pitch roll horizontal_sigma vertical_sigma")
        name = fields[0]
        if name not in names or name in seen:
            raise ValueError(f"Unknown or duplicated geolocation image: {name}")
        numbers = list(map(float, fields[1:]))
        if not np.isfinite(numbers).all() or (len(numbers) == 8 and min(numbers[-2:]) <= 0):
            raise ValueError("Geolocation values must be finite, with positive stated accuracy")
        rows.append({"image": name, "values": numbers})
        seen.add(name)
    if len(rows) < 3:
        raise ValueError("At least three camera positions are required for georeferencing")
    xy = np.array([row["values"][:2] for row in rows])
    if np.linalg.matrix_rank(xy - xy.mean(axis=0)) < 2:
        raise ValueError("Camera geolocation layout is collinear")
    return {"crs": crs, "records": rows}

