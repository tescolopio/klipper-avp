"""Motion-independent planning and topography history for AVP."""

import json
import math
import sqlite3
from contextlib import closing
from datetime import datetime, timezone


def validate_points(points):
    points = [tuple(float(v) for v in point) for point in points]
    if not points or any(len(p) != 3 or not all(math.isfinite(v) for v in p)
                         for p in points):
        raise ValueError("Expected finite XYZ measurements")
    if len({p[:2] for p in points}) != len(points):
        raise ValueError("Duplicate measurement coordinates")
    return points


def surface_stats(points):
    """Fit a plane and separate bed tilt from non-planar deformation."""
    points = validate_points(points)
    count = len(points)
    means = [sum(p[i] for p in points) / count for i in range(3)]
    centered = [tuple(p[i] - means[i] for i in range(3)) for p in points]
    xx = sum(x * x for x, y, z in centered)
    yy = sum(y * y for x, y, z in centered)
    xy = sum(x * y for x, y, z in centered)
    xz = sum(x * z for x, y, z in centered)
    yz = sum(y * z for x, y, z in centered)
    det = xx * yy - xy * xy
    if det <= max(xx * yy, 1.0) * 1.e-12:
        raise ValueError("At least three non-collinear measurements required")
    a = (xz * yy - yz * xy) / det
    b = (yz * xx - xz * xy) / det
    c = means[2] - a * means[0] - b * means[1]
    residuals = [z - (a * x + b * y + c) for x, y, z in points]
    return {
        "count": count, "minimum": min(p[2] for p in points),
        "maximum": max(p[2] for p in points),
        "range": max(p[2] for p in points) - min(p[2] for p in points),
        "tilt_x": a, "tilt_y": b, "intercept": c,
        "warp": max(residuals) - min(residuals),
        "rms": math.sqrt(sum(r * r for r in residuals) / count),
    }


def grid_points(bounds, count):
    x0, y0, x1, y1 = bounds
    if not all(math.isfinite(v) for v in bounds) or x0 >= x1 or y0 >= y1:
        raise ValueError("Expected increasing finite mesh bounds")
    if count < 3 or count % 2 != 1:
        raise ValueError("Grid size must be odd and at least three")
    return [(x0 + (x1 - x0) * i / (count - 1),
             y0 + (y1 - y0) * j / (count - 1))
            for j in range(count)
            for i in (range(count) if j % 2 == 0
                      else range(count - 1, -1, -1))]


def adaptive_count(points, threshold, minimum=3, maximum=9):
    """Increase regular-grid density for deformation, not simple bed tilt."""
    if threshold <= 0 or minimum < 3 or maximum < minimum:
        raise ValueError("Invalid adaptive grid limits")
    if minimum % 2 != 1 or maximum % 2 != 1:
        raise ValueError("Adaptive grid limits must be odd")
    if not points:
        return minimum
    warp = surface_stats(points)["warp"]
    steps = max(0, math.ceil(warp / threshold - 1.e-9) - 1)
    return min(maximum, minimum + 2 * steps)


def predict_height(points, x, y):
    """Inverse-distance estimate, restricted to the measured rectangle."""
    points = validate_points(points)
    if not math.isfinite(x) or not math.isfinite(y):
        raise ValueError("Expected finite XY coordinates")
    if not (min(p[0] for p in points) <= x <= max(p[0] for p in points)
            and min(p[1] for p in points) <= y <= max(p[1] for p in points)):
        raise ValueError("Prediction outside measured bounds")
    weights = []
    for px, py, pz in points:
        distance2 = (x - px) ** 2 + (y - py) ** 2
        if distance2 < 1.e-16:
            return pz
        weights.append((1.0 / distance2, pz))
    return sum(w * z for w, z in weights) / sum(w for w, z in weights)


def clearance_prediction(points, start, end, margin, surface_bound=None):
    """Advisory path estimate plus a conservative global measured envelope."""
    if not math.isfinite(margin) or margin <= 0:
        raise ValueError("Clearance margin must be positive and finite")
    points = validate_points(points)
    heights = [predict_height(
        points, start[0] + (end[0] - start[0]) * i / 32,
        start[1] + (end[1] - start[1]) * i / 32) for i in range(33)]
    envelope = max(p[2] for p in points)
    if surface_bound is not None:
        if not math.isfinite(surface_bound):
            raise ValueError("Surface bound must be finite")
        envelope = max(envelope, surface_bound)
    return {"predicted_peak": max(heights),
            "recommended_z": envelope + margin,
            "margin": margin}


class History:
    """Bounded SQLite history; only completed, validated scans are stored."""

    def __init__(self, path, limit=100):
        if limit < 1:
            raise ValueError("History limit must be positive")
        self.path, self.limit = path, limit
        with closing(sqlite3.connect(path)) as db, db:
            db.execute(
                "CREATE TABLE IF NOT EXISTS scans "
                "(id INTEGER PRIMARY KEY, timestamp TEXT NOT NULL, "
                "points TEXT NOT NULL, metadata TEXT NOT NULL)")

    def save(self, points, metadata):
        points = validate_points(points)
        surface_stats(points)
        payload = json.dumps(points, allow_nan=False)
        meta = json.dumps(metadata, allow_nan=False)
        timestamp = datetime.now(timezone.utc).isoformat()
        with closing(sqlite3.connect(self.path)) as db, db:
            cursor = db.execute(
                "INSERT INTO scans(timestamp, points, metadata) VALUES(?,?,?)",
                (timestamp, payload, meta))
            scan_id = cursor.lastrowid
            db.execute("DELETE FROM scans WHERE id NOT IN "
                       "(SELECT id FROM scans ORDER BY id DESC LIMIT ?)",
                       (self.limit,))
        return scan_id

    def recent(self, limit=10):
        if limit < 1:
            raise ValueError("History query limit must be positive")
        with closing(sqlite3.connect(self.path)) as db:
            rows = db.execute(
                "SELECT id, timestamp, points, metadata FROM scans "
                "ORDER BY id DESC LIMIT ?", (min(limit, self.limit),)).fetchall()
        return [{"id": row[0], "timestamp": row[1],
                 "points": validate_points(json.loads(row[2])),
                 "metadata": json.loads(row[3]),
                 "stats": surface_stats(json.loads(row[2]))} for row in rows]

    def compare(self):
        scans = self.recent(2)
        if len(scans) < 2:
            return None
        latest, previous = scans
        old = {p[:2]: p[2] for p in previous["points"]}
        changes = [z - old[(x, y)] for x, y, z in latest["points"]
                   if (x, y) in old]
        return {
            "latest_id": latest["id"], "previous_id": previous["id"],
            "common_points": len(changes),
            "mean_height_change": (sum(changes) / len(changes)
                                   if changes else None),
            "max_abs_height_change": (max(abs(v) for v in changes)
                                      if changes else None),
            "warp_change": latest["stats"]["warp"] - previous["stats"]["warp"],
            "same_conditions": latest["metadata"] == previous["metadata"],
        }
