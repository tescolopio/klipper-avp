"""Bed surface statistics and comparisons independent of persistence."""

import math

from ..core.geometry import validate_points


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


def compare_scans(scans):
    """Compare the two most recent scans, supplied newest first."""
    if len(scans) < 2:
        return None
    latest, previous = scans[:2]
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
