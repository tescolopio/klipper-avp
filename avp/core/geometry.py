"""Shared measurement validation."""

import math


def validate_points(points):
    points = [tuple(float(v) for v in point) for point in points]
    if not points or any(len(p) != 3 or not all(math.isfinite(v) for v in p)
                         for p in points):
        raise ValueError("Expected finite XYZ measurements")
    if len({p[:2] for p in points}) != len(points):
        raise ValueError("Duplicate measurement coordinates")
    return points
