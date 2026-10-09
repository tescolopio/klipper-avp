"""Motion-independent adaptive grids and clearance prediction."""

import math
from .geometry import validate_points
from ..analytics.topography import surface_stats


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

