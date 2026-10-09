"""Synthetic planning baseline: python -m avp.analytics.baseline."""

import json
import math

from ..core.probing import adaptive_count, clearance_prediction, grid_points


def build_report():
    """Compare planned point budgets against a fixed 9x9 reference grid.

    These analytic surfaces are fixtures, not printer measurements. The narrow
    peak deliberately falls between 3x3 samples. No elapsed-time or hardware
    accuracy claim can be inferred from this report.
    """
    bounds = (0., 0., 20., 20.)
    fixtures = (
        ("flat", lambda x, y: 0.),
        ("tilted_plane", lambda x, y: .01 * x - .02 * y + 1.),
        ("broad_warp", lambda x, y: .2 * (1 - ((x - 10) / 10) ** 2)
         * (1 - ((y - 10) / 10) ** 2)),
        ("unsampled_peak", lambda x, y: .5 * math.exp(
            -((x - 5) ** 2 + (y - 5) ** 2))),
    )
    cases = []
    reference = grid_points(bounds, 9)
    for name, surface in fixtures:
        history = [(x, y, surface(x, y)) for x, y in grid_points(bounds, 3)]
        count = adaptive_count(history, .05, 3, 9)
        prediction = clearance_prediction(history, (0., 0.), (20., 20.), 2.)
        sampled_peak = max(z for x, y, z in history)
        reference_peak = max(surface(x, y) for x, y in reference)
        cases.append({
            "name": name, "history_points": len(history),
            "planned_points": count * count, "fixed_points": len(reference),
            "point_reduction_fraction": 1 - count * count / len(reference),
            "sampled_peak_mm": sampled_peak,
            "reference_grid_peak_mm": reference_peak,
            "missed_peak_mm": max(0., reference_peak - sampled_peak),
            "clearance": prediction,
        })
    return {"schema_version": 1, "kind": "synthetic_planning_baseline",
            "parameters": {"bounds_mm": bounds, "warp_threshold_mm": .05,
                           "minimum_count": 3, "maximum_count": 9,
                           "clearance_margin_mm": 2.},
            "cases": cases}


if __name__ == "__main__":
    print(json.dumps(build_report(), indent=2, sort_keys=True, allow_nan=False))
