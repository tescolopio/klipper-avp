import math
import os
import tempfile
import unittest

from avp.core.probing import (
    adaptive_count, clearance_prediction, grid_points, predict_height, scan_heights,
)
from avp.analytics.topography import surface_stats
from avp.history.storage import History


def plane():
    return [(x, y, 0.01 * x - 0.02 * y + 1.)
            for x, y in grid_points((0., 0., 20., 20.), 3)]


class PlanningTests(unittest.TestCase):
    def test_scan_height_envelopes_for_positive_and_negative_probe_offsets(self):
        for offset in (-1.372, 0., 1., 12.):
            for bound in (-.5, 0., .5, 15.):
                with self.subTest(offset=offset, bound=bound):
                    fast = scan_heights(10., 2., offset, bound, True)
                    slow = scan_heights(10., 2., offset, bound, False)
                    self.assertEqual(fast["travel_z"], slow["travel_z"])
                    self.assertGreaterEqual(fast["travel_z"], 10.)
                    self.assertGreaterEqual(fast["travel_z"], fast["approach_z"])
                    self.assertGreaterEqual(fast["approach_z"], bound + 2.)
                    self.assertGreaterEqual(fast["approach_z"], bound + offset + 2.)
                    self.assertIsNone(slow["approach_z"])

    def test_scan_height_invalid_inputs_and_overflow_rejected(self):
        for args in ((0, 2, 0), (10, -1, 0), (10, 2, math.nan),
                     (math.inf, 2, 0), (10, 2, 0, math.inf),
                     (10, 1.e308, 1.e308), (10, 2, 0, None, True)):
            with self.subTest(args=args), self.assertRaises(ValueError):
                scan_heights(*args)

    def test_scan_without_bound_has_no_fast_descent(self):
        report = scan_heights(10, 2, -1.372)
        self.assertIsNone(report["approach_z"])
        self.assertEqual(report["travel_z"], 10)

    def test_tilt_does_not_increase_grid_density(self):
        stats = surface_stats(plane())
        self.assertAlmostEqual(stats["tilt_x"], .01)
        self.assertAlmostEqual(stats["tilt_y"], -.02)
        self.assertAlmostEqual(stats["warp"], 0.)
        self.assertEqual(adaptive_count(plane(), .05), 3)
        self.assertEqual(adaptive_count([], .05), 3)

    def test_warp_increases_density_with_bounded_point_budget(self):
        points = plane()
        points = [(x, y, z + (.2 if x == y == 10. else 0.))
                  for x, y, z in points]
        self.assertGreater(adaptive_count(points, .05), 3)
        self.assertEqual(adaptive_count(points, .001), 9)
        self.assertEqual(len(grid_points((0, 0, 20, 20), 9)), 81)

    def test_serpentine_grid_contains_unique_endpoints(self):
        grid = grid_points((0, 0, 20, 20), 3)
        self.assertEqual(grid[:3], [(0, 0), (10, 0), (20, 0)])
        self.assertEqual(grid[3:6], [(20, 10), (10, 10), (0, 10)])
        self.assertEqual(len(set(grid)), 9)

    def test_clearance_respects_global_peak_and_explicit_bound(self):
        points = [(0, 0, 0), (0, 20, .5), (20, 0, .1), (20, 20, 1)]
        self.assertEqual(predict_height(points, 20, 20), 1)
        report = clearance_prediction(points, (0, 0), (20, 0), 2, 3)
        self.assertEqual(report["recommended_z"], 5)
        self.assertLess(report["predicted_peak"], 1)
        self.assertEqual(clearance_prediction(
            points, (0, 0), (20, 0), 2)["recommended_z"], 3)

    def test_invalid_geometry_and_extrapolation_rejected(self):
        cases = [
            lambda: surface_stats([(0, 0, 0), (1, 1, 0), (2, 2, 0)]),
            lambda: surface_stats([(0, 0, 0), (0, 0, 1)]),
            lambda: surface_stats([(0, 0, math.nan)]),
            lambda: grid_points((1, 0, 0, 10), 3),
            lambda: grid_points((0, 0, 10, 10), 4),
            lambda: predict_height(plane(), -1, 0),
            lambda: clearance_prediction(plane(), (0, 0), (20, 20), 0),
            lambda: clearance_prediction(plane(), (0, 0), (20, 20), 2, math.inf),
        ]
        for operation in cases:
            with self.subTest(operation=operation), self.assertRaises(ValueError):
                operation()


class HistoryTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.path = os.path.join(self.directory.name, "history.sqlite3")
        self.history = History(self.path, limit=2)
        self.addCleanup(self.directory.cleanup)

    def test_persistence_retention_and_matching_point_changes(self):
        for shift in (0, .1, .2):
            self.history.save([(x, y, z + shift) for x, y, z in plane()],
                              {"bed_temperature": 60})
        reopened = History(self.path, limit=2)
        self.assertEqual(len(reopened.recent()), 2)
        self.assertEqual(reopened.recent()[0]["id"], 3)
        comparison = reopened.compare()
        self.assertEqual(comparison["common_points"], 9)
        self.assertAlmostEqual(comparison["mean_height_change"], .1)
        self.assertAlmostEqual(comparison["warp_change"], 0)
        self.assertTrue(comparison["same_conditions"])

    def test_empty_and_incomparable_history(self):
        self.assertEqual(self.history.recent(), [])
        self.assertIsNone(self.history.compare())
        self.history.save(plane(), {"bed_temperature": 20})
        self.assertIsNone(self.history.compare())
        self.history.save([(x + 100, y, z) for x, y, z in plane()],
                          {"bed_temperature": 60})
        comparison = self.history.compare()
        self.assertEqual(comparison["common_points"], 0)
        self.assertIsNone(comparison["mean_height_change"])
        self.assertFalse(comparison["same_conditions"])

    def test_invalid_scan_never_saved(self):
        with self.assertRaises(ValueError):
            self.history.save([(0, 0, 0)], {})
        with self.assertRaises(ValueError):
            self.history.save(plane(), {"temperature": math.nan})
        self.assertEqual(self.history.recent(), [])


if __name__ == "__main__":
    unittest.main()
