import json
import subprocess
import sys
import unittest

from avp.analytics.baseline import build_report


class BaselineTests(unittest.TestCase):
    def test_point_budgets_and_adversarial_peak(self):
        cases = {case["name"]: case for case in build_report()["cases"]}
        for name in ("flat", "tilted_plane", "unsampled_peak"):
            self.assertEqual(cases[name]["planned_points"], 9)
        self.assertEqual(cases["broad_warp"]["planned_points"], 81)
        self.assertAlmostEqual(cases["unsampled_peak"]["missed_peak_mm"], .5)
        for case in cases.values():
            self.assertEqual(case["fixed_points"], 81)
            self.assertGreaterEqual(case["clearance"]["recommended_z"],
                                    case["sampled_peak_mm"] + 2.)

    def test_report_is_deterministic_and_finite_json(self):
        first = json.dumps(build_report(), sort_keys=True, allow_nan=False)
        self.assertEqual(first, json.dumps(build_report(), sort_keys=True,
                                         allow_nan=False))

    def test_cli_emits_report_without_importing_klipper_adapter(self):
        result = subprocess.run(
            [sys.executable, "-c",
             "import runpy, sys; "
             "runpy.run_module('avp.analytics.baseline', run_name='__main__'); "
             "assert 'avp.klipper.adapter' not in sys.modules"],
            capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(json.loads(result.stdout)["kind"],
                         "synthetic_planning_baseline")


if __name__ == "__main__":
    unittest.main()
