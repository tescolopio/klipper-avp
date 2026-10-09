import copy
import math
import os
import tempfile
import unittest
from datetime import datetime, timedelta, timezone

from avp.history.eligibility import assess_history_context
from avp.history.storage import History


class HistoryEligibilityTests(unittest.TestCase):
    def setUp(self):
        self.now = datetime(2026, 10, 9, 12, tzinfo=timezone.utc)
        self.scan = {"timestamp": (self.now - timedelta(seconds=60)).isoformat(),
                     "metadata": {"bounds": [0, 0, 20, 20],
                                  "bed_temperature": 60.}}
        self.policy = dict(now=self.now, max_age_seconds=60,
                           bounds=(0, 0, 20, 20), bed_temperature=60.,
                           temperature_tolerance=2.)

    def assess(self, scan=None, **changes):
        return assess_history_context(
            self.scan if scan is None else scan, **dict(self.policy, **changes))

    def test_inclusive_age_boundary_and_no_mutation(self):
        original = copy.deepcopy(self.scan)
        self.assertEqual(self.assess(), {
            "eligible": True, "reason": "eligible", "age_seconds": 60.})
        self.assertEqual(self.scan, original)
        self.assertEqual(self.assess(max_age_seconds=59.999)["reason"], "stale")

    def test_future_and_timezone_normalization(self):
        self.scan["timestamp"] = "2026-10-09T07:59:00-04:00"
        self.assertTrue(self.assess()["eligible"])
        self.scan["timestamp"] = (self.now + timedelta(microseconds=1)).isoformat()
        self.assertEqual(self.assess()["reason"], "future_timestamp")

    def test_invalid_or_missing_timestamps_fail_closed(self):
        for value in (None, "bad", "2026-10-09", "2026-10-09T12:00:00", 12, []):
            with self.subTest(value=value):
                self.scan["timestamp"] = value
                self.assertEqual(self.assess()["reason"], "invalid_timestamp")
        del self.scan["timestamp"]
        self.assertFalse(self.assess()["eligible"])

    def test_temperature_boundary_and_mismatch(self):
        for value in (58., 62.):
            self.scan["metadata"]["bed_temperature"] = value
            self.assertTrue(self.assess()["eligible"])
        self.scan["metadata"]["bed_temperature"] = 62.001
        self.assertEqual(self.assess()["reason"], "temperature_mismatch")

    def test_missing_or_invalid_temperature_rejected(self):
        for value in (None, "60", True, math.nan, math.inf, 10 ** 400):
            with self.subTest(value=value):
                self.scan["metadata"]["bed_temperature"] = value
                self.assertEqual(self.assess()["reason"], "invalid_temperature")
        del self.scan["metadata"]["bed_temperature"]
        self.assertFalse(self.assess()["eligible"])

    def test_invalid_and_mismatched_bounds_rejected(self):
        for value in (None, [], [0, 0, 20], [0, 0, 0, 20],
                      [0, 0, math.inf, 20], [False, 0, 20, 20]):
            with self.subTest(value=value):
                self.scan["metadata"]["bounds"] = value
                self.assertEqual(self.assess()["reason"], "invalid_bounds")
        self.scan["metadata"]["bounds"] = [0, 0, 21, 20]
        self.assertEqual(self.assess()["reason"], "bounds_mismatch")

    def test_malformed_metadata_and_scan_rejected(self):
        for value in (None, [], "metadata"):
            self.scan["metadata"] = value
            self.assertEqual(self.assess()["reason"], "invalid_metadata")
        for value in (None, [], "scan"):
            self.assertEqual(assess_history_context(value, **self.policy)["reason"],
                             "invalid_scan")

    def test_invalid_policy_raises(self):
        cases = ({"now": self.now.replace(tzinfo=None)}, {"now": None},
                 {"max_age_seconds": 0}, {"max_age_seconds": -1},
                 {"max_age_seconds": math.inf}, {"max_age_seconds": True},
                 {"max_age_seconds": 10 ** 400},
                 {"bounds": (0, 0, 0, 20)}, {"bed_temperature": math.nan},
                 {"temperature_tolerance": -1},
                 {"temperature_tolerance": math.nan})
        for changes in cases:
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                self.assess(**changes)

    def test_existing_sqlite_records_work_without_migration(self):
        with tempfile.TemporaryDirectory() as directory:
            history = History(os.path.join(directory, "history.sqlite3"))
            history.save([(0, 0, 0), (0, 20, 0), (20, 0, 0)],
                         self.scan["metadata"])
            saved = history.recent()[0]
            captured = datetime.fromisoformat(saved["timestamp"])
            self.assertTrue(self.assess(saved, now=captured)["eligible"])
            self.assertEqual(self.assess(
                saved, now=captured + timedelta(seconds=61))["reason"], "stale")


if __name__ == "__main__":
    unittest.main()
