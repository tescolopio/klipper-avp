import json
import math
import os
import tempfile
import unittest
from collections import namedtuple
from unittest.mock import Mock

from avp.klipper.adapter import AVP


Result = namedtuple("Result", "bed_x bed_y bed_z test_x test_y test_z")


class Command:
    def __init__(self, **params):
        self.params = params
        self.messages = []

    def get(self, key, default=None):
        return self.params.get(key, default)

    def get_int(self, key, default=None, minval=None, maxval=None):
        value = int(self.get(key, default))
        if ((minval is not None and value < minval)
                or (maxval is not None and value > maxval)):
            raise self.error("Invalid " + key)
        return value

    def get_float(self, key, default=None, minval=None):
        value = float(self.get(key, default))
        if minval is not None and value < minval:
            raise self.error("Invalid " + key)
        return value

    def error(self, message):
        return RuntimeError(message)

    def respond_info(self, message):
        self.messages.append(message)


class Config(Command):
    getint = Command.get_int

    def getfloat(self, key, default=None, above=None, maxval=None):
        raw = self.get(key, default)
        if raw is None:
            return None
        value = float(raw)
        if ((above is not None and value <= above)
                or (maxval is not None and value > maxval)):
            raise self.error("Invalid " + key)
        return value

    def getfloatlist(self, key, count):
        values = tuple(float(v) for v in self.get(key).split(","))
        if len(values) != count:
            raise self.error("Invalid " + key)
        return values

    def get_printer(self):
        return self.printer


class CommandTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.gcode = Mock()
        self.toolhead = Mock()
        self.position = [0., 0., 10., 0.]
        self.moves = []
        self.toolhead.get_position.side_effect = lambda: list(self.position)
        self.toolhead.manual_move.side_effect = self.move
        self.status = {"homed_axes": "xyz", "axis_minimum": (0, 0, -5),
                       "axis_maximum": (200, 200, 200)}
        self.toolhead.get_status.return_value = self.status
        self.probe = Mock()
        self.probe.get_offsets.return_value = (5., 3., 1.)
        self.session = Mock()
        self.probe.start_probe_session.return_value = self.session
        self.session.run_probe.side_effect = self.touch
        self.session.pull_probed_results.side_effect = lambda: [
            Result(self.position[0] + 5., self.position[1] + 3., 0.,
                   self.position[0], self.position[1], 1.)]
        self.objects = {"gcode": self.gcode, "probe": self.probe,
                        "toolhead": self.toolhead}
        self.printer = Mock()
        self.printer.lookup_object.side_effect = lambda name, default=None: (
            self.objects.get(name, default))
        self.config = Config(
            mesh_min="10,10", mesh_max="30,30", max_surface_z="0.5",
            history_path=os.path.join(directory.name, "avp.sqlite3"))
        self.config.printer = self.printer
        self.avp = AVP(self.config)
        self.avp.connect()

    def move(self, position, speed):
        for i, value in enumerate(position):
            if value is not None:
                self.position[i] = value
        self.moves.append((tuple(position), tuple(self.position), speed))

    def touch(self, gcmd):
        self.position[2] = 1.

    def test_scan_lifts_before_xy_and_approaches_above_trigger(self):
        cmd = Command()
        self.avp.cmd_AVP_SCAN(cmd)
        self.assertEqual(len(self.avp.points), 9)
        self.assertEqual(self.avp.points[0], (10., 10., 0.))
        self.assertEqual(self.avp.history.recent()[0]["stats"]["count"], 9)
        self.session.end_probe_session.assert_called_once()
        for i, (requested, actual, speed) in enumerate(self.moves):
            if requested[0] is not None:
                self.assertGreaterEqual(actual[2], 10.)
                self.assertIsNone(self.moves[i - 1][0][0])
            elif requested[2] == 3.5:
                self.assertGreater(actual[2], 1.)
        self.assertEqual(self.position[2], 10.)
        self.assertIn("3x3", cmd.messages[0])

    def test_no_bound_disables_fast_approach(self):
        self.avp.surface_bound = None
        self.avp.cmd_AVP_SCAN(Command())
        self.assertFalse(any(move[0][2] == 3.5 for move in self.moves))

    def test_legacy_probe_offsets_and_xyz_results(self):
        self.probe.get_offsets = lambda: (5., 3., 1.)
        self.session.pull_probed_results.side_effect = lambda: [
            list(self.position[:3])]
        self.avp.cmd_AVP_SCAN(Command())
        self.assertEqual(self.avp.points[0], (10., 10., 0.))
        self.assertEqual(len(self.avp.history.recent()[0]["points"]), 9)

    def test_scan_uses_historical_warp_for_adaptive_density(self):
        points = [(x, y, .2 if x == y == 20 else 0.)
                  for x in (10, 20, 30) for y in (10, 20, 30)]
        self.avp.history.save(points, {"bounds": self.avp.bounds})
        self.avp.cmd_AVP_SCAN(Command())
        self.assertGreater(len(self.avp.points), 9)
        self.assertLessEqual(len(self.avp.points), 81)

    def test_failed_probe_closes_session_without_saving_partial_scan(self):
        self.session.run_probe.side_effect = RuntimeError("Probe failed")
        with self.assertRaisesRegex(RuntimeError, "Probe failed"):
            self.avp.cmd_AVP_SCAN(Command())
        self.session.end_probe_session.assert_called_once()
        self.assertEqual(self.avp.history.recent(), [])
        self.assertEqual(self.avp.points, [])

    def test_violated_bound_stops_scan_after_retract(self):
        self.session.pull_probed_results.return_value = None
        self.session.pull_probed_results.side_effect = lambda: [
            Result(10, 10, .6, 5, 7, 1.6)]
        with self.assertRaisesRegex(RuntimeError, "bound violated"):
            self.avp.cmd_AVP_SCAN(Command())
        self.assertEqual(self.position[2], 10.)
        self.assertEqual(self.avp.history.recent(), [])
        self.session.end_probe_session.assert_called_once()

    def test_unhomed_or_unreachable_scan_does_not_move(self):
        for change in ({"homed_axes": "xy"}, {"axis_maximum": (20, 20, 200)}):
            with self.subTest(change=change):
                original = dict(self.status)
                self.status.update(change)
                with self.assertRaises(RuntimeError):
                    self.avp.cmd_AVP_SCAN(Command())
                self.status.update(original)
        self.probe.start_probe_session.assert_not_called()
        self.assertEqual(self.moves, [])

    def test_native_mesh_uses_adaptive_density_and_print_area(self):
        self.objects["bed_mesh"] = Mock()
        configfile = Mock()
        configfile.get_status.return_value = {"config": {"bed_mesh": {
            "mesh_min": "0,0", "mesh_max": "100,100"}}}
        self.objects["configfile"] = configfile
        self.avp.cmd_AVP_MESH(Command())
        script = self.gcode.run_script_from_command.call_args[0][0]
        self.assertIn("BED_MESH_CALIBRATE", script)
        self.assertIn("PROBE_COUNT=3,3 ALGORITHM=lagrange ADAPTIVE=1", script)
        self.assertIn("MESH_MIN=10.000000,10.000000", script)
        self.assertEqual(self.moves, [])

    def test_mesh_cannot_extend_native_safe_area(self):
        self.objects["bed_mesh"] = Mock()
        configfile = Mock()
        configfile.get_status.return_value = {"config": {"bed_mesh": {
            "mesh_min": "20,20", "mesh_max": "100,100"}}}
        self.objects["configfile"] = configfile
        with self.assertRaisesRegex(RuntimeError, "bounds exceed"):
            self.avp.cmd_AVP_MESH(Command())
        self.gcode.run_script_from_command.assert_not_called()

    def test_leveling_ends_with_native_tolerance_checked_pass(self):
        self.objects["quad_gantry_level"] = Mock()
        self.avp.points = [(10, 10, 0)]
        self.avp.cmd_AVP_LEVEL(Command())
        scripts = [c.args[0] for c in self.gcode.run_script_from_command.call_args_list]
        self.assertEqual(scripts, [
            "QUAD_GANTRY_LEVEL SAMPLES=1 RETRIES=0",
            "QUAD_GANTRY_LEVEL RETRIES=3 RETRY_TOLERANCE=0.050000"])
        self.assertEqual(self.avp.points, [])

    def test_leveling_failure_propagates(self):
        self.objects["z_tilt"] = Mock()
        self.gcode.run_script_from_command.side_effect = [
            None, RuntimeError("Too many retries")]
        cmd = Command()
        with self.assertRaisesRegex(RuntimeError, "Too many retries"):
            self.avp.cmd_AVP_LEVEL(cmd)
        self.assertEqual(cmd.messages, [])

    def test_leveling_rejects_command_injection(self):
        with self.assertRaises(RuntimeError):
            self.avp.cmd_AVP_LEVEL(Command(METHOD="Z_TILT_ADJUST\nG28"))
        self.gcode.run_script_from_command.assert_not_called()

    def test_clearance_and_history_commands_are_advisory_only(self):
        self.avp.cmd_AVP_SCAN(Command())
        self.moves.clear()
        cmd = Command(X0=10, Y0=10, X1=30, Y1=30)
        self.avp.cmd_AVP_CLEARANCE(cmd)
        self.assertIn("advisory only", cmd.messages[0])
        self.assertEqual(self.moves, [])
        cmd = Command(LIMIT=1)
        self.avp.cmd_AVP_HISTORY(cmd)
        self.assertIn('"stats"', cmd.messages[0])
        self.assertNotIn('"points"', cmd.messages[0])

    def test_nonfinite_config_rejected(self):
        self.config.params["clearance"] = "nan"
        with self.assertRaisesRegex(RuntimeError, "finite"):
            AVP(self.config)

    def test_plan_is_read_only_and_matches_scan_geometry(self):
        self.avp.points = [(10, 10, 0), (10, 30, 0), (30, 10, 0)]
        original = list(self.avp.points)
        cmd = Command(FAST_APPROACH=1)
        self.avp.cmd_AVP_PLAN(cmd)
        report = json.loads(cmd.messages[0].split(": ", 1)[1])
        self.assertEqual(report["point_count"], 9)
        self.assertEqual(report["travel_z"], 10.)
        self.assertEqual(report["approach_z"], 3.5)
        self.assertTrue(report["fast_approach"])
        self.assertEqual(self.avp.points, original)
        self.assertEqual(self.avp.history.recent(), [])
        self.assertEqual(self.moves, [])
        self.probe.start_probe_session.assert_not_called()
        self.gcode.run_script_from_command.assert_not_called()
        self.toolhead.wait_moves.assert_not_called()

    def test_fast_approach_requires_bound_before_motion_or_session(self):
        self.avp.surface_bound = None
        for method in (self.avp.cmd_AVP_PLAN, self.avp.cmd_AVP_SCAN):
            with self.assertRaisesRegex(RuntimeError, "requires configured"):
                method(Command(FAST_APPROACH=1))
        self.assertEqual(self.moves, [])
        self.probe.start_probe_session.assert_not_called()

    def test_fast_approach_switch_is_per_command_and_validated(self):
        self.avp.cmd_AVP_SCAN(Command(FAST_APPROACH=0))
        self.assertFalse(any(m[0][2] == 3.5 for m in self.moves))
        self.moves.clear()
        self.avp.cmd_AVP_SCAN(Command())
        self.assertTrue(any(m[0][2] == 3.5 for m in self.moves))
        self.moves.clear()
        for value in (-1, 2):
            with self.assertRaises(RuntimeError):
                self.avp.cmd_AVP_SCAN(Command(FAST_APPROACH=value))
        self.assertEqual(self.moves, [])

    def test_baseline_still_enforces_configured_surface_bound(self):
        self.session.pull_probed_results.side_effect = lambda: [
            Result(10, 10, .6, 5, 7, 1.6)]
        with self.assertRaisesRegex(RuntimeError, "bound violated"):
            self.avp.cmd_AVP_SCAN(Command(FAST_APPROACH=0))
        self.assertEqual(self.position[2], 10.)
        self.assertEqual(self.avp.history.recent(), [])
        self.session.end_probe_session.assert_called_once()

    def test_native_high_finish_is_never_followed_by_downward_retraction(self):
        def finish_high(gcmd):
            self.position[2] = 12.
        self.session.run_probe.side_effect = finish_high
        self.avp.cmd_AVP_SCAN(Command(FAST_APPROACH=0))
        self.assertEqual(self.position[2], 12.)
        for requested, actual, speed in self.moves[3:]:
            self.assertGreaterEqual(actual[2], 12.)

    def test_preflight_rejects_invalid_offsets_current_z_and_height_limits(self):
        for offset in (math.nan, math.inf):
            self.probe.get_offsets.return_value = (0, 0, offset)
            with self.assertRaisesRegex(RuntimeError, "offsets"):
                self.avp.cmd_AVP_SCAN(Command())
        self.probe.get_offsets.return_value = (5, 3, 1)
        for z in (math.nan, math.inf, 201):
            self.position[2] = z
            with self.assertRaisesRegex(RuntimeError, "current Z"):
                self.avp.cmd_AVP_SCAN(Command())
        self.position[2] = 10.
        self.avp.surface_bound = 200.
        with self.assertRaisesRegex(RuntimeError, "travel height"):
            self.avp.cmd_AVP_SCAN(Command())
        self.avp.surface_bound = -10.
        with self.assertRaisesRegex(RuntimeError, "approach height"):
            self.avp.cmd_AVP_SCAN(Command())
        self.assertEqual(self.moves, [])
        self.probe.start_probe_session.assert_not_called()

    def test_invalid_native_finish_closes_session_without_saving(self):
        for z in (math.nan, math.inf, 201):
            with self.subTest(z=z):
                self.position[2] = 10.
                self.session.reset_mock()
                self.session.run_probe.side_effect = lambda gcmd: self.position.__setitem__(2, z)
                with self.assertRaises(RuntimeError):
                    self.avp.cmd_AVP_SCAN(Command())
                self.session.end_probe_session.assert_called_once()
                self.assertEqual(self.avp.history.recent(), [])

    def test_25_point_tap_ab_preserves_samples_and_clearance(self):
        # Ideal constant-speed simulation, not a hardware timing claim.
        # Native three-sample probing and one retry batch are identical in A/B.
        self.avp.bounds = (50., 50., 450., 450.)
        self.avp.minimum = self.avp.maximum = 5
        self.avp.approach_speed = 5.
        self.status["axis_maximum"] = (500, 500, 500)
        self.probe.get_offsets.return_value = (0., 0., -1.372)
        times, results, starts = [], [], []
        for fast in (0, 1):
            self.position[:] = [50., 50., 10., 0.]
            self.moves.clear()
            elapsed, touches, probe_starts = [0.], [0], []
            command = Command(FAST_APPROACH=fast, PROBE_SPEED=2.5,
                              SAMPLES=3, SAMPLES_TOLERANCE=.01,
                              SAMPLES_TOLERANCE_RETRIES=10)

            def timed_move(position, speed):
                before = list(self.position)
                self.move(position, speed)
                elapsed[0] += math.sqrt(sum((a - b) ** 2 for a, b in
                                           zip(self.position[:3], before[:3]))) / speed

            def native_probe(gcmd):
                self.assertIs(gcmd, command)
                self.assertEqual(gcmd.params["SAMPLES_TOLERANCE"], .01)
                self.assertEqual(gcmd.params["SAMPLES_TOLERANCE_RETRIES"], 10)
                probe_starts.append(self.position[2])
                elapsed[0] += (self.position[2] - (-1.372)) / 2.5
                batch = gcmd.params["SAMPLES"]
                # One identical simulated tolerance retry at the first point.
                n = batch * (2 if len(probe_starts) == 1 else 1)
                elapsed[0] += (n - 1) * (5. / 2.5 + 5. / 2.5)
                touches[0] += n
                self.position[2] = -1.372

            self.toolhead.manual_move.side_effect = timed_move
            self.session.run_probe.side_effect = native_probe
            self.session.pull_probed_results.side_effect = lambda: [Result(
                self.position[0], self.position[1], 0.,
                self.position[0], self.position[1], -1.372)]
            self.avp.cmd_AVP_SCAN(command)
            self.assertEqual(touches[0], 78)
            self.assertEqual(len(probe_starts), 25)
            self.assertEqual(self.position[2], 10.)
            for requested, actual, speed in self.moves:
                if requested[0] is not None:
                    self.assertGreaterEqual(actual[2], 10.)
            times.append(elapsed[0])
            results.append(list(self.avp.points))
            starts.append(probe_starts)
        self.assertEqual(results[0], results[1])
        self.assertEqual(starts[0], [10.] * 25)
        self.assertEqual(starts[1], [2.5] * 25)
        # 25 * 7.5 mm * (1/2.5 - 1/5) = 37.5 seconds saved in this model.
        self.assertAlmostEqual(times[0] - times[1], 37.5)


if __name__ == "__main__":
    unittest.main()
