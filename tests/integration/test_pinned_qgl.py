"""Pinned Klipper QGL contract tests; no MCU, heaters or real motion.

Run separately with KLIPPER_SOURCE pointing at the documented clean checkout.
Real QGL, retry, point sequencing and sample averaging code runs. Hardware
sampling and stepper adjustment are simulated, not a model of stopping distance.
"""
import os
from pathlib import Path
import subprocess
import sys
import unittest
from unittest.mock import Mock

PIN = "461c4e3722c3a897fba1c6b3f0780a5315043842"
source = os.environ.get("KLIPPER_SOURCE")
if not source:
    raise RuntimeError("Set KLIPPER_SOURCE to the pinned Klipper checkout")
root = Path(source).resolve()
revision = subprocess.check_output(
    ["git", "-C", str(root), "rev-parse", "HEAD"], text=True).strip()
if revision != PIN:
    raise RuntimeError("Klipper revision mismatch: expected " + PIN)
if subprocess.check_output(
        ["git", "-C", str(root), "status", "--porcelain", "--untracked-files=no"],
        text=True).strip():
    raise RuntimeError("Pinned Klipper checkout has tracked modifications")
sys.path.insert(0, str(root / "klippy"))
import gcode
from extras import probe, quad_gantry_level, manual_probe
from avp.klipper.adapter import AVP

CORNERS = [(50., 50.), (50., 450.), (450., 450.), (450., 50.)]


class Config:
    def __init__(self, printer):
        self.printer = printer
        self.values = dict(retries=10, retry_tolerance=.01, max_adjust=4.,
                           horizontal_move_z=10., speed=2.5, lift_speed=2.5,
                           samples=3, sample_retract_dist=5.,
                           samples_result="median", samples_tolerance=.01,
                           samples_tolerance_retries=10)

    def get_printer(self):
        return self.printer

    def get_name(self):
        return "quad_gantry_level"

    def get(self, key, default=None):
        return self.values.get(key, default)

    def getint(self, key, default=None, **kwargs):
        return int(self.get(key, default))

    def getfloat(self, key, default=None, **kwargs):
        return float(self.get(key, default))

    def getchoice(self, key, choices, default):
        return self.get(key, default)

    def getlists(self, key, **kwargs):
        # Synthetic gantry geometry, not BigBoom's verified motor locations.
        return CORNERS if key == "points" else [(0., 0.), (500., 500.)]

    error = gcode.CommandError


class Rig:
    command_error = gcode.CommandError

    def __init__(self, rounds, fault_at=None, noise=None):
        self.rounds, self.fault_at = rounds, fault_at
        self.noise = noise or {}
        self.handlers, self.objects, self.commands = {}, {}, {}
        self.position = [250., 250., 10., 0.]
        self.touches, self.moves, self.adjustments, self.scripts = [], [], [], []
        self.messages, self.ends, self.round_index = [], 0, 0
        self.homed = "xyz"
        self.gcode = Mock()
        self.gcode.error = gcode.CommandError
        self.gcode.respond_info.side_effect = self.messages.append
        self.gcode.register_command.side_effect = (
            lambda name, handler, **kwargs: self.commands.update({name: handler}))
        self.gcode.create_gcode_command.side_effect = self.command
        self.gcode.run_script_from_command.side_effect = self.dispatch
        self.toolhead = Mock()
        self.toolhead.get_position.side_effect = lambda: list(self.position)
        self.toolhead.manual_move.side_effect = self.move
        self.toolhead.get_status.side_effect = lambda t: {"homed_axes": self.homed}
        self.objects.update(gcode=self.gcode, toolhead=self.toolhead)
        config = Config(self)
        self.params = probe.ProbeParameterHelper(config)
        self.sampling = probe.SampleAveragingHelper(config, self.params, lambda cmd: self)
        probe_object = Mock()
        probe_object.get_probe_params.side_effect = self.params.get_probe_params
        probe_object.get_offsets.return_value = (0., 0., -1.372)
        probe_object.start_probe_session.side_effect = self.sampling.start_probe_session
        self.objects["probe"] = probe_object
        self.qgl = quad_gantry_level.QuadGantryLevel(config)
        # Only the physical motor-adjustment boundary is replaced.
        self.qgl.z_helper.adjust_steppers = self.adjust
        self.objects["quad_gantry_level"] = self.qgl
        # Exercise the real AVP command without SQLite or config startup.
        self.avp = AVP.__new__(AVP)
        self.avp.printer, self.avp.gcode = self, self.gcode
        self.avp.toolhead = self.toolhead
        self.avp.level_retries, self.avp.level_tolerance = 10, .01
        self.avp.points = [(50., 50., 0.)]

    def lookup_object(self, name, default=None):
        return self.objects.get(name, default)

    def register_event_handler(self, name, callback):
        self.handlers.setdefault(name, []).append(callback)

    def send_event(self, name, *args):
        for callback in self.handlers.get(name, []):
            callback(*args)

    def get_reactor(self):
        return Mock(monotonic=lambda: 0.)

    def command(self, name, line="", params=None):
        return gcode.GCodeCommand(self.gcode, name, line, params or {}, False)

    def dispatch(self, script):
        self.scripts.append(script)
        name, *params = script.split()
        self.commands[name](self.command(name, script, dict(p.split("=") for p in params)))

    def run(self):
        # Klipper's top-level dispatch emits this on command failure; reproduce
        # that boundary so native SampleAveragingHelper owns session cleanup.
        try:
            self.avp.cmd_AVP_LEVEL(self.command("AVP_LEVEL"))
        except gcode.CommandError:
            self.send_event("gcode:command_error")
            raise

    def move(self, values, speed):
        for i, value in enumerate(values):
            if value is not None:
                self.position[i] = value
        self.moves.append((tuple(self.position), speed))

    def run_probe(self, cmd):
        if len(self.touches) == self.fault_at:
            self.moves_at_fault = len(self.moves)
            raise gcode.CommandError("Timeout during endstop homing")
        xy = tuple(self.position[:2])
        z = self.rounds[self.round_index][CORNERS.index(xy)]
        z += self.noise.get(len(self.touches), 0.)
        self.position[2] = z - 1.372
        self.result = manual_probe.create_probe_result(self.position, (0., 0., -1.372))
        self.touches.append((self.round_index, xy, self.params.get_probe_params(cmd)))

    def pull_probed_results(self):
        return [self.result]

    def end_probe_session(self):
        self.ends += 1

    def adjust(self, adjustments, speed):
        self.adjustments.append(tuple(adjustments))
        self.round_index += 1


class PinnedQGLTests(unittest.TestCase):
    def test_four_single_touches_then_native_three_sample_refinement(self):
        rig = Rig([[0., .2, .3, .1], [0., .003, .006, .002]])
        rig.run()
        self.assertEqual(len(rig.touches), 16)
        self.assertEqual([t[1] for t in rig.touches[:4]], CORNERS)
        self.assertEqual([t[1] for t in rig.touches[4:]],
                         [xy for xy in CORNERS for _ in range(3)])
        self.assertEqual([t[2]["samples"] for t in rig.touches], [1]*4 + [3]*12)
        self.assertTrue(all(t[2]["probe_speed"] == 2.5 for t in rig.touches))
        self.assertTrue(all(t[2]["samples_tolerance"] == .01 for t in rig.touches))
        self.assertTrue(all(t[2]["samples_tolerance_retries"] == 10
                            for t in rig.touches))
        self.assertEqual(len(rig.adjustments), 2)
        self.assertTrue(any(abs(a) > .01 for a in rig.adjustments[0]))
        self.assertAlmostEqual(sum(rig.adjustments[0]), 0.)
        self.assertTrue(rig.qgl.get_status(0)["applied"])
        self.assertEqual(rig.ends, 2)
        self.assertIsNone(rig.sampling.hw_probe_session)
        self.assertEqual(rig.avp.points, [])

    def test_refinement_reprobes_all_corners_until_tolerance(self):
        rig = Rig([[0., .2, .3, .1], [0., .02, .02, 0.], [0., .005, .005, 0.]])
        rig.run()
        self.assertEqual(len(rig.touches), 28)
        self.assertEqual(len(rig.adjustments), 3)
        self.assertTrue(any("Retries: 1/10" in m for m in rig.messages))

    def test_native_sample_retry_preserved(self):
        rig = Rig([[0.]*4, [0.]*4], noise={5: .02})
        rig.run()
        self.assertEqual(len(rig.touches), 18)
        self.assertTrue(any("Probe samples exceed tolerance. Retrying" in m
                            for m in rig.messages))

    def test_timeout_aborts_either_pass_and_cleans_session(self):
        for touch in (0, 2, 4, 7):
            with self.subTest(touch=touch):
                rig = Rig([[0.]*4, [0.]*4], fault_at=touch)
                with self.assertRaisesRegex(gcode.CommandError, "Timeout"):
                    rig.run()
                self.assertEqual(len(rig.touches), touch)
                self.assertEqual(len(rig.scripts), 1 if touch < 4 else 2)
                self.assertEqual(len(rig.adjustments), 0 if touch < 4 else 1)
                self.assertEqual(len(rig.moves), rig.moves_at_fault)
                self.assertIsNone(rig.sampling.hw_probe_session)
                self.assertFalse(rig.qgl.get_status(0)["applied"])
                self.assertNotIn("AVP native gantry leveling completed", rig.messages)

    def test_excessive_coarse_adjustment_prevents_refinement(self):
        rig = Rig([[0., 20., 20., 0.]])
        with self.assertRaisesRegex(gcode.CommandError, "max_adjust"):
            rig.run()
        self.assertEqual(len(rig.touches), 4)
        self.assertEqual(rig.adjustments, [])
        self.assertEqual(len(rig.scripts), 1)
        self.assertIsNone(rig.sampling.hw_probe_session)

    def test_refinement_retry_exhaustion_is_not_success(self):
        rig = Rig([[0.]*4, [0., .02, .02, 0.], [0., .02, .02, 0.]])
        rig.avp.level_retries = 1
        with self.assertRaisesRegex(gcode.CommandError, "Too many retries"):
            rig.run()
        self.assertEqual(len(rig.touches), 28)
        self.assertFalse(rig.qgl.get_status(0)["applied"])
        self.assertIsNone(rig.sampling.hw_probe_session)
        self.assertNotIn("AVP native gantry leveling completed", rig.messages)

    def test_unhomed_rejected_before_native_commands(self):
        rig = Rig([])
        rig.homed = "xy"
        with self.assertRaisesRegex(gcode.CommandError, "Home XYZ"):
            rig.run()
        self.assertEqual(rig.scripts, [])
        self.assertEqual(rig.moves, [])
        self.assertEqual(rig.touches, [])

    def test_native_sample_retry_exhaustion_aborts_refinement(self):
        rig = Rig([[0.]*4, [0.]*4], noise={5: .02, 7: .02})
        rig.params.samples_retries = 1
        with self.assertRaisesRegex(gcode.CommandError, "samples_tolerance"):
            rig.run()
        self.assertEqual(len(rig.touches), 8)
        self.assertEqual(len(rig.adjustments), 1)
        self.assertIsNone(rig.sampling.hw_probe_session)
        self.assertFalse(rig.qgl.get_status(0)["applied"])

    def test_increasing_refinement_error_aborts(self):
        rig = Rig([[0.]*4, [0., .02, .02, 0.],
                   [0., .03, .03, 0.], [0., .04, .04, 0.]])
        with self.assertRaisesRegex(gcode.CommandError, "is increasing"):
            rig.run()
        self.assertEqual(len(rig.touches), 40)
        self.assertIsNone(rig.sampling.hw_probe_session)
        self.assertFalse(rig.qgl.get_status(0)["applied"])

    def test_coarse_motor_adjustment_failure_stops_sequence(self):
        rig = Rig([[0., .2, .3, .1]])
        def fail_adjust(adjustments, speed):
            raise gcode.CommandError("Simulated motor adjustment failure")
        rig.qgl.z_helper.adjust_steppers = fail_adjust
        with self.assertRaisesRegex(gcode.CommandError, "motor adjustment"):
            rig.run()
        self.assertEqual(len(rig.scripts), 1)
        self.assertEqual(len(rig.touches), 4)
        self.assertIsNone(rig.sampling.hw_probe_session)
        self.assertFalse(rig.qgl.get_status(0)["applied"])


if __name__ == "__main__":
    unittest.main()
