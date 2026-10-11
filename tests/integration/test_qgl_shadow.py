"""Shadow instrumentation and native motor-transform contract tests."""
import json
import unittest
from unittest.mock import Mock
from test_pinned_qgl import Rig, gcode
from extras.z_tilt import ZAdjustHelper
from avp.klipper.qgl_shadow import corner_shift, SUPPORTED_VERSION


def setup_shadow(rig):
    rig.get_start_args = lambda: {"software_version": SUPPORTED_VERSION}
    rig.avp.probe = rig.objects["probe"]
    rig.avp.clearance, rig.avp.approach_speed = 2., 5.
    rig.avp.shadow_observer = None
    rig.register_event_handler("probe:update_results", rig.avp.observe_shadow_probe)


def run_shadow(rig, **parameters):
    params = dict(SHADOW="1", **parameters)
    try:
        rig.avp.cmd_AVP_LEVEL(rig.command("AVP_LEVEL", params=params))
    except gcode.CommandError:
        rig.send_event("gcode:command_error")
        raise


def report(rig):
    return json.loads(next(m.split(": ", 1)[1] for m in rig.messages
                          if m.startswith("AVP QGL shadow: ")))


class ShadowTests(unittest.TestCase):
    def test_identical_native_calls_motion_samples_and_adjustments(self):
        rounds = [[0., .2, .3, .1], [0., .02, .02, 0.], [0., .005, .005, 0.]]
        normal, shadow = Rig(rounds), Rig(rounds)
        setup_shadow(shadow)
        normal.run()
        original_finalize = shadow.qgl.probe_helper.finalize_callback
        original_adjust = shadow.qgl.z_helper.adjust_steppers
        run_shadow(shadow)
        self.assertEqual(normal.scripts, shadow.scripts)
        self.assertEqual(normal.moves, shadow.moves)
        self.assertEqual(normal.touches, shadow.touches)
        self.assertEqual(normal.adjustments, shadow.adjustments)
        self.assertEqual(normal.ends, shadow.ends)
        self.assertIs(shadow.qgl.probe_helper.finalize_callback, original_finalize)
        self.assertIs(shadow.qgl.z_helper.adjust_steppers, original_adjust)
        self.assertIsNone(shadow.avp.shadow_observer)
        data = report(shadow)
        self.assertTrue(data["completed"])
        self.assertFalse(data["motion_changed"])
        self.assertEqual(len(data["rounds"]), 3)
        self.assertEqual([p["raw_touch_count"] for p in data["rounds"][0]["points"]], [1]*4)
        self.assertEqual([p["raw_touch_count"] for p in data["rounds"][1]["points"]], [3]*4)
        self.assertIsNone(data["rounds"][0]["points"][0]["predicted_trigger_z"])
        self.assertIsNotNone(data["rounds"][1]["points"][0]["predicted_trigger_z"])

    def test_prediction_transform_correct_for_ideal_corner_readings(self):
        # First round plane has mean .15. Ideal adjustment removes its tilt.
        rig = Rig([[0., .2, .3, .1], [.15]*4])
        setup_shadow(rig)
        run_shadow(rig)
        for row in report(rig)["rounds"][1]["points"]:
            self.assertAlmostEqual(row["prediction_error"], 0.)
            self.assertAlmostEqual(row["predicted_trigger_z"], .15 - 1.372)
            self.assertAlmostEqual(row["minimum_observed_gap"], 2.6)

    def test_sample_retries_included_in_observed_gap(self):
        rig = Rig([[0.]*4, [0.]*4], noise={5: .02})
        setup_shadow(rig)
        run_shadow(rig)
        row = report(rig)["rounds"][1]["points"][0]
        self.assertEqual(row["raw_touch_count"], 5)
        self.assertAlmostEqual(row["raw_trigger_max"], -1.352)
        self.assertAlmostEqual(row["minimum_observed_gap"], 2.58)

    def test_failure_restores_callbacks_and_reports_partial_run(self):
        for fault in (0, 4, 7):
            rig = Rig([[0.]*4, [0.]*4], fault_at=fault)
            setup_shadow(rig)
            finalize = rig.qgl.probe_helper.finalize_callback
            adjust = rig.qgl.z_helper.adjust_steppers
            with self.assertRaisesRegex(gcode.CommandError, "Timeout"):
                run_shadow(rig)
            self.assertIs(rig.qgl.probe_helper.finalize_callback, finalize)
            self.assertIs(rig.qgl.z_helper.adjust_steppers, adjust)
            self.assertIsNone(rig.avp.shadow_observer)
            self.assertFalse(report(rig)["completed"])
            self.assertIsNone(rig.sampling.hw_probe_session)
            self.assertNotIn("AVP native gantry leveling completed", rig.messages)

    def test_unsupported_version_or_geometry_rejects_before_motion(self):
        rig = Rig([])
        setup_shadow(rig)
        rig.get_start_args = lambda: {"software_version": "unknown"}
        with self.assertRaisesRegex(gcode.CommandError, "requires QGL"):
            run_shadow(rig)
        self.assertEqual(rig.moves, [])
        self.assertEqual(rig.scripts, [])
        setup_shadow(rig)
        rig.qgl.gantry_corners = [(100., 100.), (500., 500.)]
        with self.assertRaisesRegex(gcode.CommandError, "inside increasing"):
            run_shadow(rig)
        self.assertEqual(rig.scripts, [])

    def test_dirty_pinned_version_allowed_and_nonfinite_parameters_rejected(self):
        rig = Rig([[0.]*4, [0.]*4])
        setup_shadow(rig)
        rig.get_start_args = lambda: {"software_version": SUPPORTED_VERSION + "-dirty"}
        with self.assertRaisesRegex(gcode.CommandError, "parameters"):
            run_shadow(rig, SHADOW_UNCERTAINTY="nan")
        self.assertEqual(rig.moves, [])
        run_shadow(rig)
        self.assertTrue(report(rig)["completed"])

    def test_new_shadow_command_cannot_reuse_previous_predictions(self):
        rig = Rig([[0.]*4 for _ in range(4)])
        setup_shadow(rig)
        run_shadow(rig)
        rig.messages.clear()
        run_shadow(rig)
        data = report(rig)
        self.assertEqual(len(data["rounds"]), 2)
        self.assertTrue(all(p["predicted_trigger_z"] is None
                            for p in data["rounds"][0]["points"]))
        self.assertIsNone(rig.avp.shadow_observer)

    def test_adjustment_failure_restores_hooks_without_refinement(self):
        rig = Rig([[0.]*4])
        setup_shadow(rig)
        finalize = rig.qgl.probe_helper.finalize_callback
        def fail(*args):
            raise gcode.CommandError("Motor error")
        rig.qgl.z_helper.adjust_steppers = fail
        with self.assertRaisesRegex(gcode.CommandError, "Motor error"):
            run_shadow(rig)
        self.assertIs(rig.qgl.z_helper.adjust_steppers, fail)
        self.assertIs(rig.qgl.probe_helper.finalize_callback, finalize)
        self.assertEqual(len(rig.scripts), 1)
        self.assertFalse(report(rig)["rounds"][0]["completed"])

    def test_temperature_targets_and_actuals_are_captured(self):
        rig = Rig([[0.]*4, [0.]*4])
        setup_shadow(rig)
        for name, temp, target in (("heater_bed", 99.9, 100.), ("extruder", 149.5, 150.)):
            heater = Mock()
            heater.get_status.return_value = dict(temperature=temp, target=target)
            rig.objects[name] = heater
        run_shadow(rig)
        temperatures = report(rig)["rounds"][1]["temperatures"]
        self.assertEqual(temperatures["heater_bed"], dict(temperature=99.9, target=100.))
        self.assertEqual(temperatures["extruder"], dict(temperature=149.5, target=150.))


class Motor:
    def __init__(self, name):
        self.name, self.physical, self.trapq = name, 0., None

    def get_name(self):
        return self.name

    def set_trapq(self, trapq):
        self.trapq = trapq


class TransformTests(unittest.TestCase):
    def test_actual_native_helper_matches_trigger_transform(self):
        for adjustments in ((.2, -.1, .05, -.15), (1., 1., 1., 1.),
                            (-1., -.4, -.7, -.2), (0., 0., 0., 0.)):
            with self.subTest(adjustments=adjustments):
                motors = [Motor(name) for name in ("z", "z1", "z2", "z3")]
                position = [250., 250., 10., 0.]
                trapq = object()
                toolhead = Mock()
                toolhead.get_position.side_effect = lambda: list(position)
                toolhead.get_trapq.return_value = trapq
                toolhead.set_position.side_effect = lambda p: position.__setitem__(slice(None), p)
                def move(p, speed):
                    dz = p[2] - position[2]
                    for motor in motors:
                        if motor.trapq is trapq:
                            motor.physical += dz
                    position[:] = p
                toolhead.move.side_effect = move
                printer = Mock()
                printer.lookup_object.side_effect = lambda name: toolhead if name == "toolhead" else Mock()
                helper = ZAdjustHelper.__new__(ZAdjustHelper)
                helper.printer, helper.z_steppers = printer, motors
                helper.adjust_steppers(adjustments, 2.5)
                for motor, adjustment in zip(motors, adjustments):
                    self.assertAlmostEqual(motor.physical, adjustment - min(adjustments))
                    self.assertIs(motor.trapq, trapq)
                self.assertAlmostEqual(position[2] - 10., -min(adjustments))
                for x, y in ((0., 0.), (0., 500.), (500., 500.), (500., 0.), (150., 350.)):
                    u, v = x / 500., y / 500.
                    weights = ((1-u)*(1-v), (1-u)*v, u*v, u*(1-v))
                    physical = sum(w*m.physical for w, m in zip(weights, motors))
                    observed_shift = position[2] - 10. - physical
                    self.assertAlmostEqual(corner_shift(x, y, [(0., 0.), (500., 500.)],
                                                       adjustments), observed_shift)


if __name__ == "__main__":
    unittest.main()
