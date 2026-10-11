"""Real pinned HomingMove control flow with simulated MCU/stepper boundaries."""
import math
import unittest
from unittest.mock import Mock
# Reuse the mandatory revision/cleanliness check before loading upstream code.
from test_pinned_qgl import gcode
from extras import homing
from experiments.pinned_monitor import observe_descent
from experiments.variable_speed import (
    ApproachError, CornerEstimate, Outcome, PreparedApproach,
)


class Stepper:
    def __init__(self, rig, name):
        self.rig, self.name = rig, name

    def get_name(self):
        return self.name

    def get_step_dist(self):
        return .001

    def calc_position_from_coord(self, position):
        return position[2]

    def get_commanded_position(self):
        return self.rig.position[2]

    def get_mcu_position(self, commanded=None):
        z = self.rig.position[2] if commanded is None else commanded
        return round(z / .001)

    def mcu_to_commanded_position(self, position):
        return position * .001

    def get_past_mcu_position(self, time):
        z = (self.rig.contact_z if self.rig.timestamp > 0
             and time == self.rig.timestamp else self.rig.position[2])
        return round(z / .001)


class Endstop:
    def __init__(self, rig):
        self.rig = rig

    def get_steppers(self):
        return self.rig.steppers

    def query_endstop(self, time):
        return self.rig.initially_triggered

    def home_start(self, print_time, sample_time, sample_count, rest_time,
                   triggered=True):
        self.rig.monitor_started = True
        self.rig.samples = sample_count
        if not triggered:
            raise AssertionError("Must watch contact, not release")
        return object()

    def home_wait(self, time):
        if self.rig.timeout:
            raise gcode.CommandError("Simulated CAN timeout")
        return self.rig.timestamp


class Rig:
    command_error = gcode.CommandError

    def __init__(self, contact_z=None):
        self.position = [50., 50., 10., 0.]
        self.contact_z = contact_z
        self.timestamp = 0. if contact_z is None else 1.5
        self.timeout, self.initially_triggered = False, False
        self.monitor_started, self.moves = False, []
        self.steppers = [Stepper(self, name) for name in ("z", "z1", "z2", "z3")]
        self.endstop = Endstop(self)
        self.toolhead = Mock()
        self.toolhead.get_position.side_effect = lambda: list(self.position)
        self.toolhead.get_last_move_time.return_value = 2.
        self.toolhead.get_kinematics.return_value = self
        self.toolhead.drip_move.side_effect = self.drip_move
        self.toolhead.set_position.side_effect = self.set_position
        self.toolhead.manual_move.side_effect = AssertionError("Unmonitored move")
        self.events = []

    def get_steppers(self):
        return self.steppers

    def calc_position(self, positions):
        return [50., 50., sum(positions.values()) / len(positions)]

    def lookup_object(self, name):
        if name != "toolhead":
            raise AssertionError(name)
        return self.toolhead

    def send_event(self, name, *args):
        self.events.append(name)

    def set_position(self, position):
        self.position[:] = position

    def drip_move(self, target, speed, completion):
        if not self.monitor_started:
            raise AssertionError("Descent without monitoring")
        self.moves.append((tuple(target), speed))
        self.position[:] = target
        if self.contact_z is not None:
            self.position[2] = self.contact_z - .01  # synthetic halt overtravel

    def run(self, target=3.):
        return observe_descent(self, self.endstop, target, 5., homing.HomingMove, .001)


class PinnedMonitorTests(unittest.TestCase):
    def test_no_contact_reaches_endpoint_with_monitoring(self):
        rig = Rig()
        result = rig.run()
        self.assertIs(result.outcome, Outcome.CLEAR)
        self.assertEqual(result.halt_z, 3.)
        self.assertTrue(rig.monitor_started)
        self.assertEqual(rig.samples, 4)
        self.assertEqual(rig.events,
                         ["homing:homing_move_begin", "homing:homing_move_end"])
        rig.toolhead.manual_move.assert_not_called()

    def test_early_and_endpoint_contact_are_explicit(self):
        for z in (8., 3.):
            with self.subTest(contact_z=z):
                rig = Rig(z)
                result = rig.run()
                self.assertIs(result.outcome, Outcome.CONTACT)
                self.assertAlmostEqual(result.halt_z, z - .01)

    def test_exact_endpoint_contact_is_not_clear_even_without_overtravel(self):
        rig = Rig(3.)
        def exact_stop(target, speed, completion):
            rig.position[:] = target
        rig.toolhead.drip_move.side_effect = exact_stop
        self.assertIs(rig.run().outcome, Outcome.CONTACT)
        self.assertEqual(rig.position[2], 3.)

    def test_timeout_propagates_after_native_completion_cleanup(self):
        rig = Rig()
        rig.timeout = True
        with self.assertRaisesRegex(gcode.CommandError, "CAN timeout"):
            rig.run()
        self.assertIn("homing:homing_move_end", rig.events)
        self.assertEqual(len(rig.moves), 1)

    def test_invalid_completion_fails_through_native_error_path(self):
        for timestamp in (math.nan, -1., None, True):
            rig = Rig()
            rig.timestamp = timestamp
            # Position lookup must stay valid while the invalid timestamp is
            # rejected; simulated steppers don't need a historical trigger.
            for stepper in rig.steppers:
                stepper.get_past_mcu_position = lambda time: 3000
            with self.assertRaisesRegex(gcode.CommandError, "Invalid endstop"):
                rig.run()
            self.assertIn("homing:homing_move_end", rig.events)

    def test_missing_steppers_or_initial_contact_prevents_motion(self):
        for mode in ("no_steppers", "stuck"):
            rig = Rig()
            if mode == "no_steppers":
                rig.steppers = []
            else:
                rig.initially_triggered = True
            with self.assertRaises(ApproachError):
                rig.run()
            self.assertEqual(rig.moves, [])

    def test_untriggered_short_move_is_not_success(self):
        rig = Rig()
        rig.toolhead.drip_move.side_effect = lambda *args: rig.position.__setitem__(2, 4.)
        with self.assertRaisesRegex(ApproachError, "did not reach"):
            rig.run()

    def test_native_move_error_propagates(self):
        rig = Rig()
        rig.toolhead.drip_move.side_effect = gcode.CommandError("Move failed")
        with self.assertRaisesRegex(gcode.CommandError, "Move failed"):
            rig.run()
        self.assertIn("homing:homing_move_end", rig.events)

    def test_controller_never_starts_native_sampling_after_contact(self):
        # Connect the policy to real HomingMove, keeping hardware simulated.
        rig = Rig(8.)
        control = PreparedApproach()
        token = control.prepare(
            frame=1, estimate=CornerEstimate(50., 50., 0., .1, 1), xy=(50., 50.),
            start_z=10., clearance=2., approach_speed=5., probe_speed=2.5,
            deceleration=100., latency=.01, z_min=-5., z_max=500.)
        backend = Mock()
        backend.current_z.return_value = 10.
        backend.is_triggered.return_value = False
        backend.monitored_move.side_effect = lambda z, speed: observe_descent(
            rig, rig.endstop, z, speed, homing.HomingMove, .001)
        with self.assertRaisesRegex(ApproachError, "Unexpected contact"):
            control.execute(token, backend, frame=1, xy=(50., 50.))
        backend.native_probe.assert_not_called()
        backend.finish.assert_called_once()


if __name__ == "__main__":
    unittest.main()
