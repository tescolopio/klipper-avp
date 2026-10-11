import math
import unittest
from experiments.variable_speed import (
    ApproachError, CornerEstimate, MoveResult, Outcome, PreparedApproach,
    plan_descent,
)


PLANNING = dict(start_z=10., clearance=2., approach_speed=5., probe_speed=2.5,
                deceleration=100., latency=.01, z_min=-5., z_max=500.)
ESTIMATE = CornerEstimate(50., 50., -1.372, .1, 1)


class Backend:
    def __init__(self):
        self.calls = []
        self.z = 10.
        self.triggered = False
        self.outcome = Outcome.CLEAR
        self.fault = None
        self.hook = lambda name: None
        self.halt_error = 0.

    def call(self, name):
        self.calls.append(name)
        self.hook(name)
        if self.fault == name:
            raise ApproachError(name + " failed")

    def activate(self):
        self.call("activate")

    def current_z(self):
        return self.z

    def is_triggered(self):
        self.call("query")
        return self.triggered

    def monitored_move(self, z, speed):
        self.call("monitored")
        self.z = z + self.halt_error
        return MoveResult(self.outcome, self.z)

    def native_probe(self, speed):
        self.call("native")
        return "native averaged measurement"

    def finish(self):
        self.call("finish")


class PlanningTests(unittest.TestCase):
    def plan(self, **updates):
        args = dict(PLANNING, predicted_trigger_z=-1.372, uncertainty=.1)
        args.update(updates)
        return plan_descent(**args)

    def test_two_speed_geometry_includes_stopping_and_uncertainty(self):
        plan = self.plan()
        self.assertAlmostEqual(plan.stopping_reserve, .175)
        self.assertAlmostEqual(plan.approach_z, .903)
        self.assertAlmostEqual(plan.ideal_saving, 1.8194)

    def test_more_uncertainty_or_latency_moves_transition_up(self):
        base = self.plan()
        for change in (dict(uncertainty=1.), dict(latency=.1), dict(deceleration=10.)):
            plan = self.plan(**change)
            self.assertGreater(plan.approach_z, base.approach_z)
            self.assertLess(plan.ideal_saving, base.ideal_saving)

    def test_no_fast_region_or_no_speed_benefit_uses_native_only(self):
        for change in (dict(uncertainty=20.), dict(start_z=.5),
                       dict(approach_speed=2.5), dict(approach_speed=1.)):
            with self.subTest(change=change):
                plan = self.plan(**change)
                self.assertIsNone(plan.approach_z)
                self.assertEqual(plan.ideal_saving, 0.)

    def test_invalid_planning_inputs(self):
        for key in dict(PLANNING, predicted_trigger_z=0., uncertainty=.1):
            for value in (math.nan, math.inf, -math.inf):
                with self.subTest(key=key, value=value):
                    with self.assertRaises(ValueError):
                        self.plan(**{key: value})
        for change in (dict(uncertainty=-1.), dict(clearance=0.),
                       dict(deceleration=0.), dict(latency=-1.),
                       dict(probe_speed=0.), dict(approach_speed=0.),
                       dict(start_z=501.), dict(predicted_trigger_z=-6.),
                       dict(z_min=501.)):
            with self.assertRaises(ValueError):
                self.plan(**change)

    def test_transform_requires_new_frame_and_adds_uncertainty(self):
        new = ESTIMATE.after_adjustment(frame=2, trigger_shift=.2, shift_uncertainty=.05)
        self.assertAlmostEqual(new.trigger_z, -1.172)
        self.assertAlmostEqual(new.uncertainty, .15)
        self.assertEqual(new.frame, 2)
        self.assertEqual(ESTIMATE.frame, 1)
        for frame, shift, uncertainty in ((1, 0., 0.), (2, math.nan, 0.), (2, 0., -.1)):
            with self.assertRaises(ValueError):
                ESTIMATE.after_adjustment(frame=frame, trigger_shift=shift,
                                          shift_uncertainty=uncertainty)

    def test_overflow_and_invalid_estimate_rejected(self):
        for change in (dict(approach_speed=1.e308), dict(probe_speed=1.e-320)):
            with self.assertRaises(ValueError):
                self.plan(**change)
        for frame in (math.nan, -1, True):
            with self.assertRaises(ValueError):
                CornerEstimate(50., 50., 0., .1, frame)


class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.control = PreparedApproach()
        self.backend = Backend()
        self.token = self.prepare()

    def prepare(self, estimate=ESTIMATE, **updates):
        planning = dict(PLANNING, **updates)
        return self.control.prepare(frame=estimate.frame, estimate=estimate,
                                    xy=(50., 50.), **planning)

    def run_approach(self, token=None):
        return self.control.execute(self.token if token is None else token,
                                    self.backend, frame=1, xy=(50., 50.))

    def test_monitored_then_native_only_after_clear_completion(self):
        self.assertEqual(self.run_approach(), "native averaged measurement")
        self.assertEqual(self.backend.calls,
                         ["activate", "query", "monitored", "query", "native", "finish"])
        with self.assertRaises(ApproachError):
            self.run_approach()
        self.assertEqual(self.backend.calls.count("native"), 1)

    def test_native_only_fallback(self):
        self.token = self.prepare(estimate=CornerEstimate(50., 50., -1.372, 20., 1))
        self.run_approach()
        self.assertNotIn("monitored", self.backend.calls)
        self.assertIn("native", self.backend.calls)

    def test_contact_even_at_endpoint_never_starts_native_probe(self):
        self.backend.outcome = Outcome.CONTACT
        with self.assertRaisesRegex(ApproachError, "Unexpected contact"):
            self.run_approach()
        self.assertNotIn("native", self.backend.calls)
        self.assertEqual(self.backend.calls[-1], "finish")
        with self.assertRaises(ApproachError):
            self.run_approach()

    def test_stuck_probe_stops_before_descent(self):
        self.backend.triggered = True
        with self.assertRaisesRegex(ApproachError, "already triggered"):
            self.run_approach()
        self.assertEqual(self.backend.calls, ["activate", "query", "finish"])

    def test_no_further_stage_after_backend_failure(self):
        for stage in ("activate", "query", "monitored", "native", "finish"):
            with self.subTest(stage=stage):
                self.setUp()
                self.backend.fault = stage
                with self.assertRaises(ApproachError):
                    self.run_approach()
                self.assertIsNone(self.control.pending)
                self.assertEqual(self.backend.calls[-1], "finish")
                if stage in ("activate", "query", "monitored"):
                    self.assertNotIn("native", self.backend.calls)

    def test_relevant_changes_invalidate_before_activation(self):
        # Policy coverage, not proof that Klipper event hooks are wired.
        for reason in ("homing", "qgl", "z_tilt", "motor_off", "individual_motor_off",
                       "restart", "disconnect", "offset", "plate_reset", "temperature"):
            with self.subTest(reason=reason):
                self.setUp()
                self.control.invalidate(reason)
                with self.assertRaises(ApproachError):
                    self.run_approach()
                self.assertEqual(self.backend.calls, [])

    def test_old_estimate_cannot_rearm_after_invalidation_or_newer_frame(self):
        self.control.invalidate("qgl")
        with self.assertRaisesRegex(ApproachError, "Stale"):
            self.prepare()
        estimate = ESTIMATE.after_adjustment(frame=2, trigger_shift=.1, shift_uncertainty=.1)
        self.prepare(estimate=estimate)
        with self.assertRaisesRegex(ApproachError, "Stale"):
            self.prepare()

    def test_invalidation_during_cleanup_cannot_return_measurement(self):
        self.backend.hook = lambda name: (
            self.control.invalidate("restart") if name == "finish" else None)
        with self.assertRaisesRegex(ApproachError, "invalidated"):
            self.run_approach()

    def test_mid_approach_invalidation_prevents_next_stage(self):
        for stage in ("activate", "monitored", "native"):
            with self.subTest(stage=stage):
                self.setUp()
                self.backend.hook = lambda name: (
                    self.control.invalidate("frame changed") if name == stage else None)
                with self.assertRaisesRegex(ApproachError, "invalidated"):
                    self.run_approach()
                self.assertEqual(self.backend.calls[-1], "finish")
                if stage != "native":
                    self.assertNotIn("native", self.backend.calls)

    def test_wrong_frame_or_corner_rejected(self):
        for frame, xy in ((2, (50., 50.)), (1, (450., 50.))):
            self.token = self.prepare()
            with self.assertRaises(ApproachError):
                self.control.execute(self.token, self.backend, frame=frame, xy=xy)
        self.assertEqual(self.backend.calls, [])
        with self.assertRaisesRegex(ApproachError, "Stale"):
            self.control.prepare(frame=2, estimate=ESTIMATE, xy=(50., 50.), **PLANNING)

    def test_position_changes_and_wrong_completion_fail_closed(self):
        for z in (9., math.nan):
            self.setUp()
            self.backend.z = z
            with self.assertRaisesRegex(ApproachError, "Position changed"):
                self.run_approach()
            self.assertNotIn("monitored", self.backend.calls)
        for error in (.01, math.nan):
            self.setUp()
            self.backend.halt_error = error
            with self.assertRaisesRegex(ApproachError, "endpoint"):
                self.run_approach()
            self.assertNotIn("native", self.backend.calls)

    def test_unknown_outcome_and_postmove_trigger_fail_closed(self):
        self.backend.outcome = None
        with self.assertRaises(ApproachError):
            self.run_approach()
        self.assertNotIn("native", self.backend.calls)
        self.setUp()
        self.backend.hook = lambda name: (
            setattr(self.backend, "triggered", True) if name == "monitored" else None)
        with self.assertRaisesRegex(ApproachError, "triggered after"):
            self.run_approach()
        self.assertNotIn("native", self.backend.calls)


if __name__ == "__main__":
    unittest.main()
