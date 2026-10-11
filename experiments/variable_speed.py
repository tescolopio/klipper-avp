"""Offline two-speed approach policy. No printer registration or motion API.

A backend must supply activation, monitored motion, native samples and cleanup.
The interface is intentionally not connected to Klipper on a real printer.
"""
from dataclasses import dataclass
from enum import Enum
import math
from typing import Optional


class ApproachError(RuntimeError):
    pass


@dataclass(frozen=True)
class Plan:
    start_z: float
    approach_z: Optional[float]
    approach_speed: float
    probe_speed: float
    stopping_reserve: float
    uncertainty: float
    ideal_saving: float


def plan_descent(*, start_z, predicted_trigger_z, uncertainty, clearance,
                 approach_speed, probe_speed, deceleration, latency,
                 z_min, z_max):
    """Hypothetical geometry, not a physical bound or validated stopping model.

    Inputs are current-frame toolhead trigger Z, not bed_z or historical maxima.
    Deceleration/latency are externally supplied assumptions for simulation.
    """
    values = (start_z, predicted_trigger_z, uncertainty, clearance, approach_speed,
              probe_speed, deceleration, latency, z_min, z_max)
    if not all(math.isfinite(v) for v in values):
        raise ValueError("All planning inputs must be finite")
    if (uncertainty < 0 or clearance <= 0 or approach_speed <= 0
            or probe_speed <= 0 or deceleration <= 0 or latency < 0):
        raise ValueError("Invalid margin, speed, deceleration or latency")
    if z_min >= z_max or not z_min <= start_z <= z_max:
        raise ValueError("Invalid start or Z limits")
    if not z_min <= predicted_trigger_z <= z_max:
        raise ValueError("Predicted trigger outside Z limits")
    reserve = (approach_speed * latency
               + .5 * approach_speed * (approach_speed / deceleration))
    target = predicted_trigger_z + uncertainty + clearance + reserve
    if not math.isfinite(target) or not math.isfinite(reserve):
        raise ValueError("Planning arithmetic overflow")
    # Uncertain geometry must remove acceleration, never reduce a margin.
    fast = approach_speed > probe_speed and z_min <= target < start_z
    saving = ((start_z - target) * (1 / probe_speed - 1 / approach_speed)
              if fast else 0.)
    if not math.isfinite(saving):
        raise ValueError("Timing arithmetic overflow")
    return Plan(start_z, target if fast else None, approach_speed, probe_speed,
                reserve, uncertainty, saving)


@dataclass(frozen=True)
class CornerEstimate:
    x: float
    y: float
    trigger_z: float
    uncertainty: float
    frame: int

    def __post_init__(self):
        if (not all(math.isfinite(v) for v in
                    (self.x, self.y, self.trigger_z, self.uncertainty))
                or self.uncertainty < 0 or type(self.frame) is not int or self.frame < 0):
            raise ValueError("Invalid corner estimate")

    def after_adjustment(self, *, frame, trigger_shift, shift_uncertainty):
        """Explicit modeled transform, never automatic reuse across QGL.

        Caller must obtain trigger_shift in toolhead coordinates (including any
        coordinate reset). Raw individual motor adjustments are NOT that shift.
        """
        if frame <= self.frame:
            raise ValueError("Adjustment requires a newer coordinate frame")
        values = (self.x, self.y, self.trigger_z, self.uncertainty,
                  trigger_shift, shift_uncertainty)
        if not all(math.isfinite(v) for v in values) or min(
                self.uncertainty, shift_uncertainty) < 0:
            raise ValueError("Invalid corner transform")
        z = self.trigger_z + trigger_shift
        uncertainty = self.uncertainty + shift_uncertainty
        if not math.isfinite(z) or not math.isfinite(uncertainty):
            raise ValueError("Corner transform overflow")
        return CornerEstimate(self.x, self.y, z, uncertainty, frame)


class Outcome(Enum):
    CLEAR = "endpoint_without_contact"
    CONTACT = "contact"


@dataclass(frozen=True)
class MoveResult:
    outcome: Outcome
    halt_z: float


class PreparedApproach:
    """One-use offline controller; integration must wire all invalidation paths.

    It intentionally has no event subscriptions or live-printer entry point.
    An in-flight invalidation stops later operations, not an already running move;
    MCU monitoring and shutdown remain the backend's responsibility.
    """
    def __init__(self):
        self.epoch = 0
        self.minimum_frame = 0
        self.latest_frame = -1
        self.pending = None
        self.last_invalidation = "not prepared"

    def invalidate(self, reason):
        self.epoch += 1
        self.minimum_frame = max(self.minimum_frame, self.latest_frame + 1)
        self.pending = None
        self.last_invalidation = reason

    def prepare(self, *, frame, estimate, xy, **planning):
        self.pending = None
        if (not isinstance(estimate, CornerEstimate) or frame != estimate.frame
                or type(frame) is not int or frame < max(self.minimum_frame, self.latest_frame)):
            raise ApproachError("Stale or missing coordinate-frame estimate")
        if tuple(xy) != (estimate.x, estimate.y):
            raise ApproachError("Only the measured corner is eligible")
        plan = plan_descent(predicted_trigger_z=estimate.trigger_z,
                            uncertainty=estimate.uncertainty, **planning)
        self.latest_frame = frame
        # Each preparation permits one descent only. No persistence or re-arming.
        token = object()
        self.pending = (token, self.epoch, frame, tuple(xy), plan)
        return token

    def execute(self, token, backend, *, frame, xy):
        pending = self.pending
        self.pending = None  # consume before activation; failures cannot reuse it
        if (pending is None or pending[0] is not token or pending[1] != self.epoch
                or pending[2] != frame or pending[3] != tuple(xy)):
            raise ApproachError("Preparation missing, consumed or stale")
        _, epoch, _, _, plan = pending

        def check_epoch():
            if self.epoch != epoch:
                raise ApproachError("Preparation invalidated during approach")

        succeeded = False
        try:
            backend.activate()  # Includes probe deployment and thermal checks.
            check_epoch()
            z = backend.current_z()
            if not math.isfinite(z) or abs(z - plan.start_z) > 1.e-6:
                raise ApproachError("Position changed after preparation")
            if backend.is_triggered():
                raise ApproachError("Probe already triggered")
            check_epoch()
            if plan.approach_z is not None:
                result = backend.monitored_move(plan.approach_z, plan.approach_speed)
                check_epoch()
                if not isinstance(result, MoveResult) or result.outcome is not Outcome.CLEAR:
                    raise ApproachError("Unexpected contact or unknown approach result")
                # Backend normalizes a reached target within verified step tolerance.
                if (not math.isfinite(result.halt_z)
                        or abs(result.halt_z - plan.approach_z) > 1.e-6):
                    raise ApproachError("Approach did not reach the planned endpoint")
                if backend.is_triggered():
                    raise ApproachError("Probe triggered after approach")
            check_epoch()
            # Native multi-sample processing remains a single backend operation.
            result = backend.native_probe(plan.probe_speed)
            check_epoch()
            succeeded = True
            return result
        except BaseException:
            self.invalidate("approach or native probing failed")
            raise
        finally:
            # No XY motion, automatic retract or retry on a fault in this policy.
            # Cleanup must be safe even if activation failed partway through.
            try:
                backend.finish()
                if succeeded:
                    check_epoch()
            except BaseException:
                self.invalidate("probe cleanup failed")
                raise
