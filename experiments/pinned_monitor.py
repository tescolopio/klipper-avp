"""Explicit endstop outcome experiment for the pinned Klipper HomingMove.

Not a live probe backend: no deployment, frame lifecycle or Z-stepper validation.
Only imported by offline tests. Never substitutes a manual move for monitoring.
"""
import math
from .variable_speed import ApproachError, MoveResult, Outcome


class ObservedEndstop:
    def __init__(self, endstop, error):
        self.endstop = endstop
        self.error = error
        self.trigger_time = None

    def __getattr__(self, name):
        return getattr(self.endstop, name)

    def home_wait(self, end_time):
        value = self.endstop.home_wait(end_time)
        if (type(value) not in (float, int) or not math.isfinite(value) or value < 0):
            raise self.error("Invalid endstop completion")
        self.trigger_time = value
        return value


def observe_descent(printer, endstop, target_z, speed, homing_move_factory,
                    endpoint_tolerance):
    """Test seam using native homing with an explicit, observed home_wait result.

    A positive trigger timestamp means CONTACT even at the exact endpoint.
    Missing monitoring or ambiguous completion fails closed. Hardware eligibility,
    temperature, lifecycle and validated stopping behavior are NOT provided here.
    """
    toolhead = printer.lookup_object("toolhead")
    start = list(toolhead.get_position())
    if (not all(math.isfinite(v) for v in (target_z, speed, endpoint_tolerance, start[2]))
            or speed <= 0 or endpoint_tolerance <= 0 or target_z >= start[2]):
        raise ApproachError("Invalid monitored descent")
    if not endstop.get_steppers():
        raise ApproachError("No steppers attached to monitored endstop")
    if endstop.query_endstop(toolhead.get_last_move_time()):
        raise ApproachError("Probe already triggered")
    observed = ObservedEndstop(endstop, printer.command_error)
    move = homing_move_factory(printer, [(observed, "probe")], toolhead)
    target = list(start)
    target[2] = target_z
    move.homing_move(target, speed, probe_pos=True, check_triggered=False)
    halt_z = toolhead.get_position()[2]
    if observed.trigger_time is None or not math.isfinite(halt_z):
        raise ApproachError("No valid monitored completion")
    if observed.trigger_time > 0:
        return MoveResult(Outcome.CONTACT, halt_z)
    if abs(halt_z - target_z) > endpoint_tolerance:
        raise ApproachError("Untriggered move did not reach target")
    return MoveResult(Outcome.CLEAR, target_z)
