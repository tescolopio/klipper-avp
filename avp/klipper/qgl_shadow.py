"""Observation-only QGL instrumentation for Klipper 461c4e37.

Temporarily observes native callbacks; never supplies movement or probe commands.
Predictions are diagnostic hypotheses, not motion authorizations.
"""
import math

SUPPORTED_VERSION = "v0.13.0-786-g461c4e37"


def corner_shift(x, y, gantry_corners, adjustments):
    """Hypothesized trigger-Z change after native ZAdjustHelper.

    The helper moves actuator i by a_i - min(a), then changes toolhead Z by
    -min(a). Thus trigger-coordinate change is -interpolate(a), assuming ideal
    bilinear gantry motion. This is NOT an assertion of physical bed clearance.
    """
    (x0, y0), (x1, y1) = gantry_corners
    if len(adjustments) != 4 or not all(math.isfinite(v) for v in
            (x, y, x0, y0, x1, y1, *adjustments)):
        raise ValueError("Invalid QGL transform inputs")
    if x0 >= x1 or y0 >= y1 or not x0 <= x <= x1 or not y0 <= y <= y1:
        raise ValueError("Shadow requires points inside increasing gantry corners")
    u, v = (x - x0) / (x1 - x0), (y - y0) / (y1 - y0)
    weights = ((1-u)*(1-v), (1-u)*v, u*v, u*(1-v))
    shift = -sum(w*a for w, a in zip(weights, adjustments))
    if not math.isfinite(shift):
        raise ValueError("QGL transform overflow")
    return shift


class QGLShadow:
    def __init__(self, avp, qgl, uncertainty, stop_reserve):
        self.avp, self.qgl = avp, qgl
        self.uncertainty, self.stop_reserve = uncertainty, stop_reserve
        self.predictions = None
        self.raw = []
        self.rounds = []
        self.active_positions = None
        self.errors = []
        self.phase = "coarse"
        self.completed = False
        self.started = avp.printer.get_reactor().monotonic()
        self.in_round = None
        self.original_finalize = qgl.probe_helper.finalize_callback
        self.original_adjust = qgl.z_helper.adjust_steppers
        self.gantry = tuple(tuple(p) for p in qgl.gantry_corners)
        # Preflight happens before any native command or callback replacement.
        if len(self.gantry) != 2 or len(qgl.probe_helper.probe_points) != 4:
            raise ValueError("Shadow requires two gantry corners and four QGL points")
        for xy in qgl.probe_helper.probe_points:
            corner_shift(*xy, self.gantry, (0., 0., 0., 0.))
        self.probe_speed = avp.probe.get_probe_params()['probe_speed']
        self.travel_z = qgl.probe_helper.default_horizontal_move_z
        if (not all(math.isfinite(v) for v in
                    (uncertainty, stop_reserve, avp.clearance, avp.approach_speed,
                     self.probe_speed, self.travel_z))
                or min(uncertainty, stop_reserve) < 0
                or min(avp.clearance, avp.approach_speed, self.probe_speed) <= 0):
            raise ValueError("Invalid shadow planning parameters")

    def __enter__(self):
        self.qgl.probe_helper.finalize_callback = self.finalize
        self.qgl.z_helper.adjust_steppers = self.adjust
        self.avp.shadow_observer = self
        return self

    def __exit__(self, *exception):
        self.avp.shadow_observer = None
        self.qgl.probe_helper.finalize_callback = self.original_finalize
        self.qgl.z_helper.adjust_steppers = self.original_adjust
        # Estimates have no lifetime outside this command, including on failure.
        self.predictions = None

    def temperatures(self):
        eventtime = self.avp.printer.get_reactor().monotonic()
        result = {}
        for name in ("heater_bed", "extruder"):
            heater = self.avp.printer.lookup_object(name, None)
            if heater is not None:
                status = heater.get_status(eventtime)
                result[name] = {key: status.get(key) for key in ("temperature", "target")}
        return result

    def observe(self, results):
        for pos in results:
            values = (pos.test_x, pos.test_y, pos.test_z)
            if not all(math.isfinite(v) for v in values):
                self.errors.append("Non-finite raw probe observation")
                self.predictions = None
                continue
            self.raw.append(values)

    def compare(self, positions):
        rows = []
        for pos in positions:
            x, y, z = pos.test_x, pos.test_y, pos.test_z
            raw = [p[2] for p in self.raw if abs(p[0]-x) <= 1.e-5 and abs(p[1]-y) <= 1.e-5]
            matches = [p[2] for p in (self.predictions or [])
                       if abs(p[0]-x) <= 1.e-5 and abs(p[1]-y) <= 1.e-5]
            predicted = matches[0] if len(matches) == 1 else None
            transition = (predicted + self.uncertainty + self.avp.clearance
                          + self.stop_reserve if predicted is not None else None)
            rows.append(dict(x=x, y=y, observed_trigger_z=z,
                             raw_touch_count=len(raw), raw_trigger_min=min(raw) if raw else None,
                             raw_trigger_max=max(raw) if raw else None,
                             predicted_trigger_z=predicted,
                             prediction_error=z-predicted if predicted is not None else None,
                             proposed_transition_z=transition,
                             minimum_observed_gap=transition-max(raw)
                             if transition is not None and raw else None,
                             hypothetical_fast_region=bool(transition is not None
                                 and transition < self.travel_z
                                 and self.avp.approach_speed > self.probe_speed)))
        return rows

    def finalize(self, positions):
        record = dict(round=len(self.rounds), phase=self.phase, completed=False,
                      elapsed_seconds=self.avp.printer.get_reactor().monotonic()-self.started,
                      temperatures=self.temperatures(),
                      points=self.compare(positions))
        self.rounds.append(record)
        self.in_round = record
        self.active_positions = positions
        self.raw = []
        try:
            result = self.original_finalize(positions)
            record.update(completed=True, native_result=result)
            return result
        finally:
            self.active_positions = None
            self.in_round = None

    def adjust(self, adjustments, speed):
        before = self.avp.toolhead.get_position()[2]
        # The original function receives the exact arguments once and exceptions
        # propagate. No motion is added, removed, retried or accelerated here.
        result = self.original_adjust(adjustments, speed)
        after = self.avp.toolhead.get_position()[2]
        self.predictions = None
        if self.in_round is not None:
            self.in_round.update(adjustments=list(adjustments),
                                 toolhead_z_before=before, toolhead_z_after=after)
        try:
            if self.active_positions is None:
                raise ValueError("Adjustment outside captured QGL round")
            predictions = [(p.test_x, p.test_y, p.test_z + corner_shift(
                p.test_x, p.test_y, self.gantry, adjustments)) for p in self.active_positions]
            if not all(math.isfinite(v) for p in predictions for v in p):
                raise ValueError("Non-finite QGL prediction")
            self.predictions = predictions
        except ValueError as exc:
            self.errors.append(str(exc))
        return result

    def report(self):
        return dict(shadow_only=True, motion_changed=False, fast_approach_executable=False,
                    elapsed_seconds=self.avp.printer.get_reactor().monotonic()-self.started,
                    completed=self.completed, uncertainty=self.uncertainty,
                    assumed_stop_reserve=self.stop_reserve, clearance=self.avp.clearance,
                    proposed_approach_speed=self.avp.approach_speed,
                    native_probe_speed=self.probe_speed, native_travel_z=self.travel_z,
                    rounds=self.rounds, incomplete_round_raw_touches=len(self.raw),
                    errors=self.errors)
