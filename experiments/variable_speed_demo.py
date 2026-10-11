"""Print hypothetical second-pass corner plans. No printer access or motion."""
import json
from dataclasses import asdict
from .variable_speed import CornerEstimate, plan_descent


def report():
    # Entirely synthetic trigger heights and post-QGL frame transforms.
    # These are not BigBoom measurements or recommended hardware settings.
    corners = [(50., 50.), (50., 450.), (450., 450.), (450., 50.)]
    triggers = [-1.3, -1.1, -1.0, -1.2]
    shifts = [.15, -.10, -.15, .10]
    rows = []
    for xy, trigger, shift in zip(corners, triggers, shifts):
        coarse = CornerEstimate(*xy, trigger, .1, frame=0)
        current = coarse.after_adjustment(
            frame=1, trigger_shift=shift, shift_uncertainty=.15)
        plan = plan_descent(
            start_z=10., predicted_trigger_z=current.trigger_z,
            uncertainty=current.uncertainty, clearance=2., approach_speed=5.,
            probe_speed=2.5, deceleration=100., latency=.01, z_min=-5., z_max=500.)
        rows.append(dict(corner=xy, synthetic_trigger_shift=shift,
                         current_frame_estimate=asdict(current), plan=asdict(plan)))
    return dict(hardware_enabled=False, synthetic_only=True,
                coarse_touches=4, refinement_touches_without_retries=12,
                ideal_first_descent_saving_seconds=sum(
                    row["plan"]["ideal_saving"] for row in rows), corners=rows,
                caveat="Excludes acceleration, arming, settling, retries and thermal waits")


if __name__ == "__main__":
    print(json.dumps(report(), indent=2, allow_nan=False))
