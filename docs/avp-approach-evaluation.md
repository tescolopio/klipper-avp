# AVP two-pass leveling and approach safety

## Intended workflow

For BigBoom, use four configured QGL corners with one native Tap sample each
for the coarse adjustment, then a refinement pass. `AVP_LEVEL` already delegates:

```
QUAD_GANTRY_LEVEL SAMPLES=1 RETRIES=0
QUAD_GANTRY_LEVEL RETRIES=<level_retries> RETRY_TOLERANCE=<level_tolerance>
```

The second command uses native configured sampling and convergence checks. The
first pass physically adjusts the gantry; its readings are not a reusable surface
bound after that adjustment. The current implementation does not use those four
measurements to accelerate the second pass. That optimization is still work to do.
Four corner measurements also cannot establish clearance over interior obstacles.
Saving samples in the first pass is separate from increasing descent speed.

## Implemented safety change

The previous fast scan approach used an ordinary `manual_move` before native
probing. Tap could not stop that move on unexpected contact. That call is removed.

* `AVP_SCAN` and `AVP_SCAN FAST_APPROACH=0` use native probe descent.
* `AVP_SCAN FAST_APPROACH=1` rejects before motion, activation, or scan-state changes.
* Configuring `max_surface_z` no longer implicitly enables fast descent.
* `AVP_PLAN FAST_APPROACH=1` previews hypothetical geometry and requires a bound.
  It reports `fast_approach_executable: false`; it cannot arm a scan.
* Scans retain the bound-derived travel floor, native samples/retries/hooks,
  measured-bound checks, and upward-only retraction after probing.

This does not guarantee against crashes. Native probing depends on working
hardware, correct configuration and activation hooks. XY travel is not obstacle
monitored. Saved meshes never authorize reduced clearance. No Pi deployment or
hardware validation accompanies this change.

## Pinned Klipper API review

Reviewed BigBoom host revision `461c4e37`:

* [homing.py](https://github.com/Klipper3d/klipper/blob/461c4e37/klippy/extras/homing.py):
  `HomingMove.homing_move` starts MCU endstop monitoring and uses `drip_move`.
  `check_triggered=False` permits no contact at the endpoint, but trigger times
  remain local. Returned position alone cannot reliably distinguish endpoint
  contact from reaching the endpoint normally. An explicit trigger result is needed.
* [probe.py](https://github.com/Klipper3d/klipper/blob/461c4e37/klippy/extras/probe.py):
  the public probe session does not expose a bounded non-contact approach.
  Underlying endstop and deployment access would depend on implementation details.
  Tap temperature activation must precede any approach, not only the later sample.
* Probe-based Z homing uses a separate path from `homing:home_rails_begin` in this
  revision. That event alone cannot invalidate preparation for all homing.

No monitored backend or preparation token is implemented here. Fast execution
remains disabled regardless of homing, QGL, restart, offsets or plate handling.
An incomplete lifecycle tracker must not be represented as a safety interlock.

## Next milestones, in priority order

1. Test the existing coarse/fine QGL sequence against pinned Klipper code and
   capture native baseline timings, convergence and total touches. Preserve
   BigBoom's configured four QGL points, Tap protections, final samples/tolerance.
2. Design acceleration within the second QGL pass, accounting for each gantry
   adjustment changing the coordinate reference. Do not apply pre-adjustment
   measurements as an unchanged bed-height bound.
3. Implement a monitored backend with explicit endpoint/no-contact, contact and
   fault results. Unexpected contact, including endpoint contact, aborts without
   another XY move or an automatic retry. Cover initially triggered Tap, activation
   failure, CAN timeout, cleanup and position reconciliation using pinned Klipper
   code, not only mocks. Establish allowable approach speed and stopping distance.
4. Implement preparation invalidation across all homing paths, direct/AVP QGL and
   Z tilt, individual/all motor disable, restart/disconnect and coordinate/probe
   offset changes, including failed operations. Plate/clip handling needs an
   operator reset. Saved history must never re-arm motion.
5. Review before any supervised accelerated hardware test. Keep BigBoom's Pi
   pinned and `max_surface_z` unset for now; retain 50..450 mm scan bounds, 10 mm
   travel, existing Tap temperature protection, three samples and 0.01 mm tolerance.

## Offline verification

Run `python3 -m unittest discover -s tests -v` from the repository root.
The 25-point Tap-style adapter simulation verifies native baseline samples,
78 touches including an injected retry, 10 mm XY travel and final lift. Other tests
cover rejection before session creation, preservation of completed results,
preflight, bound violations, cleanup and no partial persistence. These mocks do
not validate endstop stopping behavior or demonstrate second-pass acceleration.

A hypothetical 10-to-2.5 mm descent at 5 instead of 2.5 mm/s saves 1.5 seconds
per point before overhead. This is arithmetic, not a hardware benchmark, a
validated surface bound, or an implemented QGL optimization.
