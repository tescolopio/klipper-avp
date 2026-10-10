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

1. Pinned-code coarse/fine contract tests are implemented (see below). Hardware
   baseline timings and convergence measurements remain pending. Preserve
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

## Pinned Klipper integration tests

The separate CI job runs 10 tests using upstream revision
`461c4e3722c3a897fba1c6b3f0780a5315043842`. It exercises AVP's real `cmd_AVP_LEVEL`,
Klipper's QGL geometry and adjustment limits, `RetryHelper`, `ProbePointsHelper`,
`ProbeParameterHelper`, `SampleAveragingHelper`, and native command parsing.
No upstream source is copied or modified. A wrong revision or tracked modification
in the reference checkout fails the test module rather than silently skipping it.

Run separately from the 50 ordinary offline tests, using an isolated checkout:

```sh
git clone https://github.com/Klipper3d/klipper.git ../klipper-reference
git -C ../klipper-reference checkout --detach 461c4e3722c3a897fba1c6b3f0780a5315043842
KLIPPER_SOURCE="$(cd ../klipper-reference && pwd)" \
  python3 -m unittest discover -s tests/integration -v
```

For PowerShell, set `$env:KLIPPER_SOURCE` to the checkout's absolute path, then
run the same `python -m unittest discover -s tests/integration -v` command.
The integration directory intentionally is not a package, keeping ordinary
recursive test discovery independent of the external checkout.

Verified scenarios:

| Scenario | Result |
| --- | --- |
| Coarse pass then converged refinement | 4 single touches, then 12 touches at 3 per corner; 2 adjustment callbacks |
| One refinement retry | All four corners reprobed; 28 total touches |
| Native sample tolerance retry | Real averaging loop discards the bad sample group; 18 total touches |
| Timeout at start/middle of either pass | Error propagated; session cleaned up; no later simulated move |
| Coarse adjustment above native maximum | No adjustment callback or refinement command |
| Coarse adjustment callback fails | No refinement command; session cleaned up |
| Refinement retries exhausted or error increasing | Native error; no success response; applied status false |
| Sample retries exhausted | Refinement aborts; session cleaned up |
| XYZ not homed | No native command, probing or motion |

The fixtures use the 50..450 mm corner pattern, -1.372 mm probe offset, 2.5 mm/s
probe speed, three median samples, 5 mm sample retract, 0.01 mm sample tolerance
and ten sample retries. Gantry motor locations and post-adjustment measurements
are synthetic. The harness replaces toolhead movement, the raw probe hardware
session and physical stepper adjustment, and reproduces the top-level command
error event for cleanup. It does not execute MCU homing, Tap activation templates,
CAN transport, the real motor-adjustment implementation or thermal behavior.

Notable native behavior: `RETRIES=0` returns done without a convergence check and
can temporarily mark QGL applied after the coarse pass. The refinement command
resets that status and must independently pass tolerance. The tests check final
status on both success and failure; coarse status is not a clearance authorization.
Acceleration and preparation invalidation remain a separate follow-up PR.
