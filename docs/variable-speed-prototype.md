# Variable-speed probing: offline prototype

Status: motion research code remains in `experiments/`, with no import from the
production AVP adapter and no live variable-speed backend. PR #3 is merged and
this PR targets main. A separate opt-in [QGL shadow observer](shadow-testing.md)
now collects predictions during ordinary QGL. Installing it does not enable
acceleration; the printer changes only when the operator updates it.

## The intended four-corner workflow

1. Perform the ordinary four-corner coarse QGL pass, one native-speed Tap touch at
   each corner. Record native trigger coordinates, not just rounded console text.
2. QGL adjusts the gantry. Invalidate the old frame before applying the adjustment.
   Transform the corner estimates into the new toolhead frame with an explicit
   uncertainty allowance, or decline acceleration if the transform is unavailable.
3. At the same corner in the refinement pass, approach more quickly only above the
   predicted trigger region. Keep the endstop monitored throughout that descent.
4. Reach the slow region without contact, then let native Klipper perform its
   ordinary multi-sample measurement, retraction, tolerance and retries.
5. Treat every subsequent gantry adjustment as a new frame. A failed operation,
   unexpected contact, or stale preparation cannot fall through to more probing.

This is initially a two-speed system, not continuous velocity adaptation. It
accelerates only the first descent at a refinement corner. Native repeat samples
remain unchanged. It does not lower XY travel or interpolate corner observations
across the unmeasured interior. Each speed phase ends before the next begins;
there is no assumption of seamless, zero-overhead velocity switching.

## Implemented in the prototype

`experiments/variable_speed.py` contains a pure height/speed planner and a one-use
preparation controller. It accepts current-frame trigger predictions with explicit
uncertainty, falls back to native-only probing if there is no useful fast region,
rejects stale frames and corner substitutions, and consumes preparation before
activation. Any backend fault invalidates the preparation. It runs cleanup after
activation or descent failure, issues no recovery motion itself, and requires
explicit no-contact completion before allowing native sampling.

The advisory transition height is:

```
stop_reserve = approach_speed * assumed_latency
             + approach_speed^2 / (2 * assumed_deceleration)
slow_start_z = predicted_trigger_z + uncertainty + clearance + stop_reserve
```

The stopping model is only an assumption for offline evaluation. MCU timing,
mechanical compliance, Tap travel and actual deceleration have not been validated.
Endstop monitoring cannot make contact at arbitrary speed harmless. The initial
approach speed must remain conservative even when ample predicted clearance exists.

`CornerEstimate.after_adjustment` deliberately requires an externally supplied
trigger-height shift and shift uncertainty. It does not equate raw motor
adjustments with a toolhead-coordinate shift. A model must include gantry geometry,
all four actuator changes and Klipper's coordinate reassignment. The current demo
supplies synthetic shifts. The separate shadow observer implements an ideal
bilinear transform tested against native `ZAdjustHelper`; its accuracy on the
physical gantry remains to be measured.

Preparation invalidation is tested as policy for homing, QGL/Z tilt, all/individual
motor disable, restart/disconnect, offsets, plate reset and thermal changes. These
are explicit simulation calls, NOT installed Klipper event handlers. New frame
numbers and predictions remain a trusted backend responsibility. Temperature
activation, printer homing and actual XY positioning also belong to that backend.

## Explicit outcome experiment against pinned Klipper

`experiments/pinned_monitor.py` observes the real endstop `home_wait` return while
letting upstream `HomingMove` manage endstop arming, `drip_move`, position
reconciliation and completion events. It does not replace endstop monitoring with
an ordinary move or infer trigger state from the final Z coordinate.

* Zero timestamp: no contact; verify endpoint reached within supplied step tolerance.
* Positive timestamp: contact, including exactly at the endpoint; abort the policy.
* Missing/invalid timestamp, timeout or native motion error: fail closed.

Tests use the same pinned Klipper revision as PR #3 and four simulated Z steppers.
They cover early contact, exact-endpoint contact with and without synthetic
overshoot, untriggered completion, initially triggered probe, absent steppers,
short move, bad completion data, native motion failure and a simulated CAN timeout.
One test connects the controller to real `HomingMove` and verifies contact prevents
native sampling. This is a technical feasibility experiment, not a verified probe
backend. The endstop proxy may affect identity-sensitive callbacks; compatibility
with deployment hooks and every attached Z stepper still needs review.

## Running offline

```sh
python3 -m experiments.variable_speed_demo
python3 -m unittest discover -s tests/unit -v
```

The demo emits four hypothetical current-frame plans, an explicit
`hardware_enabled: false`, and ideal timing arithmetic. Its present example yields
6.98 seconds saved across the four initial refinement descents, before arming,
acceleration, settling and communication overhead. That number is not a measured
BigBoom improvement and not a speed recommendation. The coarse pass still takes
four touches and a no-retry refinement still takes twelve.

For upstream integration, use the clean isolated checkout documented in
[the PR #3 evaluation](avp-approach-evaluation.md#pinned-klipper-integration-tests),
set `KLIPPER_SOURCE`, and run:

```sh
python3 -m unittest discover -s tests/integration -v
```

## Remaining work before any hardware enablement

1. The ideal transform is now tested against actual `ZAdjustHelper` coordinate
   changes with simulated motors. Compare its predictions against fresh native
   measurements using shadow mode. No measured uncertainty bound exists yet.
2. Build a pinned-version probe backend with verified endstop/stepper attachment,
   Tap activation/deactivation and thermal protections, native sampling ownership,
   position reconciliation and command-error cleanup. Test the full lifecycle,
   including asynchronous faults and identity-sensitive homing callbacks.
3. Wire invalidation at every real entry point. In-flight invalidation must invoke
   the appropriate native stop/shutdown behavior; the policy alone only prevents
   subsequent stages. Plate changes require operator acknowledgement.
4. Collect the implemented observation-only QGL report: proposed slow regions,
   actual next-pass trigger points, adjustments, temperatures and raw touches.
   Motion speeds and travel heights stay native while collecting that evidence.
5. After mechanical inspection and stable native probing, review measured stopping
   behavior and an explicit supervised acceptance plan before enabling any motion.

BigBoom scans 8/9 matched all 25 nominal locations offline, with approximately
0.0129 mm RMS and 0.0328 mm maximum point-to-point change. The operator reported
sample retries. That is useful baseline evidence, not a sufficient uncertainty
bound for fast motion and not proof of mechanical failure. The existing scan
comparison still has its separate floating-coordinate matching limitation.
