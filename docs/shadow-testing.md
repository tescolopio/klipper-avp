# BigBoom QGL shadow test

Shadow mode performs ordinary two-pass QGL and logs hypothetical variable-speed
predictions. It does move the printer at the existing native settings. It never
executes a faster approach, changes sampling, or lowers native travel clearance.
`AVP_SCAN FAST_APPROACH=1` remains disabled.

## Compatibility and scope

Supported host: `v0.13.0-786-g461c4e37` (also its `-dirty` version, to permit the
untracked AVP installation). A dirty version is not proof that upstream code is
unchanged; check tracked host modifications before installation. Other versions,
Z tilt, non-native QGL, and unsupported gantry geometry reject shadow mode before
movement. Default `AVP_LEVEL` behavior is unchanged.

The observer temporarily wraps the native finalize/adjust callbacks only while
this command runs and restores both in a `finally` path on success or failure.
It listens to `probe:update_results` to count all raw touches, including native
sample retries. It does not replace probe sessions, Tap activation templates,
endstop monitoring or stepper adjustment. Estimates are local to one invocation;
every new invocation starts with an unpredicted, native coarse pass.

## Operator procedure after installing the reviewed revision

1. With no print running, turn heaters off before stopping/restarting Klipper to
   update the package. Back up the configuration and old AVP revision. Do not pull
   an unreviewed moving branch for a hardware test; use the supplied commit pin.
2. Confirm the probe still changes open/triggered/open using the established manual
   check. Clear the bed and verify the four configured QGL locations avoid clips.
3. Re-establish the normal thermal state (BigBoom bed 100 C, nozzle near 150 C,
   existing Tap temperature protection) and heat soak. Hardware tightening and
   restart mean the earlier homing/reference must not be assumed valid.
4. Home using the normal `G28` procedure. Once successful, run one supervised test:

```gcode
AVP_LEVEL METHOD=QUAD_GANTRY_LEVEL SHADOW=1
```

5. Save the `AVP QGL shadow:` JSON line and the ordinary QGL output from Fluidd or
   klippy.log. Do not run a mesh scan or change speeds for this test. A native
   probing error ends the command; the report may contain incomplete rounds.
   Inspect the error instead of retrying automatically.

A report does not authorize acceleration. In particular, a positive gap on one
run is not a physical clearance bound, and a negative gap is evidence the proposed
transition was inadequate. Leave fast approach disabled regardless of the result.

## Reading the report

* `shadow_only=true`, `motion_changed=false`, `fast_approach_executable=false`:
  the report describes a proposal, not executed variable-speed motion.
* `rounds`: coarse pass, then each native refinement/retry round. Each includes
  temperatures/targets, elapsed time, adjustments, toolhead Z before/after, and
  whether native processing completed. A completed round may request `retry`.
* `predicted_trigger_z`: prior round trigger Z adjusted into the next frame using
  the ideal bilinear gantry model. The first coarse round has no prediction.
* `prediction_error`: actual averaged trigger Z minus prediction.
* `raw_touch_count`, `raw_trigger_min/max`: include samples discarded by retries.
* `proposed_transition_z`: prediction + configured AVP clearance + diagnostic
  uncertainty + assumed stopping reserve. `minimum_observed_gap` subtracts the
  largest raw trigger in that round, not just the averaged result.
* `hypothetical_fast_region`: whether the proposed transition is below native
  QGL travel and configured approach speed is greater than native probe speed.
  It never enables movement.

`SHADOW_UNCERTAINTY` defaults to 0.1 mm and `SHADOW_STOP_RESERVE` to 0.5 mm.
These are deliberately labeled diagnostic assumptions, not validated BigBoom
error or stopping-distance bounds. They affect report arithmetic only. Native
QGL travel is taken from QGL configuration, not AVP's scan travel setting.

## Coordinate model and verification

Pinned `ZAdjustHelper` moves motor i physically by `a_i - min(a)`, while the
reported toolhead Z changes by `-min(a)`. At nozzle XY with ideal bilinear motor
weights `w_i`, the next trigger coordinate shifts by:

```
-min(a) - sum(w_i * (a_i - min(a))) = -sum(w_i * a_i)
```

The test executes the real helper against simulated motors, including positive,
negative, equal and zero adjustments, and verifies this transform at corners and
an interior point. Other tests verify identical native commands, movement lists,
speeds, samples and adjustment values with shadow on/off, retry observations,
temperatures, callback restoration after failure, and no cross-command reuse.

This is a rigid/ideal model, not a measurement of backlash, frame compliance,
thermal drift or actual stopping behavior. The dirty-version allowance does not
certify local Klipper patches. Full variable-speed backend integration and actual
invalidation/stop wiring remain separate unfinished work.
