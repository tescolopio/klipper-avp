# AVP vertical approach evaluation

This increment makes the existing bound-based fast approach testable per command.
It does not reduce XY travel height, native sample retraction, sample count, or
sampling tolerance. It does not infer a physical bound from saved bed meshes.
No hardware performance or accuracy claim is made by the simulation.

## Controls

* `AVP_PLAN FAST_APPROACH=0`: read-only preflight and baseline height report.
* `AVP_PLAN FAST_APPROACH=1`: preview fast approach; rejects a missing bound.
* `AVP_SCAN FAST_APPROACH=0`: native slow descent from the travel height.
* `AVP_SCAN FAST_APPROACH=1`: fast descent to the bound-derived approach height,
  then native triggered probing, including configured multi-sample processing.

The switch applies to one command. Omission preserves existing configuration
semantics. `max_surface_z` remains a configuration value, not a command override.
Both modes retain the bound's travel floor and stop on a measured bound violation.
The stop cannot protect against an incorrect bound already crossed during descent.
The preview requires homing and shares execution preflight, but does not validate
probe state, surface height, temperature, obstacles, or arbitrary kinematic reach.
The existing native activation hook runs when probing, not during the preview.

Planning uses:

```
approach_z = max_surface_z + max(probe_z_offset, 0) + clearance
travel_z = max(horizontal_move_z, max(probe_z_offset, 0) + clearance,
               approach_z)
```

In baseline mode the bound-derived travel floor remains, but the approach move
is omitted. Scan results and current toolhead Z may raise the travel floor.
When native probing finishes above it, the retract target stays at least at the
current Z. Non-finite positions or an unreachable required lift abort the scan;
partial scans are not stored. Native kinematics still enforce movement limits.

## Reproducible simulation

Run `python3 -m unittest discover -s tests -v` from the repository root.
The `test_25_point_tap_ab_preserves_samples_and_clearance` integration test uses
a simulated 500 mm printer, 50..450 mm grid, zero XY probe offsets, -1.372 mm Z
offset, and an analytic flat surface. This resembles BigBoom's configuration,
but is not a replay of its physical surface or a validation of its bound.

| Model parameter | Value |
| --- | ---: |
| Grid | 5x5 |
| XY travel floor | 10 mm |
| Hypothetical surface bound (simulation only) | 0.5 mm |
| Clearance | 2 mm |
| Approach height | 2.5 mm |
| Native probe / sample lift speed | 2.5 mm/s |
| Fast approach / AVP lift speed | 5 mm/s |
| Samples per point | 3 |
| Extra touches from simulated retry | 3 |

Both modes produce identical XYZ measurements, perform 78 touches including
the same injected retry, keep XY moves at or above 10 mm, and finish at 10 mm.
The ideal time reduction comes only from each point's first descent:

```
25 * (10 - 2.5) * (1 / 2.5 - 1 / 5) = 37.5 seconds
```

This model excludes acceleration, MCU communication, probe settling, heating
waits, and real retry variability. It assumes the probe's trigger lies below the
approach plane. It does not predict a percentage improvement for BigBoom or
compare against native bed-mesh timing. Faster commanded approach may also be
limited by configured kinematics. A slower approach speed could lose time.

Additional tests cover missing bounds, positive/negative offsets, invalid limits,
non-finite inputs, per-command switching, native high-finish retraction, bound
violation, session cleanup, no partial persistence, and preview without motion.

## Later supervised comparison

Do not deploy or enable fast approach solely on the strength of the simulation.
Keep the Pi's pinned installation unchanged until the change is reviewed and a
physical surface bound is independently established for its current coordinate
frame. The largest historical measurement is not such a bound.

For BigBoom retain the 50..450 mm bounds chosen for clips. Retain existing Tap
temperature protection, three samples, 0.01 mm tolerance and retry limits.
After thermal stabilization, native QGL and the established post-QGL homing
procedure, compare baseline and fast scans without intervening homing, leveling,
configuration changes or plate handling. Alternate A/B order over multiple pairs.
Record start/end times, retries/touches, actual temperatures, and coordinate
reference changes. A faster run only passes if measurement quality also meets
the agreed acceptance criterion; that threshold remains to be established.

A separate native-versus-AVP baseline must use the same coordinates, samples,
temperatures, speeds and travel heights. Keep KAMP/adaptive clipping from changing
the test grid. QGL's coarse/fine optimization is a separate experiment; no QGL
behavior changes are included here. The parked history comparison fix is also
outside this PR, so use offline coordinate matching for repeatability analysis.
