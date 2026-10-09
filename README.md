# klipper-avp
Fast-approach, slow-touch Z-probing for 3D printers: cuts probing time without sacrificing accuracy.

AVP is a Python-only Klipper extension for adaptive probing, advisory
collision-clearance prediction, coarse/fine gantry leveling, and persistent
bed-topography analytics. No additional Python dependencies are needed.

## Installation

On the printer host, link both modules into your Klipper installation. For
example, with this repository at `/home/pi/klipper-avp`:

```sh
ln -s /home/pi/klipper-avp/klippy/extras/avp.py /home/pi/klipper/klippy/extras/avp.py
ln -s /home/pi/klipper-avp/klippy/extras/avp_core.py /home/pi/klipper/klippy/extras/avp_core.py
```

Copy the settings from `/home/pi/klipper-avp/config/avp.cfg` into your printer
configuration, **adjust the bed bounds and speeds for your machine**, and
restart Klipper. An existing `[probe]`-compatible probe is required. The history
directory must already exist and be writable. This extension uses Klipper's
probe-session API; it supports current `ProbeResult` records and older XYZ
session results, not older versions without probe sessions.

## Commands

Home XYZ before any probing, mesh calibration, or leveling.

| Command | Behavior |
| --- | --- |
| `AVP_SCAN PROBE_SPEED=3` | Probe a serpentine regular grid using native probe sampling and persist a completed scan. |
| `AVP_MESH ADAPTIVE=1 ADAPTIVE_MARGIN=5` | Install a native Klipper bed mesh with history-selected grid density and optional print-area clipping. Requires rectangular `[bed_mesh]`; object clipping uses Klipper's `[exclude_object]`. |
| `AVP_CLEARANCE X0=30 Y0=30 X1=180 Y1=180` | Report estimated path peak and recommended nozzle Z, without moving. XY is in measured bed coordinates. |
| `AVP_LEVEL METHOD=AUTO` | Single-sample coarse adjustment, then native tolerance-checked leveling. Supports `QUAD_GANTRY_LEVEL` or `Z_TILT_ADJUST` explicitly; AUTO prefers QGL. |
| `AVP_HISTORY LIMIT=10` | Report timestamped scans, bed temperature, tilt, height range, detrended warp/RMS, and changes between the latest two scans. |

Grid density starts at `min_probe_count`. Subsequent scans and meshes use the
latest scan with identical bounds: every additional `warp_threshold` of
plane-detrended peak-to-valley deformation adds two points per axis, capped at
`max_probe_count`. Simple bed tilt does not increase density. Counts must be odd
and between 3 and 15. A coarse scan cannot detect every small surface feature;
choose a suitable minimum for your bed and rescan when conditions change.

`AVP_SCAN` is diagnostic: it **does not install a compensation mesh**.
`AVP_MESH` invokes `BED_MESH_CALIBRATE` with native configured probe speeds and
sampling, preserving native zero-reference and faulty-region handling. Its
bounds cannot extend beyond the native mesh configuration. Use
`ADAPTIVE=0` to mesh the complete AVP rectangle. Native print-area clipping can
reduce counts further. Mesh calibration itself is not added to scan history.

The coarse leveling pass uses `SAMPLES=1 RETRIES=0`; the final pass restores
configured probe sampling and uses `level_retries` (at least one) and
`level_tolerance`. It retains native travel heights, probe limits, motor
adjustment limits, and error handling. This can save measurements during large
initial corrections on multi-sample setups; it is not guaranteed faster for
an already level gantry. Current scan coordinates are invalidated by leveling.
Level **before** scanning/calibrating a mesh, and follow your printer's usual
post-leveling homing procedure.

## Motion safety and fast approach

By default AVP does not perform a blind fast descent. Every scan raises Z before
moving XY, applies probe XY offsets, and uses native slow, triggered probing,
including sampling/tolerance retries. It never changes native probe speed or
individual Z motors directly. `PROBE_SPEED`, `SAMPLES`, and other native probe
options can be supplied to `AVP_SCAN`.

To enable fast approach, explicitly set `max_surface_z` to a **verified upper
bound on physical bed height in homed toolhead coordinates** across the entire
scan. AVP approaches at `approach_speed` only to:

```
max_surface_z + max(probe_z_offset, 0) + clearance
```

It then probes at native `PROBE_SPEED`. Travel Z is at least
`horizontal_move_z`, the approach height, and the highest measured trigger
height plus clearance. The probe trigger offset is included so the approach
does not deliberately cross the trigger plane. A measurement above the bound
stops the scan after retraction, but **cannot retroactively prevent a collision
from an incorrect bound**. Leave this setting disabled until validated.

History predictions **never lower motion clearance**. The path estimate uses
inverse-distance interpolation within measured bounds; the recommendation uses
the highest measurement anywhere in the scan (or the configured bound, if
higher) plus clearance. This is advisory, not a guarantee: unmeasured peaks,
clips, printed parts, probe deployment motions, tool offsets, frame changes,
and stale temperature-dependent history are not obstacle detection. Verify
clearance physically, keep hardware protections enabled, and supervise initial
runs. Native kinematics enforce reachability; rectangular prechecks do not
prove reachability on every printer geometry.

Only successful complete scans are written to SQLite. History retains the
latest `history_limit` scans (100 by default). Comparisons use matching XY
locations and report whether recorded bounds/temperatures match; changes under
different conditions should not be interpreted as bed wear.

## Development

Run the dependency-free tests from the repository root:

```sh
cd /home/pi/klipper-avp
python3 -m unittest discover -s tests -v
```

The tests cover planning, persistence, and Klipper command integration with
simulated toolhead/probe objects. Hardware accuracy, physical clearance, and
performance still require validation on the target printer.
