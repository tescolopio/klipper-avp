# Offline history context policy

`avp.history.eligibility.assess_history_context` checks the timestamp, bed bounds,
and bed temperature of one existing `History.recent()` record. It performs no
database writes, uses no implicit clock, and imports no Klipper adapter. It
requires no schema migration. Existing history remains readable.

```python
from datetime import datetime, timezone
from avp.history.eligibility import assess_history_context

# `scan` is one record returned by History.recent(). Supply current conditions.
assessment = assess_history_context(
    scan,
    now=datetime.now(timezone.utc),
    max_age_seconds=3600,
    bounds=(10, 10, 190, 190),
    bed_temperature=60.,
    temperature_tolerance=2.,
)
```

The limits above are illustrative caller policy, not validated printer defaults.
The return value contains `eligible`, `reason`, and `age_seconds`. Eligibility
means only that these three context checks passed. It does not certify point
coverage, probe calibration, coordinate-frame validity, or physical clearance.
The function does not inspect scan points or select among multiple records.

Age must be between zero and `max_age_seconds`, inclusive. A scan exactly at
the limit passes; an older scan or a future timestamp fails. Both the stored
timestamp and `now` must carry timezone information; different UTC offsets are
compared as instants. Malformed, missing, or naive stored timestamps fail closed.
A backwards clock adjustment can cause rejection and requires investigation,
not silently treating the record as fresh.

Bounds must match exactly after normalizing list/tuple representation. All
bounds must be finite with increasing X and Y. Temperature difference must not
exceed `temperature_tolerance` (inclusive). Missing temperatures, including
legacy records or scans without a bed sensor, are rejected; unknown does not
mean matching. Booleans, strings, NaN, and infinity are not numeric conditions.

Invalid caller policy raises `ValueError`. Malformed stored context returns the
first rejection in this order: scan structure, timestamp, future age, stale age,
metadata structure, bounds validity/match, temperature validity/match. Reasons:

`invalid_scan`, `invalid_timestamp`, `future_timestamp`, `stale`,
`invalid_metadata`, `invalid_bounds`, `bounds_mismatch`, `invalid_temperature`,
`temperature_mismatch`, or `eligible`.

**Runtime integration is pending.** The adapter still uses its existing history
selection behavior. This API adds no printer configuration setting and does not
gate `AVP_SCAN`, `AVP_MESH`, or `AVP_CLEARANCE`. A subsequent change must add
explicit user settings, provenance and coordinate-frame invalidation, visible
rejection diagnostics, and simulated fallback tests before using this policy in
commands. Passing these checks must never authorize lowering motion clearance.
