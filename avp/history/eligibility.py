"""Offline history context checks; eligibility is not a clearance guarantee."""

import math
from datetime import datetime


def _number(value):
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        return False
    try:
        return math.isfinite(value)
    except OverflowError:
        return False


def _bounds(value):
    return (isinstance(value, (list, tuple)) and len(value) == 4
            and all(_number(v) for v in value)
            and value[0] < value[2] and value[1] < value[3])


def assess_history_context(scan, *, now, max_age_seconds, bounds,
                           bed_temperature, temperature_tolerance=0.):
    """Check one stored scan without reading a clock, database, or printer.

    Age and temperature limits are inclusive. Future/naive timestamps and
    missing conditions fail closed. Invalid caller policy raises ValueError;
    malformed stored context returns a rejection reason. This checks context
    only: it does not validate points, provenance, obstacles, or homing state.
    The Klipper adapter does not yet call this function.
    """
    if not isinstance(now, datetime) or now.utcoffset() is None:
        raise ValueError("now must be a timezone-aware datetime")
    if not _number(max_age_seconds) or max_age_seconds <= 0:
        raise ValueError("max_age_seconds must be positive and finite")
    if not _bounds(bounds):
        raise ValueError("Expected increasing finite bounds")
    if not _number(bed_temperature):
        raise ValueError("bed_temperature must be finite")
    if not _number(temperature_tolerance) or temperature_tolerance < 0:
        raise ValueError("temperature_tolerance must be nonnegative and finite")

    def result(reason, age=None):
        return {"eligible": reason == "eligible", "reason": reason,
                "age_seconds": age}

    if not isinstance(scan, dict):
        return result("invalid_scan")
    try:
        timestamp = datetime.fromisoformat(scan["timestamp"])
        if timestamp.utcoffset() is None:
            return result("invalid_timestamp")
        age = (now - timestamp).total_seconds()
    except (KeyError, TypeError, ValueError, OverflowError):
        return result("invalid_timestamp")
    if age < 0:
        return result("future_timestamp", age)
    if age > max_age_seconds:
        return result("stale", age)
    metadata = scan.get("metadata")
    if not isinstance(metadata, dict):
        return result("invalid_metadata", age)
    recorded_bounds = metadata.get("bounds")
    if not _bounds(recorded_bounds):
        return result("invalid_bounds", age)
    if tuple(recorded_bounds) != tuple(bounds):
        return result("bounds_mismatch", age)
    recorded_temperature = metadata.get("bed_temperature")
    if not _number(recorded_temperature):
        return result("invalid_temperature", age)
    if abs(recorded_temperature - bed_temperature) > temperature_tolerance:
        return result("temperature_mismatch", age)
    return result("eligible", age)
