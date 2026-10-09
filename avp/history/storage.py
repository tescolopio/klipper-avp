"""Bounded SQLite scan persistence."""

import json
import sqlite3
from contextlib import closing
from datetime import datetime, timezone

from ..core.geometry import validate_points
from ..analytics.topography import compare_scans, surface_stats


class History:
    """Bounded SQLite history; only completed, validated scans are stored."""

    def __init__(self, path, limit=100):
        if limit < 1:
            raise ValueError("History limit must be positive")
        self.path, self.limit = path, limit
        with closing(sqlite3.connect(path)) as db, db:
            db.execute(
                "CREATE TABLE IF NOT EXISTS scans "
                "(id INTEGER PRIMARY KEY, timestamp TEXT NOT NULL, "
                "points TEXT NOT NULL, metadata TEXT NOT NULL)")

    def save(self, points, metadata):
        points = validate_points(points)
        surface_stats(points)
        payload = json.dumps(points, allow_nan=False)
        meta = json.dumps(metadata, allow_nan=False)
        timestamp = datetime.now(timezone.utc).isoformat()
        with closing(sqlite3.connect(self.path)) as db, db:
            cursor = db.execute(
                "INSERT INTO scans(timestamp, points, metadata) VALUES(?,?,?)",
                (timestamp, payload, meta))
            scan_id = cursor.lastrowid
            db.execute("DELETE FROM scans WHERE id NOT IN "
                       "(SELECT id FROM scans ORDER BY id DESC LIMIT ?)",
                       (self.limit,))
        return scan_id

    def recent(self, limit=10):
        if limit < 1:
            raise ValueError("History query limit must be positive")
        with closing(sqlite3.connect(self.path)) as db:
            rows = db.execute(
                "SELECT id, timestamp, points, metadata FROM scans "
                "ORDER BY id DESC LIMIT ?", (min(limit, self.limit),)).fetchall()
        return [{"id": row[0], "timestamp": row[1],
                 "points": validate_points(json.loads(row[2])),
                 "metadata": json.loads(row[3]),
                 "stats": surface_stats(json.loads(row[2]))} for row in rows]

    def compare(self):
        return compare_scans(self.recent(2))
