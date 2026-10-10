"""Klipper commands for adaptive probing and bed topography diagnostics."""

import json
import math
import os
import sqlite3

from ..core import probing
from ..analytics.topography import surface_stats
from ..history.storage import History


class AVP:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object("gcode")
        self.minimum = config.getint("min_probe_count", 3, minval=3, maxval=15)
        self.maximum = config.getint("max_probe_count", 9,
                                     minval=self.minimum, maxval=15)
        if self.minimum % 2 != 1 or self.maximum % 2 != 1:
            raise config.error("AVP probe counts must be odd")
        self.mesh_min = config.getfloatlist("mesh_min", count=2)
        self.mesh_max = config.getfloatlist("mesh_max", count=2)
        self.bounds = (*self.mesh_min, *self.mesh_max)
        if not all(math.isfinite(v) for v in self.bounds) or any(
                lo >= hi for lo, hi in zip(self.mesh_min, self.mesh_max)):
            raise config.error("AVP mesh_min must be below mesh_max")
        self.threshold = config.getfloat("warp_threshold", 0.05, above=0.)
        self.travel_z = config.getfloat("horizontal_move_z", 10., above=0.)
        self.clearance = config.getfloat("clearance", 2., above=0.)
        self.surface_bound = config.getfloat("max_surface_z", None)
        self.speed = config.getfloat("travel_speed", 100., above=0.)
        self.approach_speed = config.getfloat("approach_speed", 10., above=0.)
        self.level_tolerance = config.getfloat(
            "level_tolerance", 0.05, above=0., maxval=1.)
        self.level_retries = config.getint("level_retries", 3,
                                          minval=1, maxval=30)
        numeric = [self.threshold, self.travel_z, self.clearance, self.speed,
                   self.approach_speed, self.level_tolerance]
        if self.surface_bound is not None:
            numeric.append(self.surface_bound)
        if not all(math.isfinite(v) for v in numeric):
            raise config.error("AVP settings must be finite")
        self.history_path = os.path.expanduser(config.get(
            "history_path", "~/printer_data/config/avp.sqlite3"))
        self.history_limit = config.getint("history_limit", 100, minval=1)
        self.history = None
        self.points = []
        self.last_scan_id = None
        self.printer.register_event_handler("klippy:connect", self.connect)
        for name in ("PLAN", "SCAN", "MESH", "CLEARANCE", "LEVEL", "HISTORY"):
            self.gcode.register_command("AVP_" + name,
                                        getattr(self, "cmd_AVP_" + name))

    def connect(self):
        self.toolhead = self.printer.lookup_object("toolhead")
        self.probe = self.printer.lookup_object("probe")
        try:
            self.history = History(self.history_path, self.history_limit)
        except (OSError, sqlite3.Error, ValueError) as exc:
            raise self.printer.config_error("AVP history: %s" % exc)

    def get_status(self, eventtime):
        return {"last_scan_id": self.last_scan_id,
                "point_count": len(self.points),
                "stats": surface_stats(self.points) if self.points else None}

    def require_homed(self, gcmd):
        status = self.toolhead.get_status(self.printer.get_reactor().monotonic())
        if not all(axis in status["homed_axes"] for axis in "xyz"):
            raise gcmd.error("Home XYZ before using AVP")
        return status

    def metadata(self):
        eventtime = self.printer.get_reactor().monotonic()
        bed = self.printer.lookup_object("heater_bed", None)
        return {"bounds": self.bounds,
                "bed_temperature": (round(bed.get_status(eventtime)["temperature"], 1)
                                    if bed is not None else None)}

    def probe_count(self, gcmd):
        try:
            history = self.history.recent(1)
            previous = (history[0]["points"] if history
                        and history[0]["metadata"].get("bounds") == list(self.bounds)
                        else [])
            return probing.adaptive_count(
                previous, self.threshold, self.minimum, self.maximum)
        except (ValueError, sqlite3.Error, TypeError) as exc:
            raise gcmd.error("AVP history: %s" % exc)

    def prepare_scan(self, gcmd):
        """Shared read-only preflight for preview and execution."""
        status = self.require_homed(gcmd)
        count = self.probe_count(gcmd)
        offsets = self.probe.get_offsets()
        if len(offsets) != 3 or not all(math.isfinite(v) for v in offsets):
            raise gcmd.error("AVP probe offsets must be finite XYZ")
        fast = gcmd.get_int("FAST_APPROACH", int(self.surface_bound is not None),
                            minval=0, maxval=1)
        try:
            heights = probing.scan_heights(
                self.travel_z, self.clearance, offsets[2], self.surface_bound,
                bool(fast))
        except ValueError as exc:
            raise gcmd.error("AVP scan: %s" % exc)
        travel_z, approach_z = heights["travel_z"], heights["approach_z"]
        positions = probing.grid_points(self.bounds, count)
        for x, y in positions:
            nozzle_xy = (x - offsets[0], y - offsets[1])
            if any(not status["axis_minimum"][i] <= v <= status["axis_maximum"][i]
                   for i, v in enumerate(nozzle_xy)):
                raise gcmd.error("AVP probe grid exceeds nozzle travel limits")
        if not status["axis_minimum"][2] <= travel_z <= status["axis_maximum"][2]:
            raise gcmd.error("AVP travel height exceeds Z limits")
        if approach_z is not None and approach_z < status["axis_minimum"][2]:
            raise gcmd.error("AVP approach height is below Z limits")
        current_z = self.toolhead.get_position()[2]
        if not math.isfinite(current_z) or current_z > status["axis_maximum"][2]:
            raise gcmd.error("AVP current Z exceeds safe travel limits")
        return status, count, offsets, positions, heights

    def cmd_AVP_PLAN(self, gcmd):
        status, count, offsets, positions, heights = self.prepare_scan(gcmd)
        report = {"advisory_only": True, "bounds": self.bounds,
                  "grid_count": count, "point_count": len(positions),
                  "fast_approach": heights["approach_z"] is not None,
                  "surface_bound": self.surface_bound,
                  "travel_z": heights["travel_z"],
                  "approach_z": heights["approach_z"],
                  "approach_speed": self.approach_speed,
                  "travel_speed": self.speed,
                  "probe_z_offset": offsets[2]}
        gcmd.respond_info("AVP plan (no motion; bound is not validated): %s"
                          % json.dumps(report))

    def cmd_AVP_SCAN(self, gcmd):
        status, count, offsets, positions, heights = self.prepare_scan(gcmd)
        travel_z, approach_z = heights["travel_z"], heights["approach_z"]
        trigger_offset = heights["trigger_offset"]
        self.points = []
        samples = []
        session = self.probe.start_probe_session(gcmd)
        try:
            for x, y in positions:
                # Never move XY while at the probing height.
                z = max(travel_z, self.toolhead.get_position()[2])
                self.toolhead.manual_move([None, None, z], self.approach_speed)
                self.toolhead.manual_move(
                    [x - offsets[0], y - offsets[1], None], self.speed)
                if approach_z is not None:
                    self.toolhead.manual_move(
                        [None, None, approach_z], self.approach_speed)
                session.run_probe(gcmd)
                results = session.pull_probed_results()
                if len(results) != 1:
                    raise gcmd.error("AVP expected one averaged probe result")
                result = results[0]
                if hasattr(result, "bed_z"):
                    point = (result.bed_x, result.bed_y, result.bed_z)
                else:
                    # Older Klipper sessions return trigger XYZ.
                    point = (result[0] + offsets[0], result[1] + offsets[1],
                             result[2] - offsets[2])
                if not all(math.isfinite(v) for v in point):
                    raise gcmd.error("AVP received a non-finite probe result")
                samples.append(point)
                travel_z = max(travel_z, point[2] + trigger_offset + self.clearance)
                # Retraction must never descend if native sampling/deployment
                # ended above the planned height. Carry this floor to next XY.
                current_z = self.toolhead.get_position()[2]
                if not math.isfinite(current_z):
                    raise gcmd.error("AVP received a non-finite toolhead Z")
                travel_z = max(travel_z, current_z)
                if not math.isfinite(travel_z) or travel_z > status["axis_maximum"][2]:
                    raise gcmd.error("AVP measured surface exceeds safe travel limits")
                self.toolhead.manual_move(
                    [None, None, travel_z], self.approach_speed)
                if self.surface_bound is not None and point[2] > self.surface_bound:
                    raise gcmd.error("AVP max_surface_z bound violated; scan stopped")
        finally:
            session.end_probe_session()
        self.toolhead.wait_moves()
        try:
            scan_id = self.history.save(samples, self.metadata())
        except (ValueError, sqlite3.Error, OSError) as exc:
            raise gcmd.error("AVP history save failed: %s" % exc)
        self.points = samples
        self.last_scan_id = scan_id
        gcmd.respond_info("AVP scan %d: %dx%d points; %s" % (
            scan_id, count, count, json.dumps(surface_stats(samples))))

    def cmd_AVP_MESH(self, gcmd):
        self.require_homed(gcmd)
        if self.printer.lookup_object("bed_mesh", None) is None:
            raise gcmd.error("AVP_MESH requires configured bed_mesh")
        mesh_config = self.printer.lookup_object(
            "configfile").get_status(0.)["config"]["bed_mesh"]
        if "mesh_radius" in mesh_config:
            raise gcmd.error("AVP_MESH currently requires a rectangular bed mesh")
        # Do not extend the native mesh's configured safe probing area.
        native_min = [float(v) for v in mesh_config["mesh_min"].split(",")]
        native_max = [float(v) for v in mesh_config["mesh_max"].split(",")]
        if any(v < lo for v, lo in zip(self.mesh_min, native_min)) or any(
                v > hi for v, hi in zip(self.mesh_max, native_max)):
            raise gcmd.error("AVP bounds exceed configured bed_mesh bounds")
        count = self.probe_count(gcmd)
        adaptive = gcmd.get_int("ADAPTIVE", 1, minval=0, maxval=1)
        margin = gcmd.get_float("ADAPTIVE_MARGIN", 5., minval=0.)
        if not math.isfinite(margin):
            raise gcmd.error("AVP adaptive margin must be finite")
        algorithm = "lagrange" if count <= 5 else "bicubic"
        self.gcode.run_script_from_command(
            "BED_MESH_CALIBRATE MESH_MIN=%.6f,%.6f MESH_MAX=%.6f,%.6f "
            "PROBE_COUNT=%d,%d ALGORITHM=%s ADAPTIVE=%d ADAPTIVE_MARGIN=%.6f" %
            (*self.bounds, count, count, algorithm, adaptive, margin))

    def cmd_AVP_CLEARANCE(self, gcmd):
        try:
            history = self.history.recent(1)
            points = self.points or (history[0]["points"] if history else [])
            if not points:
                raise ValueError("Run AVP_SCAN before predicting clearance")
            start = (gcmd.get_float("X0"), gcmd.get_float("Y0"))
            end = (gcmd.get_float("X1"), gcmd.get_float("Y1"))
            prediction = probing.clearance_prediction(
                points, start, end, self.clearance, self.surface_bound)
        except (ValueError, sqlite3.Error, TypeError) as exc:
            raise gcmd.error("AVP clearance: %s" % exc)
        gcmd.respond_info("AVP advisory only (not obstacle detection): %s"
                          % json.dumps(prediction))

    def cmd_AVP_LEVEL(self, gcmd):
        self.require_homed(gcmd)
        method = gcmd.get("METHOD", "AUTO").upper()
        if method == "AUTO":
            method = ("QUAD_GANTRY_LEVEL" if self.printer.lookup_object(
                "quad_gantry_level", None) is not None else "Z_TILT_ADJUST")
        objects = {"QUAD_GANTRY_LEVEL": "quad_gantry_level",
                   "Z_TILT_ADJUST": "z_tilt"}
        if method not in objects or self.printer.lookup_object(
                objects[method], None) is None:
            raise gcmd.error("AVP_LEVEL requires configured QGL or Z tilt")
        # A motor adjustment invalidates current scan coordinates, even on failure.
        self.points = []
        self.gcode.run_script_from_command(method + " SAMPLES=1 RETRIES=0")
        self.gcode.run_script_from_command(
            "%s RETRIES=%d RETRY_TOLERANCE=%.6f" %
            (method, self.level_retries, self.level_tolerance))
        gcmd.respond_info("AVP native gantry leveling completed")

    def cmd_AVP_HISTORY(self, gcmd):
        limit = gcmd.get_int("LIMIT", 10, minval=1, maxval=self.history_limit)
        try:
            scans = self.history.recent(limit)
            report = {"scans": [{k: v for k, v in scan.items() if k != "points"}
                                for scan in scans],
                      "comparison": self.history.compare()}
        except (ValueError, sqlite3.Error, TypeError) as exc:
            raise gcmd.error("AVP history: %s" % exc)
        gcmd.respond_info(json.dumps(report))


def load_config(config):
    return AVP(config)
