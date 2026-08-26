#!/usr/bin/env python3
"""Live scene-placement checker for a physical robot workspace.

A person lays bricks and blocks on the table from a printed build card. This
watches the overhead ArUco camera, compares each detected item's pose to the
target pose in a scene build sheet, and prints live offsets so the person can
nudge things into place. On Enter (or on a timer with --auto) it runs the
same physics-based checksum `make_build_cards.py` uses to grade a sheet,
against the OBSERVED poses, and says whether the built scene is the one the
sheet's numbers describe.

Does not reimplement the checksum. `scene_contact_counts` and its tolerance
constants come from make_build_cards.py; that stays the single source of
truth for what counts as the same scene.

Usage:
    PYTHONPATH=src python scripts/check_build.py \\
        --sheet ../namo_cpp/handoff/real_scene_build_sheets/1push/easy.csv \\
        --build-id easy_000

    # live schematic + camera window, auto-checksum every 5s, closes on PASS
    PYTHONPATH=src python scripts/check_build.py --sheet easy.csv \\
        --build-id easy_000 --gui --auto 5

    # headless self-test, no camera
    PYTHONPATH=src python scripts/check_build.py --sheet easy.csv \\
        --build-id easy_000 --simulate --noise-cm 0.3 --noise-deg 2
"""
from __future__ import annotations

import argparse
import csv
import random
import select
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

# make_build_cards.py lives next to this file. Added so `import
# make_build_cards` works whether this is run as `python check_build.py` or
# `python scripts/check_build.py` from the repo root.
SCRIPTS_DIR = Path(__file__).resolve().parent
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

from make_build_cards import (  # noqa: E402 (path setup above must run first)
    CHECKSUM_COLUMNS,
    CHECKSUM_TIE_TOLERANCE_CONTACTS,
    CONTACTS_PER_OBJECT,
    WORKSPACE_H_CM,
    WORKSPACE_W_CM,
    _corners,
    scene_contact_counts,
)

# ─── Named constants ────────────────────────────────────────────────────

# camera_service.py's --port default. --camera-service uses this address
# unless told otherwise, since that is what is actually running at the table.
DEFAULT_CAMERA_SERVICE_PORT = 5556
DEFAULT_CAMERA_SERVICE_ADDRESS = f"tcp://localhost:{DEFAULT_CAMERA_SERVICE_PORT}"

# Requested live-loop refresh rate. Also doubles as the GUI window's key-poll
# timeout, so one number paces both the terminal block and the OpenCV window.
REFRESH_HZ = 2.0
REFRESH_PERIOD_S = 1.0 / REFRESH_HZ

# Live "OK" marker tolerance. Display aid only: the contact checksum in
# scene_contact_counts is the real pass/fail criterion, not this.
DISPLAY_TOL_CM = 2.0
DISPLAY_TOL_DEG = 10.0

# Sheet centres/sizes are rounded to 0.1 cm, so long_cm == short_cm within
# this tolerance is "square", not exact float equality.
SQUARE_TOL_CM = 0.1

# A bar's long-axis bearing is only defined mod 180 (two ends look the same
# footprint). A square's is only defined mod 90. See make_build_cards.py's
# module docstring for why yaw is not used instead. Camera theta can arrive
# unwrapped from continuous multi-turn tracking (e.g. 730.4 instead of
# 10.4); the modulo wrap below is exact for any magnitude, so no separate
# unwrap step is needed before comparing. Confirmed against the live
# camera_service on 2026-08-23.
BAR_SYMMETRY_DEG = 180.0
SQUARE_SYMMETRY_DEG = 90.0
FLIP_ROTATION_DEG = 90.0

# How close a non-square item's wrapped dtheta must be to the 90-degree mark
# to shout "FLIPPED?" instead of just failing the OK check. Set to the far
# edge of the OK band (90 - DISPLAY_TOL_DEG), so the shout fires exactly for
# errors that look like the long and short axes got swapped, not any old
# large error.
FLIP_SUSPECT_DEG = BAR_SYMMETRY_DEG / 2.0 - DISPLAY_TOL_DEG

# config/objects.yaml documents (top of file): depth = X dimension, ALONG the
# marker heading at theta=0; width = Y, perpendicular to it. So a marker's
# raw theta lies along the object's DEPTH side, not necessarily its world
# long axis. When depth >= width the marker heading already IS the long
# axis, so bearing = theta. When width > depth the long axis is the marker's
# perpendicular side, so bearing = theta + 90. Confirmed 2026-08-23 against
# a live annotated frame plus objects.yaml's shapes: wall_9 (depth 19.0 >=
# width 5.5, tag glued along the long side per the comment above wall_9 in
# objects.yaml) theta 1.2 sat at true bearing ~0, no offset; wall_11 (width
# 19.5 > depth 5.5, standard mounting) theta 96.8 sat at true bearing ~7
# after mod 180 (96.8 + 90 = 186.8); wall_10, same mounting, theta 31.3 sat
# at true bearing ~121 (31.3 + 90 = 121.3).
MARKER_TO_BEARING_OFFSET_DEG = 90.0

# Seed for --simulate's position/bearing noise, so a self-test run reproduces.
SIMULATE_NOISE_SEED = 20260823

# Column width for the marker_hint field in the live block.
HINT_COL_WIDTH = 10
ITEM_COL_WIDTH = 6

# --- GUI (--gui) ---------------------------------------------------------

# Prototyped in scratchpad/live_overlay.py as a legible schematic at real
# workspace scale.
GUI_SCALE_PX_PER_CM = 8
GUI_MARGIN_PX = 40
# Matches ROBOT_CIRCUMSCRIBED_R_CM in make_build_cards.py, rounded for
# drawing; this window is a placement aid, not a placement check.
GUI_ROBOT_START_RADIUS_CM = 3.5
# camera_service.py's on-demand frame REP socket replies within this long in
# normal operation; matches the prototype's RCVTIMEO.
GUI_FRAME_TIMEOUT_MS = 2000
GUI_QUIT_KEYS = (27, ord("q"))  # Esc, q


def wrapped_delta_deg(observed_deg: float, target_deg: float, period_deg: float) -> float:
    """Signed angular difference, wrapped into (-period/2, period/2].

    Works for any magnitude of the inputs, including multi-turn values from
    continuous camera tracking, because modulo arithmetic does not care how
    many full turns are folded into the raw number.
    """
    diff = observed_deg - target_deg
    half = period_deg / 2.0
    return ((diff + half) % period_deg) - half


def is_square(row: Dict[str, str]) -> bool:
    return abs(float(row["long_cm"]) - float(row["short_cm"])) <= SQUARE_TOL_CM


def load_sheet_rows(sheet_path: Path, build_id: str) -> List[Dict[str, str]]:
    with sheet_path.open() as fh:
        all_rows = list(csv.DictReader(fh))
    rows = [r for r in all_rows if r["build_id"] == build_id]
    if not rows:
        available = sorted({r["build_id"] for r in all_rows})
        sys.exit(f"build_id {build_id!r} not in {sheet_path}. "
                  f"Available: {', '.join(available)}")
    return rows


def load_axis_offsets(objects_path: str) -> Dict[str, float]:
    """marker_hint -> degrees added to raw marker theta to get world bearing.

    config/objects.yaml is the declared single source of truth for object
    width/depth. See MARKER_TO_BEARING_OFFSET_DEG above for the rule and how
    it was verified. Names not present in objects.yaml (should not happen
    for a valid sheet) get 0.0, i.e. treated as already a bearing.
    """
    import yaml

    with open(objects_path) as fh:
        data = yaml.safe_load(fh) or {}
    offsets = {}
    for name, cfg in (data.get("objects") or {}).items():
        shape = cfg.get("shape") or {}
        width = float(shape.get("width", 0.0))
        depth = float(shape.get("depth", 0.0))
        offsets[name] = 0.0 if depth >= width else MARKER_TO_BEARING_OFFSET_DEG
    return offsets


def apply_axis_mapping(
    raw_observed: Dict[str, Tuple[float, float, float]],
    axis_offset_deg: Dict[str, float],
) -> Dict[str, Tuple[float, float, float]]:
    """Turn raw marker (x, y, theta) into (x, y, world long-axis bearing).

    Every comparison and every checksum substitution in this script reads
    bearings out of the dict this returns, never a raw camera theta, so the
    marker-heading fix in load_axis_offsets only has to be applied once.
    """
    return {
        name: (x, y, theta + axis_offset_deg.get(name, 0.0))
        for name, (x, y, theta) in raw_observed.items()
    }


def simulated_observation(
    rows: List[Dict[str, str]],
    flip_hint: Optional[str],
    noise_cm: float,
    noise_deg: float,
) -> Dict[str, Tuple[float, float, float]]:
    """Headless stand-in for the camera: the sheet's own poses, perturbed.

    Bearings here are already world long-axis bearings (what the sheet
    stores), not raw marker theta, so this must NOT go through
    apply_axis_mapping. --simulate-raw-yaw builds a separate, raw-theta
    version on top of this one when the mapping itself needs exercising.
    """
    rng = random.Random(SIMULATE_NOISE_SEED)
    observed = {}
    for r in rows:
        x = float(r["centre_x_cm"])
        y = float(r["centre_y_cm"])
        theta = float(r["long_axis_bearing_deg"])
        if flip_hint is not None and r["marker_hint"] == flip_hint:
            theta += FLIP_ROTATION_DEG
        x += rng.uniform(-noise_cm, noise_cm)
        y += rng.uniform(-noise_cm, noise_cm)
        theta += rng.uniform(-noise_deg, noise_deg)
        observed[r["marker_hint"]] = (x, y, theta)
    return observed


def frame_request_address(camera_service_address: str) -> str:
    """The on-demand frame REP socket. camera_service.py always binds it at
    PUB port + 1 (its frame_bind_addr), so this is derived rather than a
    second flag that could drift from --camera-service.
    """
    host_part, _, port_str = camera_service_address.rpartition(":")
    return f"{host_part}:{int(port_str) + 1}"


class ServiceCameraSource:
    """Pulls Observations from a running camera_service over ZMQ.

    Only x/y/theta are used. The service always reports width/depth/height as
    0.0 (it only has pose from marker detection), and item sizes for the
    checksum come from the sheet regardless, so that is never read here.
    """

    def __init__(self, address: str) -> None:
        from robot_control.nodes.remote_observer import RemoteObserverNode

        self._node = RemoteObserverNode(address=address)
        if not self._node.start():
            raise RuntimeError(f"failed to connect to camera_service at {address}")

    def get(self) -> Dict[str, Tuple[float, float, float]]:
        obs = self._node.get()
        if obs is None:
            return {}
        return {name: (p.x, p.y, p.theta) for name, p in obs.objects.items()}

    def close(self) -> None:
        self._node.stop()


class DirectCameraSource:
    """Opens /dev/video and runs ArUco detection in-process. Fallback path
    for when camera_service is not running. Mirrors capture_to_xml.py.
    """

    def __init__(self, config_path: str, objects_path: str) -> None:
        from capture_to_xml import load_config
        from robot_control.camera import ArucoObserver
        from robot_control.nodes import CameraSensorNode

        camera_config, observer_config = load_config(config_path, objects_path)
        self._camera = CameraSensorNode(camera_config)
        if not self._camera.start():
            raise RuntimeError("failed to start camera")
        self._observer = ArucoObserver(observer_config)
        if not self._observer.start():
            self._camera.stop()
            raise RuntimeError("failed to start ArUco observer")

    def get(self) -> Dict[str, Tuple[float, float, float]]:
        obs = self._observer.get()
        if obs is None:
            return {}
        return {name: (p.x, p.y, p.theta) for name, p in obs.objects.items()}

    def close(self) -> None:
        self._observer.stop()
        self._camera.stop()


class LiveWindow:
    """--gui: a schematic canvas (red = current, green = target) plus the
    camera service's raw annotated feed, absorbed from the scratchpad
    prototype. Lazy cv2/numpy/zmq imports, so --simulate and plain terminal
    use never need a display or these packages.
    """

    def __init__(self, rows: List[Dict[str, str]], build_id: str,
                 frame_address: Optional[str]) -> None:
        import cv2
        import numpy as np

        self._cv2 = cv2
        self._np = np
        self._rows = rows
        self._title = f"check_build schematic {build_id}"
        self._w = int(WORKSPACE_W_CM * GUI_SCALE_PX_PER_CM) + 2 * GUI_MARGIN_PX
        self._h = int(WORKSPACE_H_CM * GUI_SCALE_PX_PER_CM) + 2 * GUI_MARGIN_PX

        self._frame_address = frame_address
        self._zmq = None
        self._ctx = None
        self._req = None
        if frame_address is not None:
            import zmq

            self._zmq = zmq
            self._ctx = zmq.Context()
            self._req = self._new_req_socket()

    def _new_req_socket(self):
        sock = self._ctx.socket(self._zmq.REQ)
        sock.connect(self._frame_address)
        sock.setsockopt(self._zmq.RCVTIMEO, GUI_FRAME_TIMEOUT_MS)
        return sock

    def _px(self, x: float, y: float) -> Tuple[int, int]:
        return (int(GUI_MARGIN_PX + x * GUI_SCALE_PX_PER_CM),
                int(GUI_MARGIN_PX + (WORKSPACE_H_CM - y) * GUI_SCALE_PX_PER_CM))

    def _poly_px(self, poly):
        return self._np.array([self._px(x, y) for x, y in poly], dtype=self._np.int32)

    def update(self, observed: Dict[str, Tuple[float, float, float]]) -> None:
        cv2, np = self._cv2, self._np
        canvas = np.full((self._h, self._w, 3), 255, np.uint8)
        cv2.rectangle(canvas, self._px(0, WORKSPACE_H_CM), self._px(WORKSPACE_W_CM, 0),
                      (0, 0, 0), 2)
        y_text = 22
        for r in self._rows:
            name = r["marker_hint"]
            tx, ty = float(r["centre_x_cm"]), float(r["centre_y_cm"])
            tb = float(r["long_axis_bearing_deg"])
            lc, sc = float(r["long_cm"]), float(r["short_cm"])
            cv2.polylines(canvas, [self._poly_px(_corners(tx, ty, lc, sc, tb))],
                         True, (0, 160, 0), 2)

            pose = observed.get(name)
            if pose is not None:
                ox, oy, otheta = pose
                period = SQUARE_SYMMETRY_DEG if is_square(r) else BAR_SYMMETRY_DEG
                dtheta = wrapped_delta_deg(otheta, tb, period)
                dx, dy = ox - tx, oy - ty
                ok = (abs(dx) <= DISPLAY_TOL_CM and abs(dy) <= DISPLAY_TOL_CM
                      and abs(dtheta) <= DISPLAY_TOL_DEG)
                color = (0, 180, 0) if ok else (0, 0, 230)
                overlay = canvas.copy()
                cv2.fillPoly(overlay, [self._poly_px(_corners(ox, oy, lc, sc, otheta))], color)
                canvas = cv2.addWeighted(overlay, 0.35, canvas, 0.65, 0)
                cv2.arrowedLine(canvas, self._px(ox, oy), self._px(tx, ty),
                               (200, 100, 0), 2, tipLength=0.15)
                msg = (f"{name}: dx{dx:+.1f} dy{dy:+.1f} dtheta{dtheta:+.0f}deg "
                       f"{'OK' if ok else ''}")
            else:
                color = (0, 0, 230)
                msg = f"{name}: NOT SEEN"
            cv2.putText(canvas, msg, (8, y_text), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
            y_text += 24

        head = self._rows[0]
        rx, ry = float(head["robot_start_x_cm"]), float(head["robot_start_y_cm"])
        cv2.circle(canvas, self._px(rx, ry), int(GUI_ROBOT_START_RADIUS_CM * GUI_SCALE_PX_PER_CM),
                   (180, 0, 180), 2)
        gx, gy = float(head["goal_x_cm"]), float(head["goal_y_cm"])
        cv2.drawMarker(canvas, self._px(gx, gy), (0, 165, 255), cv2.MARKER_STAR, 24, 3)
        cv2.imshow(self._title, canvas)

        if self._req is not None:
            try:
                self._req.send(b"vis")
                frame = cv2.imdecode(np.frombuffer(self._req.recv(), np.uint8), cv2.IMREAD_COLOR)
                if frame is not None:
                    cv2.imshow("check_build camera", frame)
            except self._zmq.error.ZMQError:
                self._req.close()
                self._req = self._new_req_socket()

    def wait_key(self, timeout_ms: int) -> bool:
        """Pumps the window event loop; returns True if the user asked to quit."""
        return (self._cv2.waitKey(timeout_ms) & 0xFF) in GUI_QUIT_KEYS

    def close(self) -> None:
        self._cv2.destroyAllWindows()
        if self._req is not None:
            self._req.close(linger=0)


def line_for_row(row: Dict[str, str], observed: Optional[Tuple[float, float, float]]) -> str:
    hint = row["marker_hint"]
    item = row["item"]
    prefix = f"  {hint:<{HINT_COL_WIDTH}} {item:<{ITEM_COL_WIDTH}}"
    if observed is None:
        return f"{prefix} MISSING (camera does not see it)"

    ox, oy, otheta = observed
    dx = ox - float(row["centre_x_cm"])
    dy = oy - float(row["centre_y_cm"])
    period = SQUARE_SYMMETRY_DEG if is_square(row) else BAR_SYMMETRY_DEG
    dtheta = wrapped_delta_deg(otheta, float(row["long_axis_bearing_deg"]), period)

    ok = abs(dx) <= DISPLAY_TOL_CM and abs(dy) <= DISPLAY_TOL_CM and abs(dtheta) <= DISPLAY_TOL_DEG
    tag = "OK" if ok else ".."
    flip_note = ""
    if not is_square(row) and abs(dtheta) >= FLIP_SUSPECT_DEG:
        flip_note = "  FLIPPED?"

    return (f"{prefix} dx={dx:+6.2f}cm dy={dy:+6.2f}cm "
            f"dtheta={dtheta:+6.1f}deg  {tag}{flip_note}")


def print_block(build_id: str, rows: List[Dict[str, str]],
                 observed: Dict[str, Tuple[float, float, float]], live: bool) -> None:
    if live:
        print("\x1b[2J\x1b[H", end="")  # plain ANSI clear, no curses
    print(f"check_build  {build_id}   {time.strftime('%H:%M:%S')}")
    print("-" * 64)
    for r in rows:
        print(line_for_row(r, observed.get(r["marker_hint"])))
    print("-" * 64)
    if live:
        print(f"OK band: +/-{DISPLAY_TOL_CM:.1f}cm, +/-{DISPLAY_TOL_DEG:.0f}deg (display "
              "only). Enter = run checksum. Ctrl+C = quit.")


def run_checksum(rows: List[Dict[str, str]],
                  observed: Dict[str, Tuple[float, float, float]]) -> Optional[str]:
    """Runs the contact checksum and prints the verdict.

    Returns "PASS", "MARGINAL", "FAIL", or None if the checksum could not be
    run at all (no checksum columns on the sheet, or an item not yet seen).
    """
    head = rows[0]
    if not all(c in head and head[c] != "" for c in CHECKSUM_COLUMNS):
        print("  no checksum columns on this sheet, cannot verify the build.")
        return None

    missing = [r["marker_hint"] for r in rows if r["marker_hint"] not in observed]
    if missing:
        print(f"  cannot checksum, camera does not currently see: {', '.join(missing)}")
        return None

    claimed = tuple(int(head[c]) for c in CHECKSUM_COLUMNS)
    built_rows = []
    for r in rows:
        ox, oy, otheta = observed[r["marker_hint"]]
        built = dict(r)
        built["centre_x_cm"] = ox
        built["centre_y_cm"] = oy
        built["long_axis_bearing_deg"] = otheta
        built_rows.append(built)
    actual = scene_contact_counts(built_rows)

    # Both sides sum to CONTACTS_PER_OBJECT, so any disagreement is a
    # reallocation and L1 distance double-counts each contact that moved.
    moved = sum(abs(a - c) for a, c in zip(actual, claimed)) // 2

    print(f"  sheet claims    reachable/cutoff/collision = "
          f"{claimed[0]}/{claimed[1]}/{claimed[2]}")
    print(f"  built measures  reachable/cutoff/collision = "
          f"{actual[0]}/{actual[1]}/{actual[2]}")
    print(f"  {moved} of {CONTACTS_PER_OBJECT} contacts moved class")
    if moved == 0:
        verdict = "PASS"
        print("  PASS: built scene matches the sheet exactly.")
    elif moved <= CHECKSUM_TIE_TOLERANCE_CONTACTS:
        verdict = "MARGINAL"
        print("  MARGINAL: pose sits near a class boundary. Nudge it or accept it.")
    else:
        verdict = "FAIL"
        print("  FAIL: the physical scene is a different scene from the measured one.")
    return verdict


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sheet", required=True, type=Path, help="build sheet CSV")
    ap.add_argument("--build-id", required=True, help="build_id to check")
    ap.add_argument("--simulate", action="store_true",
                     help="no camera; self-test against the sheet's own poses")
    ap.add_argument("--flip", metavar="ITEM", default=None,
                     help="--simulate only: rotate this marker_hint's bearing 90 deg")
    ap.add_argument("--noise-cm", type=float, default=0.0,
                     help="--simulate only: uniform position noise, +/- this many cm")
    ap.add_argument("--noise-deg", type=float, default=0.0,
                     help="--simulate only: uniform bearing noise, +/- this many deg")
    ap.add_argument("--simulate-raw-yaw", action="store_true",
                     help="--simulate only: self-test the marker-theta-to-bearing "
                          "mapping instead of feeding sheet bearings directly")
    ap.add_argument("--camera-service", metavar="ADDRESS",
                     default=DEFAULT_CAMERA_SERVICE_ADDRESS,
                     help=f"ZMQ address of a running camera_service "
                          f"(default: {DEFAULT_CAMERA_SERVICE_ADDRESS})")
    ap.add_argument("--direct-camera", action="store_true",
                     help="bypass camera_service and open the camera directly "
                          "(fallback for when the service is not running)")
    ap.add_argument("--config", default="config/real.yaml",
                     help="real robot config, for --direct-camera")
    ap.add_argument("--objects", default="config/objects.yaml",
                     help="object marker definitions: shapes for the marker-theta "
                          "mapping, and (for --direct-camera) marker IDs")
    ap.add_argument("--auto", type=float, default=None, metavar="SECONDS",
                     help="run the checksum automatically every SECONDS, "
                          "instead of waiting for Enter")
    ap.add_argument("--gui", action="store_true",
                     help="also open a live OpenCV schematic + camera-feed window. "
                          "Needs DISPLAY set by the caller. Closes automatically, "
                          "printing the verdict, once a checksum comes back PASS; "
                          "requires --auto, since Enter-to-check is a terminal "
                          "affordance the GUI window does not have focus for")
    args = ap.parse_args()

    if not args.simulate:
        if args.flip is not None:
            ap.error("--flip only makes sense with --simulate")
        if args.noise_cm or args.noise_deg:
            ap.error("--noise-cm/--noise-deg only make sense with --simulate")
        if args.simulate_raw_yaw:
            ap.error("--simulate-raw-yaw only makes sense with --simulate")
    if args.gui and args.auto is None:
        ap.error("--gui requires --auto SECONDS")
    return args


def main() -> None:
    args = parse_args()
    rows = load_sheet_rows(args.sheet, args.build_id)
    hints = {r["marker_hint"] for r in rows}
    if args.flip is not None and args.flip not in hints:
        sys.exit(f"--flip {args.flip!r} is not a marker_hint in {args.build_id}: "
                  f"{sorted(hints)}")

    axis_offsets: Dict[str, float] = {}
    if not args.simulate or args.simulate_raw_yaw:
        axis_offsets = load_axis_offsets(args.objects)

    if args.simulate:
        observed = simulated_observation(rows, args.flip, args.noise_cm, args.noise_deg)
        if args.simulate_raw_yaw:
            # Invert the mapping to fabricate a "raw marker theta", then run
            # it back through the real mapping. dtheta should land on 0.
            raw = {name: (x, y, theta - axis_offsets.get(name, 0.0))
                   for name, (x, y, theta) in observed.items()}
            observed = apply_axis_mapping(raw, axis_offsets)
        print_block(args.build_id, rows, observed, live=False)
        run_checksum(rows, observed)
        return

    source = DirectCameraSource(args.config, args.objects) if args.direct_camera \
        else ServiceCameraSource(args.camera_service)

    window = None
    if args.gui:
        frame_address = None if args.direct_camera else frame_request_address(args.camera_service)
        window = LiveWindow(rows, args.build_id, frame_address)

    last_checksum_time = time.time()
    verdict = None  # defined before the loop so a Ctrl+C on the first tick is safe
    try:
        while True:
            raw = source.get()
            observed = apply_axis_mapping(raw, axis_offsets)
            print_block(args.build_id, rows, observed, live=True)
            if window is not None:
                window.update(observed)

            verdict = None
            if args.auto is not None and time.time() - last_checksum_time >= args.auto:
                verdict = run_checksum(rows, observed)
                last_checksum_time = time.time()

            if args.gui:
                if window.wait_key(int(REFRESH_PERIOD_S * 1000)):
                    print("closed by user")
                    break
                if verdict == "PASS":
                    break
            elif select.select([sys.stdin], [], [], REFRESH_PERIOD_S)[0]:
                sys.stdin.readline()
                run_checksum(rows, observed)
                last_checksum_time = time.time()
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        if window is not None:
            window.close()
        source.close()

    if args.gui and verdict == "PASS":
        sys.exit(0)


if __name__ == "__main__":
    main()
