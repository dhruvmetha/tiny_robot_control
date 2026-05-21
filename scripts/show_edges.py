"""Render the current scene with every movable object's edge points labeled.

Use this to visually pick edge indices when authoring
config/manual_primitives.yaml for ``--strategy manual_primitives``.

Workflow:
    1. Place the workspace + obstacles + robot as you intend to run them.
    2. ``python scripts/show_edges.py --config config/real.yaml \\
           --camera-service tcp://localhost:5556 --output /tmp/edges.png``
       (Or pass ``--xml PATH`` if you already have a captured scene XML.)
    3. Open /tmp/edges.png. Each movable object shows numbered dots
       around its perimeter — the number is the ``edge_idx`` you'd
       put in the YAML.
    4. Write the manual_primitives YAML using those indices.

Notes:
    - Edge ordering matches ``edge_points.generate_edge_points`` which
      mirrors namo_cpp's ``generate_rectangular_edge_points``. With the
      production default of ``points_per_face=15`` you get 60 edges per
      object (indices 0..59).
    - Even/odd indices in [0..29] are top/bottom face samples; even/odd
      in [30..59] are right/left face samples. See edge_points.py.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import yaml

from robot_control.camera.workspace import WORKSPACE_HEIGHT_CM, WORKSPACE_WIDTH_CM
from robot_control.controller.edge_points import generate_edge_points
from robot_control.core.types import ObjectPose, Observation


# Colours per face for quick visual recognition. Tuples are (RGBA) for
# matplotlib markers; the order matches edge_points face_idx.
_FACE_COLOURS = {
    0: "tab:blue",     # Top (+Y)
    1: "tab:red",      # Bottom (-Y)
    2: "tab:green",    # Right (+X)
    3: "tab:orange",   # Left (-X)
}


# ----------------------------------------------------------------- live capture

def capture_live_observation(
    config_path: str,
    camera_service: str,
    stable_secs: float = 1.5,
) -> Observation:
    """Connect to the camera_service, wait for one stable observation, return it.

    No-thrills version of the loop in capture_to_xml.py — just receive a
    couple of frames and return whatever the observer's latest is. The
    camera_service publishes Observations directly so we don't need ArUco
    code here.
    """
    from robot_control.nodes.remote_observer import RemoteObserverNode

    print(f"[show_edges] Connecting to camera_service at {camera_service} ...",
          flush=True)
    observer = RemoteObserverNode(address=camera_service)
    if not observer.start():
        raise RuntimeError(
            f"Failed to connect to camera_service at {camera_service}. "
            "Is it running?"
        )

    try:
        deadline = time.time() + 10.0
        obs = None
        while time.time() < deadline:
            obs = observer.get()
            if obs is not None:
                break
            time.sleep(0.1)
        if obs is None:
            raise RuntimeError(
                "camera_service is reachable but published no observation "
                "within 10 s — is the camera actually streaming?"
            )

        # Soak a little extra so the observation is a stable frame rather
        # than the very first one (which can carry transient pose noise).
        time.sleep(stable_secs)
        obs = observer.get()
        return obs
    finally:
        observer.stop()


# ------------------------------------------------------------------ XML loader

def observation_from_xml(xml_path: str) -> Observation:
    """Parse a bridge-generated MuJoCo XML and reconstruct an Observation.

    Walks ``<worldbody><body>`` tags, distinguishes static walls (geom of
    type box with no joint) from movable obstacles (body with a free
    joint), pulls the robot position from the body named ``robot``, and
    converts MuJoCo metres + half-extents back to centimetres + full
    extents to match the Observation schema.

    Only enough fidelity to drive the visualizer — *not* a general MuJoCo
    parser.
    """
    import xml.etree.ElementTree as ET

    root = ET.parse(xml_path).getroot()
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise ValueError(f"{xml_path}: no <worldbody> element")

    objects: Dict[str, ObjectPose] = {}
    robot_x = robot_y = 0.0
    robot_theta = 0.0
    goal_x = goal_y = None

    for body in worldbody.findall("body"):
        body_name = body.get("name", "")
        has_free_joint = any(
            (jt.get("type") == "free") for jt in body.findall("joint")
        )

        if body_name == "robot":
            # The bridge-generated XML puts the robot's pose on the inner
            # <geom> rather than the <body>. MuJoCo's <geom> ``euler``
            # attribute defaults to degrees (mujoco compiler ``angle``
            # default), so don't math.degrees() it.
            geom = body.find("geom")
            if geom is not None:
                pos = geom.get("pos", "0 0 0").split()
                try:
                    robot_x = float(pos[0]) * 100.0
                    robot_y = float(pos[1]) * 100.0
                except (IndexError, ValueError):
                    pass
                euler = geom.get("euler", "0 0 0").split()
                try:
                    robot_theta = float(euler[2])
                except (IndexError, ValueError):
                    robot_theta = 0.0
            continue

        if body_name == "walls":
            # Plain workspace walls — represented as geoms under one body
            # with no individual <body> per wall. Loop their geoms.
            for geom in body.findall("geom"):
                _add_geom_as_object(geom, objects, is_static=True)
            continue

        # Per-body geom (movable obstacles OR named internal walls).
        geom = body.find("geom")
        if geom is None:
            continue
        is_static = not has_free_joint
        _add_geom_as_object(geom, objects, is_static=is_static, fallback_name=body_name)

    print(
        f"[show_edges] Loaded scene: robot=({robot_x:.1f},{robot_y:.1f}) cm "
        f"theta={robot_theta:.1f}°, {sum(1 for o in objects.values() if not o.is_static)} "
        f"movable, {sum(1 for o in objects.values() if o.is_static)} static",
        flush=True,
    )

    return Observation(
        robot_x=robot_x,
        robot_y=robot_y,
        robot_theta=robot_theta,
        objects=objects,
        timestamp=0.0,
        goal_x=goal_x,
        goal_y=goal_y,
    )


def _add_geom_as_object(
    geom_elem,
    dest: Dict[str, ObjectPose],
    is_static: bool,
    fallback_name: Optional[str] = None,
) -> None:
    name = geom_elem.get("name") or fallback_name
    if not name:
        return
    pos = geom_elem.get("pos", "0 0 0").split()
    size = geom_elem.get("size", "0 0 0").split()
    euler = geom_elem.get("euler", "0 0 0").split()
    try:
        x_cm = float(pos[0]) * 100.0
        y_cm = float(pos[1]) * 100.0
        # MuJoCo geom size for type=box is half-extent; convert back to full extents.
        depth_cm = float(size[0]) * 200.0  # X dimension
        width_cm = float(size[1]) * 200.0  # Y dimension
        # MuJoCo's geom euler attribute defaults to degrees (compiler
        # `angle` default), so do NOT math.degrees() the value.
        theta_deg = float(euler[2]) if len(euler) > 2 else 0.0
    except (IndexError, ValueError):
        return
    dest[name] = ObjectPose(
        x=x_cm,
        y=y_cm,
        theta=theta_deg,
        width=width_cm,
        depth=depth_cm,
        height=0.0,
        is_static=is_static,
    )


# -------------------------------------------------------------------- rendering

def draw_scene(
    ax,
    obs: Observation,
    workspace_w: float,
    workspace_h: float,
    points_per_face: int,
    standoff_cm: float,
    show_indices_for: Optional[List[str]],
    label_every: int,
) -> None:
    """Draw workspace + objects + edge labels onto ``ax``."""
    # Workspace boundary
    ax.add_patch(mpatches.Rectangle(
        (0, 0), workspace_w, workspace_h,
        fill=False, edgecolor="black", linewidth=1.5,
    ))

    for name, obj in obs.objects.items():
        rect = _rotated_box_patch(obj)
        if obj.is_static:
            rect.set_facecolor("#cccccc")
            rect.set_edgecolor("#888888")
            ax.add_patch(rect)
            ax.text(obj.x, obj.y, name, ha="center", va="center",
                    fontsize=6, color="#555555", zorder=5)
            continue

        rect.set_facecolor("#fff0b3")
        rect.set_edgecolor("#cc8800")
        ax.add_patch(rect)
        ax.text(obj.x, obj.y, name, ha="center", va="center",
                fontsize=8, fontweight="bold", color="#664400", zorder=5)

        if show_indices_for is not None and name not in show_indices_for:
            continue

        edge_points = generate_edge_points(obj, standoff_cm, points_per_face)
        for ep in edge_points:
            if ep.edge_idx % label_every != 0:
                continue
            colour = _FACE_COLOURS.get(ep.face_idx, "black")
            ax.plot(ep.position[0], ep.position[1], marker="o",
                    color=colour, markersize=4, zorder=6)
            ax.annotate(
                str(ep.edge_idx),
                xy=(ep.position[0], ep.position[1]),
                xytext=(3, 3), textcoords="offset points",
                fontsize=7, color=colour, zorder=7,
            )

    # Robot
    ax.add_patch(mpatches.Circle(
        (obs.robot_x, obs.robot_y), 3.0,
        fill=True, facecolor="#4a90e2", edgecolor="black", linewidth=1.0,
    ))
    # Heading arrow
    hx = obs.robot_x + 5.0 * math.cos(math.radians(obs.robot_theta))
    hy = obs.robot_y + 5.0 * math.sin(math.radians(obs.robot_theta))
    ax.annotate(
        "", xy=(hx, hy), xytext=(obs.robot_x, obs.robot_y),
        arrowprops=dict(arrowstyle="->", color="black", lw=1.5),
    )

    if obs.goal_x is not None and obs.goal_y is not None:
        ax.plot(obs.goal_x, obs.goal_y, marker="X", color="red",
                markersize=12, markeredgecolor="black", markeredgewidth=1.0,
                zorder=8)
        ax.annotate("goal", xy=(obs.goal_x, obs.goal_y),
                    xytext=(6, -10), textcoords="offset points",
                    fontsize=9, color="red", fontweight="bold")

    ax.set_xlim(-2, workspace_w + 2)
    ax.set_ylim(-2, workspace_h + 2)
    ax.set_aspect("equal")
    ax.set_xlabel("X (cm)")
    ax.set_ylabel("Y (cm)")
    ax.set_title(
        f"Edge indices for movable objects "
        f"(points_per_face={points_per_face}, standoff={standoff_cm:.1f} cm)"
    )

    # Legend by face
    legend_handles = [
        mpatches.Patch(color=_FACE_COLOURS[i], label=label)
        for i, label in [(0, "Top (+Y)"), (1, "Bottom (-Y)"),
                         (2, "Right (+X)"), (3, "Left (-X)")]
    ]
    ax.legend(handles=legend_handles, loc="upper right", fontsize=8)


def _rotated_box_patch(obj: ObjectPose) -> mpatches.Rectangle:
    """Build a matplotlib Rectangle representing an ObjectPose, rotated about its centre."""
    # Rectangle is positioned by its lower-left corner. We want it centred
    # on (obj.x, obj.y) with depth (X extent) and width (Y extent), then
    # rotated by obj.theta about its centre.
    rect = mpatches.Rectangle(
        (obj.x - obj.depth / 2.0, obj.y - obj.width / 2.0),
        obj.depth, obj.width,
        angle=obj.theta,
        rotation_point="center",
        linewidth=1.2,
    )
    return rect


# ------------------------------------------------------------------ entry point

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    source = p.add_mutually_exclusive_group(required=True)
    source.add_argument("--config", help="Path to real.yaml (live capture).")
    source.add_argument("--xml", help="Path to a pre-captured MuJoCo XML.")
    p.add_argument(
        "--camera-service", default=None,
        help="ZMQ address of the camera_service (e.g. tcp://localhost:5556). "
             "Required with --config.",
    )
    p.add_argument(
        "--output", "-o", default="/tmp/edges.png",
        help="Where to write the rendered PNG (default: /tmp/edges.png).",
    )
    p.add_argument(
        "--show", action="store_true",
        help="Pop up an interactive matplotlib window in addition to writing the PNG.",
    )
    p.add_argument(
        "--points-per-face", type=int, default=15, choices=[1, 3, 15],
        help="Sample density per face (default: 15 → 60 edges per object, matching production).",
    )
    p.add_argument(
        "--standoff", type=float, default=4.0,
        help="Approach standoff distance in cm — pushes the labelled dots out from the object face.",
    )
    p.add_argument(
        "--label-every", type=int, default=1,
        help="Label every N-th edge (default 1 = every edge). Use e.g. 5 to thin the labels.",
    )
    p.add_argument(
        "--only", nargs="+", default=None,
        help="Restrict edge labelling to these object names (still draws every object).",
    )
    return p.parse_args()


def main() -> int:
    args = parse_args()

    if args.config:
        if not args.camera_service:
            print("Error: --camera-service is required with --config", file=sys.stderr)
            return 2
        try:
            obs = capture_live_observation(args.config, args.camera_service)
        except Exception as exc:
            print(f"[show_edges] Live capture failed: {exc}", file=sys.stderr)
            return 1
    else:
        try:
            obs = observation_from_xml(args.xml)
        except Exception as exc:
            print(f"[show_edges] XML load failed: {exc}", file=sys.stderr)
            return 1

    workspace_w = WORKSPACE_WIDTH_CM
    workspace_h = WORKSPACE_HEIGHT_CM

    fig, ax = plt.subplots(figsize=(workspace_w / 6.0, workspace_h / 6.0), dpi=120)
    draw_scene(
        ax=ax,
        obs=obs,
        workspace_w=workspace_w,
        workspace_h=workspace_h,
        points_per_face=args.points_per_face,
        standoff_cm=args.standoff,
        show_indices_for=args.only,
        label_every=max(1, args.label_every),
    )
    fig.tight_layout()

    out_path = Path(args.output)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    print(f"[show_edges] wrote {out_path}", flush=True)

    if args.show:
        plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())
