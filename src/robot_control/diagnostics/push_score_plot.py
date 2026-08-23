"""Show what the ranker sees before the robot commits to a push.

The search picks a first push from HY5U's score over 60 contact points by 5
depths. That number decides everything downstream and nothing displays it, so
a run that picks an odd push looks identical to one that picks a good one.

This renders the grid and blocks until the window closes, which makes it an
inspection gate rather than a log line: the robot does not move until you have
looked. Off by default, since blocking is the whole point and an unattended run
must not stop.

When ``input_masks`` is given, a strip of the model's actual input channels is
drawn along the bottom. Those five 64x64 rasters are what the network sees, not
the scene coordinates: everything else in this figure is a human-readable stand
in for them, and only the strip shows what was really fed in.

Three panels. Left is the contact points in workspace coordinates, coloured by
their best score across depths, with every obstacle drawn at its real angle so
the geometry is checkable by eye. Middle is the raw 60 x 5 grid, which shows
whether the model is discriminating at all or returning a flat sheet. Right
lists the numbers the model was actually given: a picture of the output without
the input cannot tell you whether a bad score came from a bad model or a bad
scene.

To verify:
  add --show-push-scores to any run_namo.py invocation
"""

from __future__ import annotations

import math
from typing import Any, Dict, Optional, Sequence, Set, Tuple

# ─── Named constants ────────────────────────────────────────────────────

# Contact geometry, matching namo_push_controller.cpp:163.
POINTS_PER_FACE = 15
# Mirrors config/controller.yaml push.edge_offset_margin_cm and namo_cpp's
# planning.wavefront_edge_offset_margin = 0.01 m.
PUSH_OFFSET_MARGIN_CM = 1.0

# Anything under this spread across the whole grid means the model is not
# separating candidates, which is worth seeing in the title rather than
# squinting at a colourbar.
FLAT_GRID_SPREAD = 0.02


def show_push_scores(
    grid: Sequence[Sequence[float]],
    object_pose: Tuple[float, float, float],
    object_extents_cm: Tuple[float, float],
    robot_xy_cm: Tuple[float, float],
    goal_xy_cm: Tuple[float, float],
    other_objects: Optional[Dict[str, Tuple[float, float, float, float, float, bool]]] = None,
    input_masks: Optional[Any] = None,
    mask_names: Optional[Sequence[str]] = None,
    blocked_edges: Optional[Set[int]] = None,
    workspace_cm: Tuple[float, float] = (49.0, 77.5),
    robot_inflation_radius_cm: float = 3.5,
    title: str = "",
) -> None:
    """Draw the score grid and block until the window is closed.

    ``object_pose`` is (x_cm, y_cm, yaw_deg); ``object_extents_cm`` is the FULL
    (x_extent, y_extent), matching ObjectPose.depth and ObjectPose.width.
    ``other_objects`` maps a name to (cx, cy, yaw_deg, x_extent, y_extent,
    is_static) in cm and degrees, the same fields MuJoCo gets. Rotation is drawn
    rather than collapsed to a bounding box, and movables are drawn too: a
    picture that hides half the scene is worse than none.

    Never raises. A plotting failure must not stop the robot, so anything that
    goes wrong prints and returns.
    """
    try:
        import numpy as np
        import matplotlib
        # Render off-screen and gate on stdin instead of opening a window. The
        # runtime is a Qt process: matplotlib's qtagg backend attaches to that
        # QApplication without owning its event loop, so show() blocks and
        # nothing ever paints, and TkAgg refuses to load at all once Qt is up
        # ("Cannot load backend 'TkAgg' ... as 'qt' is currently running").
        # A file plus a prompt works from a terminal, over ssh, and headless.
        matplotlib.use("Agg", force=True)
        import matplotlib.pyplot as plt
        from matplotlib.patches import Rectangle
        from matplotlib.transforms import Affine2D

        from robot_control.controller.edge_points import get_edge_point
        from robot_control.core.types import ObjectPose

        scores = np.array(grid, dtype=float)
        ox, oy, oyaw = object_pose
        ex, ey = object_extents_cm
        standoff = robot_inflation_radius_cm + PUSH_OFFSET_MARGIN_CM
        obj = ObjectPose(x=ox, y=oy, theta=oyaw, depth=ex, width=ey)
        points = [
            get_edge_point(obj, edge_idx=i, standoff=standoff,
                           points_per_face=POINTS_PER_FACE).position
            for i in range(scores.shape[0])
        ]
        best = scores.max(axis=1)
        blocked = blocked_edges or set()

        masks = None if input_masks is None else np.asarray(input_masks)
        strip = 0 if masks is None else masks.shape[0] + 1   # channels plus overlay

        if strip:
            fig = plt.figure(figsize=(17.5, 11.5))
            cells = fig.add_gridspec(2, 3, width_ratios=[1.15, 1.0, 0.72],
                                     height_ratios=[2.5, 1.0])
        else:
            fig = plt.figure(figsize=(17.5, 8))
            cells = fig.add_gridspec(1, 3, width_ratios=[1.15, 1.0, 0.72])
        ax, bx, cx = (fig.add_subplot(cells[0, 0] if strip else cells[0]),
                      fig.add_subplot(cells[0, 1] if strip else cells[1]),
                      fig.add_subplot(cells[0, 2] if strip else cells[2]))
        ax.add_patch(Rectangle((0, 0), *workspace_cm, fill=False, ec="0.4", lw=2))
        for name, (px, py, pyaw, pex, pey, is_static) in (other_objects or {}).items():
            turn = Affine2D().rotate_deg_around(px, py, pyaw) + ax.transData
            ax.add_patch(Rectangle(
                (px - pex / 2, py - pey / 2), pex, pey, transform=turn,
                fc="0.75" if is_static else "lightsteelblue",
                ec="0.45", lw=1.3))
            ax.annotate(name, (px, py), ha="center", va="center",
                        fontsize=7.5, color="0.25")
        spin = Affine2D().rotate_deg_around(ox, oy, oyaw) + ax.transData
        ax.add_patch(Rectangle((ox - ex / 2, oy - ey / 2), ex, ey,
                               fc="gold", ec="peru", lw=1.5, transform=spin))

        live = [i for i in range(len(points)) if i not in blocked]
        if live:
            sc = ax.scatter([points[i][0] for i in live], [points[i][1] for i in live],
                            c=best[live], cmap="viridis", s=95,
                            edgecolors="k", linewidths=0.4, zorder=5)
            fig.colorbar(sc, ax=ax, shrink=0.75, label="P(opens region)")
            top = max(live, key=lambda i: best[i])
            ax.scatter(*points[top], s=300, facecolors="none",
                       edgecolors="red", lw=2.5, zorder=6)
            ax.annotate(f"best: edge {top}\n{best[top]:.3f}", points[top],
                        textcoords="offset points", xytext=(12, 10),
                        color="red", fontsize=10, weight="bold")
        if blocked:
            ax.scatter([points[i][0] for i in blocked], [points[i][1] for i in blocked],
                       marker="x", c="crimson", s=55, zorder=4,
                       label=f"{len(blocked)} unreachable")
            ax.legend(loc="upper left", fontsize=9)

        ax.plot(*robot_xy_cm, "s", ms=13, color="tab:blue", zorder=6)
        ax.annotate("robot", robot_xy_cm, textcoords="offset points",
                    xytext=(9, -14), color="tab:blue")
        ax.plot(*goal_xy_cm, "*", ms=22, color="tab:green", zorder=6)
        ax.annotate("goal", goal_xy_cm, textcoords="offset points",
                    xytext=(9, 4), color="tab:green")
        ax.set_xlim(-3, workspace_cm[0] + 3); ax.set_ylim(-3, workspace_cm[1] + 3)
        ax.set_aspect("equal"); ax.set_xlabel("x (cm)"); ax.set_ylabel("y (cm)")
        ax.set_title(title or "first-push score, best over depths")

        shown = scores.astype(float).copy()
        for i in blocked:
            if 0 <= i < shown.shape[0]:
                shown[i, :] = np.nan          # filtered edges are not candidates
        im = bx.imshow(shown.T, aspect="auto", cmap="viridis", origin="lower",
                       extent=[0, scores.shape[0], 0.5, scores.shape[1] + 0.5])
        im.cmap.set_bad("0.85")
        bx.set_xlabel("edge index"); bx.set_ylabel("push_steps (depth + 1)")
        bx.set_yticks(range(1, scores.shape[1] + 1))
        spread = float(scores.max() - scores.min())
        flat = "  FLAT, model is not separating" if spread < FLAT_GRID_SPREAD else ""
        bx.set_title(f"score grid {scores.shape[0]} x {scores.shape[1]}, grey = filtered\n"
                     f"range {scores.min():.3f} .. {scores.max():.3f}"
                     f"  spread {spread:.3f}{flat}")
        fig.colorbar(im, ax=bx, shrink=0.75)

        cx.axis("off")
        rows = ["MODEL INPUTS", "",
                f"robot   ({robot_xy_cm[0]:6.1f}, {robot_xy_cm[1]:6.1f}) cm",
                f"goal    ({goal_xy_cm[0]:6.1f}, {goal_xy_cm[1]:6.1f}) cm",
                f"arena   {workspace_cm[0]:.1f} x {workspace_cm[1]:.1f} cm",
                f"robot r {robot_inflation_radius_cm:.2f} cm (max half-extent)",
                f"standoff{standoff:6.2f} cm", "",
                "scored object", f"  ({ox:6.1f}, {oy:6.1f})  yaw {oyaw:6.1f}",
                f"  {ex:.1f} x {ey:.1f} cm", ""]
        if other_objects:
            rows.append("other obstacles")
            for name, (px, py, pyaw, pex, pey, is_static) in sorted(other_objects.items()):
                rows.append(f"  {name}{' [static]' if is_static else ''}")
                rows.append(f"    ({px:6.1f}, {py:6.1f}) yaw {pyaw:6.1f}")
                rows.append(f"    {pex:.1f} x {pey:.1f} cm")
            rows.append("")
        rows += ["reachability filter", f"  {len(blocked)} of {scores.shape[0]} blocked",
                 f"  {scores.shape[0] - len(blocked)} searchable", "",
                 "top reachable pushes", ""]
        for i in sorted(live, key=lambda i: -best[i])[:6]:
            rows.append(f"  edge {i:2d}   {best[i]:.4f}")
        cx.text(0.0, 1.0, "\n".join(rows), va="top", family="monospace", fontsize=9)

        if strip:
            names = list(mask_names or [f"ch{i}" for i in range(masks.shape[0])])
            inner = cells[1, :].subgridspec(1, strip, wspace=0.12)
            for i in range(masks.shape[0]):
                mx = fig.add_subplot(inner[i])
                mx.imshow(masks[i], cmap="gray", origin="lower", vmin=0, vmax=1)
                mx.set_xticks([]); mx.set_yticks([])
                mx.set_title(f"{names[i] if i < len(names) else i}\n"
                             f"{int((masks[i] > 0.5).sum())} px",
                             fontsize=8.5)
            # One composite so the channels can be read against each other.
            over = np.zeros(masks.shape[1:] + (3,))
            if masks.shape[0] >= 5:
                over[..., 0] = np.clip(masks[0] * 0.6 + masks[2], 0, 1)
                over[..., 1] = np.clip(masks[3] * 0.55 + masks[4], 0, 1)
                over[..., 2] = np.clip(masks[1], 0, 1)
            mx = fig.add_subplot(inner[masks.shape[0]])
            mx.imshow(over, origin="lower"); mx.set_xticks([]); mx.set_yticks([])
            mx.set_title("overlaid\nMODEL INPUT", fontsize=8.5, weight="bold")

        fig.tight_layout()
        import os, sys, tempfile
        out = os.path.join(tempfile.gettempdir(), "push_scores_latest.png")
        fig.savefig(out, dpi=110)
        plt.close(fig)
        print(f"\n[PushScores] {out}", flush=True)
        # Gate on stdin. Skipped when stdin is not a terminal, so a detached or
        # piped run keeps the picture without hanging forever on a read nobody
        # will answer.
        if sys.stdin is not None and sys.stdin.isatty():
            try:
                input("[PushScores] press Enter to let the robot continue... ")
            except (EOFError, KeyboardInterrupt):
                print("\n[PushScores] continuing", flush=True)
        else:
            print("[PushScores] stdin is not a terminal, not waiting", flush=True)
    except Exception as exc:  # pragma: no cover - display-dependent
        print(f"[PushScores] plot skipped: {exc!r}", flush=True)
