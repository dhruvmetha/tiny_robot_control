"""The one region boundary the robot is currently trying to open.

The planner normally re-derives everything on each replan: region graph, next
region, blocking object, target points. Executing one physical push at a time
makes that wrong -- a setup push, which by definition does not open anything,
can leave the next replan aimed at a different boundary, stranding the work
just performed.

This record is what makes a subproblem persist. It is created once from
``select_boundary_from_xml``, held until the boundary opens or is exhausted,
and passed to ``solve_boundary_from_xml`` on every replan in between.

Two identity rules matter, and both are the opposite of the obvious choice:

* **Points, not a region label.** Region labels are ordinal -- a rank over
  lexicographic cell order -- so they renumber whenever a push re-partitions
  free space. The frozen point list is the boundary's real identity, and the
  success bar with it.
* **Real object ids, not simulator ids.** ``obstacle_N_movable`` is a rank over
  the movable objects present in one observation, so a marker dropping out of a
  single camera frame shifts every name after it. The simulator id is resolved
  fresh from the bridge's mapping each time a scene is generated.

Schema follows MODEL_GUIDED_RO_INTEGRATION.local.md §6.1/§8.5, with one
deviation: blockers are a tuple, not a single id, because a boundary can be
blocked by more than one object and ``select_boundary_from_xml`` returns a set.

Frozen, so an update is an explicit new value rather than a mutation nobody
sees. Paths never appear in the record, so a run directory can be moved without
the rewriting that other artefacts in this repo need.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

from robot_control.planner.search_config import DEFAULT_EXEC_MODE, REACTIVE_EXEC_MODE

# v2 adds last_settled_dispatch_epoch. A v1 record loads with it unset, which
# reads as "never settled", so the first settlement after an upgrade counts
# normally. Reading old records matters: a schema bump that refused them would
# throw away an active boundary mid-deployment.
SCHEMA_VERSION = 2
SUPPORTED_SCHEMA_VERSIONS = (1, 2)

# The file lives at run level, not iteration level: creating iter_N+1 rewrites
# status.json from a fixed literal and regenerates scene_after/ wholesale, so
# anything held there is lost at the boundary.
ACTIVE_TARGET_FILENAME = "active_region_opening.json"

# Target-id fingerprint. Rounding to a micron is far below both the sampler's
# grid and anything physically meaningful, so it cannot merge two genuinely
# different targets; it makes the id survive a change in how points are
# formatted on disk. Twelve hex characters collide only after billions of
# distinct point sets, and still fit in a log line next to the status.
TARGET_ID_PRECISION_M = 6
TARGET_ID_DIGEST_CHARS = 12
TARGET_ID_PREFIX = "ro-"

STATUS_ACTIVE = "active"
STATUS_OPENED = "opened"
STATUS_EXHAUSTED = "exhausted"
TERMINAL_STATUSES = (STATUS_OPENED, STATUS_EXHAUSTED)


@dataclass(frozen=True)
class RegionOpeningTarget:
    """One region-opening subproblem, stable across physical pushes."""

    # The success criterion, frozen at selection. Simulator metres -- the frame
    # count_reachable_points takes. The _m suffix is load-bearing: everything
    # else this repo writes to disk is centimetres.
    target_samples_m: Tuple[Tuple[float, float], ...]
    # Durable identity, in real (camera/marker) naming.
    blocker_real_ids: Tuple[str, ...]
    # The bar this target was chosen under, so a later config change cannot
    # silently re-grade a subproblem already in flight.
    open_fraction: float

    target_id: str = ""
    status: str = STATUS_ACTIVE
    selected_iteration: int = 0
    last_iteration: int = 0
    # Diagnostics only. Labels are never identity.
    source_region_path: Tuple[str, ...] = ()
    sample_seed: int = 42
    scale_factor: float = 1.0
    # Pushes physically attempted and failed against THIS boundary. Scoped here
    # rather than to the planner instance so they survive a replan, and are
    # discarded with the subproblem rather than on the next unrelated success.
    failed_pushes: Tuple[Tuple[str, int], ...] = ()
    physical_pushes_attempted: int = 0
    # The dispatch timestamp of the last push counted against this subproblem.
    # Settlement is replayable: re-running an iteration with --allow-overwrite,
    # or resuming after a crash between saving and finishing, reads the same
    # artifacts again. Keying on the dispatch identifies the physical attempt
    # rather than the run of the code, so a replay counts nothing new while a
    # genuine retry inside one iteration still counts.
    last_settled_dispatch_epoch: Optional[float] = None

    def __post_init__(self) -> None:
        if not self.target_samples_m:
            raise ValueError("target_samples_m must not be empty")
        if not 0.0 <= self.open_fraction <= 1.0:
            raise ValueError(
                f"open_fraction must be in [0, 1], got {self.open_fraction}"
            )
        if self.status not in (STATUS_ACTIVE,) + TERMINAL_STATUSES:
            raise ValueError(f"Unknown status {self.status!r}")

    # --- lifecycle ---------------------------------------------------------

    @property
    def is_active(self) -> bool:
        return self.status == STATUS_ACTIVE

    def with_failed_push(self, real_object_id: str, edge_idx: int) -> "RegionOpeningTarget":
        entry = (str(real_object_id), int(edge_idx))
        if entry in self.failed_pushes:
            return self
        return replace(self, failed_pushes=self.failed_pushes + (entry,))

    def forgetting_moved(self, moved_object_ids: Sequence[str]) -> "RegionOpeningTarget":
        """Drop failures recorded against objects that have since moved.

        An edge index names a contact point in the object's own body frame, so
        once the object moves, an exclusion recorded before the move points
        somewhere else in the world. NAMOPlanner prunes its in-memory blacklist
        exactly this way after a successful push. Without the same prune here,
        the persisted copy outlives the planner instance and the next process
        sends stale exclusions back to namo_cpp, which can hide the very finish
        push the setup push just made available.
        """
        moved = {str(o) for o in moved_object_ids}
        kept = tuple(e for e in self.failed_pushes if e[0] not in moved)
        if len(kept) == len(self.failed_pushes):
            return self
        return replace(self, failed_pushes=kept)

    def with_push_attempted(
        self, iteration: int, dispatched_at_epoch: Optional[float] = None
    ) -> "RegionOpeningTarget":
        """Count one physical push, unless this exact dispatch was already counted.

        Exact float equality is the right test here. The value is written once
        when the push is dispatched and read back from JSON on every later
        settlement, so it is the same number rather than a recomputed one.

        Without a dispatch to key on, the count advances. The caller that cannot
        supply one is the in-process planner, which settles once per commit and
        has nothing to replay.
        """
        if (
            dispatched_at_epoch is not None
            and dispatched_at_epoch == self.last_settled_dispatch_epoch
        ):
            return self
        return replace(
            self,
            physical_pushes_attempted=self.physical_pushes_attempted + 1,
            last_iteration=int(iteration),
            last_settled_dispatch_epoch=(
                dispatched_at_epoch
                if dispatched_at_epoch is not None
                else self.last_settled_dispatch_epoch
            ),
        )

    def released(self, status: str) -> "RegionOpeningTarget":
        if status not in TERMINAL_STATUSES:
            raise ValueError(f"Release status must be one of {TERMINAL_STATUSES}")
        return replace(self, status=status)

    def minimum_reachable(self) -> int:
        """How many of the frozen points must be reachable to call this open.

        The fraction is the one recorded on the target, not whatever the config
        says now: a subproblem in flight must not be re-graded underneath itself.
        Mirrors namo_cpp's _minimum_needed -- ceil of the fraction, floored at 1.
        """
        return max(1, math.ceil(self.open_fraction * len(self.target_samples_m)))

    # --- what the planner call needs ---------------------------------------

    def as_solve_kwargs(self) -> Dict[str, Any]:
        """Arguments for NAMOPlanningService.solve_boundary_from_xml."""
        return {
            "target_points": [tuple(p) for p in self.target_samples_m],
            "blocking_objects": list(self.blocker_real_ids),
        }

    # --- serialization -----------------------------------------------------
    #
    # Hand-written rather than asdict(), matching core/serialization.py: the
    # on-disk shape is a contract read by a separate process, so it should not
    # silently follow a field rename.

    def to_dict(self) -> Dict[str, Any]:
        return {
            "schema_version": SCHEMA_VERSION,
            "target_id": self.target_id,
            "status": self.status,
            "target_samples_m": [[float(x), float(y)] for x, y in self.target_samples_m],
            "blocker_real_ids": list(self.blocker_real_ids),
            "open_fraction": float(self.open_fraction),
            "selected_iteration": int(self.selected_iteration),
            "last_iteration": int(self.last_iteration),
            "source_region_path": list(self.source_region_path),
            "sample_seed": int(self.sample_seed),
            "scale_factor": float(self.scale_factor),
            "failed_pushes": [[obj, int(edge)] for obj, edge in self.failed_pushes],
            "physical_pushes_attempted": int(self.physical_pushes_attempted),
            "last_settled_dispatch_epoch": self.last_settled_dispatch_epoch,
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "RegionOpeningTarget":
        version = int(data.get("schema_version", SCHEMA_VERSION))
        if version not in SUPPORTED_SCHEMA_VERSIONS:
            raise ValueError(
                f"Unsupported active-target schema_version {version}; "
                f"this build reads {SUPPORTED_SCHEMA_VERSIONS} and writes "
                f"{SCHEMA_VERSION}"
            )
        return cls(
            target_samples_m=tuple(
                (float(p[0]), float(p[1])) for p in data["target_samples_m"]
            ),
            blocker_real_ids=tuple(str(o) for o in data.get("blocker_real_ids", ())),
            open_fraction=float(data["open_fraction"]),
            target_id=str(data.get("target_id", "")),
            status=str(data.get("status", STATUS_ACTIVE)),
            selected_iteration=int(data.get("selected_iteration", 0)),
            last_iteration=int(data.get("last_iteration", 0)),
            source_region_path=tuple(str(r) for r in data.get("source_region_path", ())),
            sample_seed=int(data.get("sample_seed", 42)),
            scale_factor=float(data.get("scale_factor", 1.0)),
            failed_pushes=tuple(
                (str(entry[0]), int(entry[1])) for entry in data.get("failed_pushes", ())
            ),
            physical_pushes_attempted=int(data.get("physical_pushes_attempted", 0)),
            last_settled_dispatch_epoch=(
                None
                if data.get("last_settled_dispatch_epoch") is None
                else float(data["last_settled_dispatch_epoch"])
            ),
        )

    def save(self, path: Path) -> None:
        """Write atomically, so a crash mid-write cannot leave a half file."""
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp = path.parent / f".tmp_{path.name}"
        tmp.write_text(json.dumps(self.to_dict(), indent=2) + "\n", encoding="utf-8")
        os.replace(tmp, path)

    @classmethod
    def load(cls, path: Path) -> Optional["RegionOpeningTarget"]:
        """Read a target, or None when there is no active subproblem.

        A released target reads back as None: the file is left in place as a
        record of what happened, but the caller should select a new boundary.
        """
        path = Path(path)
        if not path.is_file():
            return None
        target = cls.from_dict(json.loads(path.read_text(encoding="utf-8")))
        return target if target.is_active else None


# "The same object, unmoved" against real camera measurements. Mirrors the
# scene/XML matcher's thresholds, which is the repo's existing statement of the
# same idea. Tighter and marker jitter clears the blacklist; looser and a real
# shove goes unnoticed.
OBJECT_MOVED_TOLERANCE_CM = 0.5
OBJECT_ROTATED_TOLERANCE_DEG = 5.0


def angle_diff_deg(a_deg: float, b_deg: float) -> float:
    """Smallest angle between two headings, in degrees.

    Wrapped, so 359 and 1 are two degrees apart rather than 358. Unwrapped, a
    rotation across the zero crossing reads as most of a full turn and any
    "did it move" test built on it fires on an object that barely twitched.
    """
    return abs((a_deg - b_deg + 180.0) % 360.0 - 180.0)


def pose_delta(
    after: Sequence[float], before: Sequence[float]
) -> Tuple[float, float]:
    """(distance_cm, angle_deg) between two (x_cm, y_cm, theta_deg) poses.

    Straight-line distance, not per-axis. A diagonal shove of 0.4 cm on each
    axis is 0.57 cm of real movement, and comparing the axes separately calls
    that unmoved.

    One implementation, because two callers ask the same question. The
    scene-to-XML matcher decides whether two records describe the same object,
    and objects_that_moved decides whether one object stayed put between two
    frames. Same thresholds, same arithmetic, and they used to differ on both.
    """
    return (
        math.hypot(after[0] - before[0], after[1] - before[1]),
        angle_diff_deg(after[2], before[2]),
    )


def objects_that_moved(
    before: Optional[Dict[str, Sequence[float]]],
    after: Optional[Dict[str, Sequence[float]]],
) -> set:
    """Names whose pose changed enough to invalidate a body-frame edge index.

    Takes ``{name: (x_cm, y_cm, theta_deg)}`` from either side, because the
    in-process planner reads Observation objects while the session subprocess
    reads mid_obs.jsonl records. One rule, two thin adapters: the two paths
    disagreeing about what "moved" means is how a stale exclusion survives on
    one of them.

    A push shoves its neighbours, not only its target, and their exclusions are
    just as meaningless afterwards.
    """
    moved = set()
    for name, pose in (after or {}).items():
        prior = (before or {}).get(name)
        if prior is None:
            continue
        shifted_cm, turned_deg = pose_delta(pose, prior)
        if (
            shifted_cm > OBJECT_MOVED_TOLERANCE_CM
            or turned_deg > OBJECT_ROTATED_TOLERANCE_DEG
        ):
            moved.add(str(name))
    return moved


def fingerprint_samples(samples: Sequence[Sequence[float]]) -> str:
    """A stable name for a target, derived from the points that define it.

    The id was a per-iteration counter, so "the replan after a setup push is
    still working the same boundary" compared ro-0001 against ro-0001. That
    passes whichever boundary the planner actually landed on, which is the one
    thing the check exists to catch.

    The frozen samples are the identity, for the reason in the module docstring,
    so deriving the name from them makes the comparison mean something. The same
    samples give the same name across replans and across processes, and a
    different boundary cannot borrow it.

    Order is part of the identity. The list is the caller's frozen criterion and
    a reordering is a different persisted record, so this does not sort.
    """
    payload = ";".join(
        "{:.{p}f},{:.{p}f}".format(float(x), float(y), p=TARGET_ID_PRECISION_M)
        for x, y in samples
    )
    digest = hashlib.sha256(payload.encode("utf-8")).hexdigest()
    return f"{TARGET_ID_PREFIX}{digest[:TARGET_ID_DIGEST_CHARS]}"


def target_path_for_run(run_dir: Path) -> Path:
    """Where a run's active target lives."""
    return Path(run_dir) / ACTIVE_TARGET_FILENAME


def target_from_selection(
    choice: Any,
    *,
    open_fraction: float,
    iteration: int = 0,
    sample_seed: int = 42,
    scale_factor: float = 1.0,
    target_id: str = "",
) -> RegionOpeningTarget:
    """Freeze a bridge BoundaryChoice into a durable target.

    Takes the bridge's already-translated choice -- blockers in real naming --
    rather than namo_cpp's raw selection, so this module needs no knowledge of
    the planner package. Duck-typed on ``target_points_m``, ``blocker_real_ids``
    and ``region_path``.
    """
    points: Sequence[Sequence[float]] = choice.target_points_m
    samples = tuple((float(p[0]), float(p[1])) for p in points)
    return RegionOpeningTarget(
        target_samples_m=samples,
        blocker_real_ids=tuple(str(o) for o in choice.blocker_real_ids),
        open_fraction=float(open_fraction),
        # Not the iteration counter it used to be: a counter makes every
        # cross-push identity check vacuous. An explicit target_id still wins,
        # so a caller reviving an older record keeps the name it was stored under.
        target_id=target_id or fingerprint_samples(samples),
        selected_iteration=int(iteration),
        last_iteration=int(iteration),
        source_region_path=tuple(str(r) for r in getattr(choice, "region_path", ())),
        sample_seed=int(sample_seed),
        scale_factor=float(scale_factor),
    )


# Outcomes of one advance step, for callers that must branch on them.
ADVANCE_PLANNED = "planned"
ADVANCE_NO_BOUNDARY = "no_boundary"
ADVANCE_EXHAUSTED = "exhausted"
ADVANCE_NO_PLAN = "no_plan"

# One call may cross several boundaries that turn out to be already open --
# opening one can merge regions and clear the next. Bounded so a graph/opener
# disagreement cannot spin here instead of returning.
MAX_BOUNDARY_ADVANCES = 4


# Failure reasons meaning "this boundary cannot be worked on from here", as
# opposed to "this attempt failed". namo_cpp documents the first as a normal
# outcome the caller must re-choose after. Retrying either one produces the
# identical failure on every replan.
# "ambiguous_boundary" is namo_cpp refusing to guess: two neighbours match the
# pinned objects equally well, so the object set names no single boundary.
# Re-solving the same target repeats the tie, so drop it and re-select.
UNUSABLE_BOUNDARY_REASONS = frozenset(
    {
        "target_not_immediate_neighbor",
        "blocker_not_observed",
        "no_blocking_objects",
        "ambiguous_boundary",
    }
)


def _boundary_label_pair(plan: Any) -> Optional[Tuple[str, str]]:
    """The (source, target) labels to exclude from the next selection.

    Both come from the solve that just ran, so both describe the same snapshot.
    The earlier version took the source from ``target.source_region_path[0]``,
    persisted before the last push, and paired it with the target label from the
    current solve. Labels are ordinal and get reassigned rather than retired, so
    that mix could name a live boundary that is not the one being excluded, and
    nothing in the pair says which case you are in. The blocked list being local
    to one advance call did not save it, because the staleness was inside the
    pair rather than in how long the list lived.

    Returns None when the solve could not resolve both ends. Emitting nothing
    means the next selection may re-pick this boundary, which costs a wasted
    call; guessing from persisted labels can silently exclude the wrong one.
    """
    source = str(getattr(plan, "resolved_source", "") or "")
    target = str(getattr(plan, "resolved_target", "") or "")
    return (source, target) if source and target else None


def _plan_is_executable(plan: Any, planner_kwargs: Optional[Dict[str, Any]]) -> bool:
    """Is there something here worth sending to the robot?

    Subgoals are required either way. What `success` means is not the same in
    both modes, so it gates only one of them.

    Under search, `success=False` with subgoals means a chain that failed
    verification. Running it anyway is not what the search arm claims to do, so
    the check stays exactly as it was.

    Under reactive there is nothing to verify. The rule returns the argmax at the
    state in front of the robot, having already seen the simulator say that push
    does not open the boundary, and the answer to that is to push and look again
    rather than to stand still. `success` there reports whether the boundary
    opened *in simulation*, which is precisely the prediction this mode exists
    not to trust. Gating on it would make reactive refuse to move on every scene
    it cannot open in one push, and that reads in the results as "reactive is
    terrible" rather than as a wiring bug.

    Read from `planner_kwargs` rather than from config, so it reflects the call
    that was actually made.
    """
    if not plan.subgoals:
        return False
    mode = str((planner_kwargs or {}).get("mode", DEFAULT_EXEC_MODE))
    if mode == REACTIVE_EXEC_MODE:
        return True
    return bool(plan.success)


def advance_boundary(
    bridge: Any,
    observation: Any,
    robot_goal_cm: Tuple[float, float],
    *,
    target: Optional[RegionOpeningTarget],
    open_fraction: float,
    scale_factor: float = 1.0,
    iteration: int = 0,
    max_advances: int = MAX_BOUNDARY_ADVANCES,
    planner_kwargs: Optional[Dict[str, Any]] = None,
) -> Tuple[Any, Optional[RegionOpeningTarget], str, Optional[str]]:
    """One step of the inner loop: solve the held boundary, or pick the next.

    Pure with respect to storage -- it takes the current target and returns the
    one the caller should now hold (``None`` once released). Persisting it, and
    doing something with the plan, are the caller's business.

    Both entry points use this. The in-process planner and the plan-only
    subprocess share no other code, and a second copy of this logic would drift
    the way the two chain-reuse ladders in this repo already have.

    ``planner_kwargs`` are the strategy, seed and opener options that would
    otherwise reach the planner through ``bridge.plan``. Held mode bypasses that
    call entirely, so without forwarding them here every option is silently
    dropped and repeated attempts become identical searches.

    Returns ``(plan, target, status, released)``, where ``released`` says what
    happened to the target that was passed in: ``STATUS_OPENED``,
    ``STATUS_EXHAUSTED``, or ``None`` if it is still being worked on. Callers
    used to infer this from the status and got it wrong twice: a transient
    failure looked like the boundary had opened, and advancing past one
    already-open boundary before exhausting the next marked the wrong record.
    """
    blocked: List[Tuple[str, str]] = []
    last_plan = None
    incoming = target
    released: Optional[str] = None

    def _release(status: str) -> None:
        nonlocal released
        if target is incoming and incoming is not None and released is None:
            released = status

    for _advance in range(max_advances):
        if target is None:
            choice = bridge.select_boundary(
                observation, robot_goal_cm, blocked_boundaries=blocked or None
            )
            if choice.goal_already_reachable or not choice.found:
                # Nothing selectable. If we got here by discarding a boundary,
                # say so, because the caller has one to mark dead.
                status = ADVANCE_EXHAUSTED if blocked else ADVANCE_NO_BOUNDARY
                return last_plan, None, status, released
            target = target_from_selection(
                choice,
                open_fraction=open_fraction,
                iteration=iteration,
                scale_factor=scale_factor,
            )

        plan = bridge.solve_boundary(
            observation, robot_goal_cm, target, **(planner_kwargs or {})
        )
        last_plan = plan

        if plan.already_open:
            # Opened with no push -- often by an earlier push in this same
            # subproblem. Release it and look at the next boundary.
            _release(STATUS_OPENED)
            target = None
            continue

        if _plan_is_executable(plan, planner_kwargs):
            # Checked BEFORE the exhausted branch. The reactive solver stamps
            # boundary_exhausted/all_pushes_failed on the same plan whose
            # argmax push it returns, because in simulation that push opens
            # nothing -- which is the exact prediction reactive exists not to
            # trust. With the old order the stamp won, the only doorway got
            # blacklisted, and every reactive run on a scene search cannot
            # crack ended in 5 seconds with zero pushes (hard_004, 2026-08-31;
            # the earlier "reactive zero-push row" was this same bug). Under
            # search, executable requires success=True and a successful plan
            # never carries the exhausted stamp, so this order changes nothing
            # for the search arms.
            return plan, target, ADVANCE_PLANNED, released

        if plan.boundary_exhausted or plan.failure_reason in UNUSABLE_BOUNDARY_REASONS:
            # Selection is deterministic, so simply dropping this boundary would
            # pick the same one straight back. Exclude it, then look again.
            pair = _boundary_label_pair(plan)
            if pair is not None:
                blocked.append(pair)
            _release(STATUS_EXHAUSTED)
            target = None
            continue

        return plan, target, ADVANCE_NO_PLAN, released

    status = ADVANCE_EXHAUSTED if blocked else ADVANCE_NO_BOUNDARY
    return last_plan, None, status, released
