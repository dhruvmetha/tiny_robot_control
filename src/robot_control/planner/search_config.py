"""Which local search namo_cpp runs, and the knobs that identify it.

`full_namo` picks one region boundary at a time and hands it to a *local*
search. Two exist: `region_bfs` (level-by-level chain search over primitive
goals) and `best_first` (a single global priority queue over unsimulated
pushes, ordered either by a learned ranker or at random).

Selecting between them is a matter of forwarding a few keys through
`NAMOPlanningService.plan_from_xml`, which passes unknown kwargs straight into
the planner's `algorithm_params`. Both the runtime planner and run_namo's
plan-only mode need that mapping, so it lives here once rather than in each.

Defaults deliberately resolve to `region_bfs` with no extra keys, so a run that
does not ask for best-first behaves exactly as it did before this module
existed.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Optional

LOCAL_SEARCH_CHOICES = ("region_bfs", "best_first")
BEST_FIRST_PRIOR_CHOICES = ("model", "uniform")

# Which decision rule reads the ranked pool, forwarded verbatim as
# `solve_boundary_from_xml(mode=...)`. `search` pops a priority queue and can
# back up. `reactive` takes the argmax at the state in front of it and cannot.
#
# The second one exists because the sim-to-real gap attacks lookahead
# specifically: a real block pushed off-centre keeps rotating, 2.13 to 2.42
# degrees per cm of travel at corner contacts, where the simulator squares it
# back up. After one real push a block sat 27 degrees off the predicted pose, so
# the second push of a depth-2 plan contacts a face that is not there. Reactive
# re-decides from what the camera sees and never rests on a prediction. Which one
# survives hardware is unmeasured, so both run and every trial records which.
#
# These strings are namo_cpp's `BOUNDARY_MODES` verbatim, and are forwarded
# without translation. test_exec_mode_routing pins that they still match: a
# mapping table between the two repositories would be one more place to write
# the wrong arm into a paired comparison.
SEARCH_EXEC_MODE = "search"
REACTIVE_EXEC_MODE = "reactive"
EXEC_MODE_CHOICES = (SEARCH_EXEC_MODE, REACTIVE_EXEC_MODE)
DEFAULT_EXEC_MODE = SEARCH_EXEC_MODE

# namo_cpp owns the canonical protocol (hmax=2, 900 simulations per keyhole).
# Leaving these unset here forwards nothing and lets that default apply, so the
# two repositories cannot disagree about what "canonical" means.
DEFAULT_LOCAL_SEARCH = "region_bfs"
DEFAULT_BEST_FIRST_PRIOR = "model"


@dataclass(frozen=True)
class LocalSearchConfig:
    """Resolved local-search selection, ready to forward as planner kwargs."""

    local_search: str = DEFAULT_LOCAL_SEARCH
    best_first_prior: str = DEFAULT_BEST_FIRST_PRIOR
    scorer_ckpt: Optional[str] = None
    best_first_hmax: Optional[int] = None
    keyhole_simulation_budget: Optional[int] = None
    ml_device: Optional[str] = None
    exec_mode: str = DEFAULT_EXEC_MODE

    def __post_init__(self) -> None:
        if self.local_search not in LOCAL_SEARCH_CHOICES:
            raise ValueError(
                f"Unknown local_search {self.local_search!r}. "
                f"Valid: {list(LOCAL_SEARCH_CHOICES)}"
            )
        if self.exec_mode not in EXEC_MODE_CHOICES:
            raise ValueError(
                f"Unknown exec_mode {self.exec_mode!r}. "
                f"Valid: {list(EXEC_MODE_CHOICES)}"
            )
        if self.uses_reactive and not self.uses_best_first:
            # namo_cpp refuses this too, but only once the service is called
            # mid-run. Reactive returns the argmax of a ranked pool and
            # region_bfs builds none, it sweeps every edge and depth in its own
            # order.
            raise ValueError(
                "exec_mode='reactive' needs --local-search best_first: reactive "
                f"ranks candidates and picks the top one, and {self.local_search!r} "
                "builds no ranked pool. Pass --local-search best_first, or use "
                "--exec-mode search."
            )
        if self.best_first_prior not in BEST_FIRST_PRIOR_CHOICES:
            raise ValueError(
                f"Unknown best_first_prior {self.best_first_prior!r}. "
                f"Valid: {list(BEST_FIRST_PRIOR_CHOICES)}"
            )
        if self.uses_ranker and not self.scorer_ckpt:
            # namo_cpp raises this too, but only once the planner is constructed
            # mid-run. Catching it here fails before the robot has moved.
            raise ValueError(
                "local_search='best_first' with prior='model' requires a scorer "
                "checkpoint; pass --scorer-ckpt or use --best-first-prior uniform"
            )
        if self.best_first_hmax is not None and self.best_first_hmax < 1:
            raise ValueError(f"best_first_hmax must be >= 1, got {self.best_first_hmax}")
        if self.keyhole_simulation_budget is not None and self.keyhole_simulation_budget < 1:
            raise ValueError(
                f"keyhole_simulation_budget must be >= 1, "
                f"got {self.keyhole_simulation_budget}"
            )

    @property
    def uses_best_first(self) -> bool:
        return self.local_search == "best_first"

    @property
    def uses_ranker(self) -> bool:
        return self.uses_best_first and self.best_first_prior == "model"

    @property
    def uses_reactive(self) -> bool:
        return self.exec_mode == REACTIVE_EXEC_MODE

    def as_boundary_kwargs(self) -> Dict[str, Any]:
        """The extra keys `solve_boundary_from_xml` names, on top of the search ones.

        Separate from `as_planner_kwargs` because the two go to different
        methods. `mode` is a named parameter of `solve_boundary_from_xml` and
        means nothing to `plan_from_xml`, where it would ride into
        `algorithm_params` and be dropped without a word. Sending it only down
        the path that reads it is what keeps that from happening.
        """
        return {"mode": self.exec_mode}

    def as_planner_kwargs(self) -> Dict[str, Any]:
        """Keys to forward into plan_from_xml. Empty for the default search."""
        if not self.uses_best_first:
            return {}
        kwargs: Dict[str, Any] = {
            "full_namo_local_search": self.local_search,
            "best_first_prior": self.best_first_prior,
        }
        if self.scorer_ckpt:
            kwargs["scorer_ckpt"] = self.scorer_ckpt
        if self.best_first_hmax is not None:
            kwargs["best_first_hmax"] = self.best_first_hmax
        if self.keyhole_simulation_budget is not None:
            kwargs["full_namo_keyhole_simulation_budget"] = self.keyhole_simulation_budget
        if self.ml_device:
            kwargs["ml_device"] = self.ml_device
        return kwargs

    def describe(self) -> str:
        """One line for the startup banner.

        A run log that does not record which search produced a push cannot be
        diagnosed afterwards, so this is printed even for the default. The mode
        leads, because with execution mode crossed against the ranker arm there
        are two ways to mislabel a trial rather than one, and a person at the
        table should not have to parse a command string to see which cell of the
        design they are filling.
        """
        mode = f"exec mode: {self.exec_mode}"
        if not self.uses_best_first:
            return f"{mode}  |  local search: region_bfs (chain BFS)"
        parts = [mode, f"local search: best_first/{self.best_first_prior}"]
        if self.uses_ranker:
            parts.append(f"ckpt={self.scorer_ckpt}")
        parts.append(f"hmax={self.best_first_hmax if self.best_first_hmax is not None else 'canonical'}")
        parts.append(
            f"budget={self.keyhole_simulation_budget if self.keyhole_simulation_budget is not None else 'canonical'}"
        )
        if self.ml_device:
            parts.append(f"device={self.ml_device}")
        return "  ".join(parts)


# ─── Which planner actually reads these keys ────────────────────────────

# Only `full_namo` reads `full_namo_local_search`
# (namo_cpp full_namo_planner.py:125). `region_opening` runs its own
# exhaustive sweep over every edge and depth and never looks at the key, so a
# best-first request aimed at it is dropped in silence. That cost a real
# debugging session: 2,959 primitives and 394 s of wall clock on hardware,
# spent believing a ranker was ordering the queue when none was loaded.
BEST_FIRST_AWARE_ALGORITHMS = ("full_namo",)

# How `region_opening` takes a learned ranker instead. It reads
# `scorer_ckpt` only when the goal strategy is this one
# (namo_cpp region_opening.py:670).
SCORER_GOAL_STRATEGY = "scorer"


# `mode` is a named parameter of `solve_boundary_from_xml`, and that method is
# reached only by the held-boundary loop (--hold-region-target / --active-target).
# The whole-problem path calls `plan_from_xml`, which does not name it, so an
# exec mode aimed there rides into `algorithm_params` and is dropped in silence.
# That failure is worse than the one BEST_FIRST_AWARE_ALGORITHMS guards: a
# misrouted search flag produces a slow run somebody eventually investigates,
# while a misrouted exec mode produces a normal-looking trial filed under the
# wrong cell of a paired design.
REACTIVE_REQUIRES_HELD_BOUNDARY = True


def check_search_reaches_planner(
    algorithm: str,
    goal_strategy: str,
    config: LocalSearchConfig,
    held_boundary: bool = False,
) -> None:
    """Raise if a selection is addressed to a planner or path that ignores it.

    Unknown keys ride into `algorithm_params` and get dropped without a word,
    so the only symptom is a slow run that looks like a hard scene. Fail before
    the robot moves instead.
    """
    if config.uses_reactive and REACTIVE_REQUIRES_HELD_BOUNDARY and not held_boundary:
        raise ValueError(
            "--exec-mode reactive has no effect without --hold-region-target: the "
            "mode is read by solve_boundary_from_xml, and only the held-boundary "
            "loop calls it. The whole-problem path plans through plan_from_xml, "
            "which would drop the mode and search anyway. Pass "
            "--hold-region-target, or use --exec-mode search."
        )
    if config.uses_best_first and algorithm not in BEST_FIRST_AWARE_ALGORITHMS:
        raise ValueError(
            f"--local-search {config.local_search} has no effect with "
            f"--algorithm {algorithm}: only {list(BEST_FIRST_AWARE_ALGORITHMS)} "
            f"read it, and {algorithm} runs its own exhaustive sweep. "
            f"Use --algorithm full_namo, or stay on {algorithm} and pass "
            f"--strategy {SCORER_GOAL_STRATEGY} to let the checkpoint in."
        )
    if (
        config.scorer_ckpt
        and algorithm not in BEST_FIRST_AWARE_ALGORITHMS
        and goal_strategy != SCORER_GOAL_STRATEGY
    ):
        raise ValueError(
            f"--scorer-ckpt is never loaded with --algorithm {algorithm} and "
            f"--strategy {goal_strategy}. Pass --strategy "
            f"{SCORER_GOAL_STRATEGY} to use the checkpoint, or drop the flag."
        )


def describe_effective_search(
    algorithm: str,
    goal_strategy: str,
    config: LocalSearchConfig,
    held_boundary: bool = False,
) -> str:
    """One startup line naming the planner and the decision rule it will really run.

    Printed on every path, default included. A log that does not say which
    search produced a push cannot be diagnosed afterwards, and with execution
    mode crossed against the ranker arm the same is now true of which arm a
    trial belongs to. The banner half matters more than the raise half: the
    raise only catches combinations somebody thought to forbid.
    """
    if algorithm in BEST_FIRST_AWARE_ALGORITHMS:
        return f"planner: {algorithm}  |  {config.describe()}"
    ranked = goal_strategy == SCORER_GOAL_STRATEGY
    ranker = f"scorer ckpt={config.scorer_ckpt}" if ranked else "no ranker"
    # The held-boundary loop reaches best_first and reactive through
    # solve_boundary_from_xml's own parameters, so on that path the mode is real
    # even though the algorithm is not best-first aware. Off it, the sweep runs
    # and the mode is not consulted.
    rule = config.exec_mode if held_boundary else "search"
    return (
        f"planner: {algorithm}  |  exec mode: {rule}"
        f"  |  {'held boundary' if held_boundary else 'whole problem'}"
        f"  |  exhaustive sweep over all edges and depths"
        f"  |  goal strategy: {goal_strategy} ({ranker})"
    )
