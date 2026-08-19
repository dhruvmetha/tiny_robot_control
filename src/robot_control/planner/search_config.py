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

    def __post_init__(self) -> None:
        if self.local_search not in LOCAL_SEARCH_CHOICES:
            raise ValueError(
                f"Unknown local_search {self.local_search!r}. "
                f"Valid: {list(LOCAL_SEARCH_CHOICES)}"
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
        diagnosed afterwards, so this is printed even for the default.
        """
        if not self.uses_best_first:
            return "local search: region_bfs (chain BFS)"
        parts = [f"local search: best_first/{self.best_first_prior}"]
        if self.uses_ranker:
            parts.append(f"ckpt={self.scorer_ckpt}")
        parts.append(f"hmax={self.best_first_hmax if self.best_first_hmax is not None else 'canonical'}")
        parts.append(
            f"budget={self.keyhole_simulation_budget if self.keyhole_simulation_budget is not None else 'canonical'}"
        )
        if self.ml_device:
            parts.append(f"device={self.ml_device}")
        return "  ".join(parts)
