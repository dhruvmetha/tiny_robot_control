#!/usr/bin/env python3
"""Model-agnostic GRADING CONTRACT for all NAMO 1-push evals.

The per-episode matching + difficulty binning + random floor that EVERY eval (scorer, diffusion,
random) must apply IDENTICALLY so the comparison stays apples-to-apples. This is the SINGLE SOURCE
OF TRUTH: neither eval_scorer.py nor eval_grounding.py may redefine these — both import here.

Why this module exists: match_episode / bin_of / floor are the rules that decide *which episode a
sample is graded against* and *what difficulty bucket it lands in*. Two copies of those rules can
drift; if they ever diverge between the scorer eval and the diffusion eval, the comparison table
silently becomes apples-to-oranges. Importing one definition everywhere ENFORCES the
multi_episode_rooms.md invariant instead of hoping two copies stay in sync.
See docs/pipeline/multi_episode_rooms.md.
"""
import math
from math import comb

# canonical 5-channel scene-mask order (local_tight crop)
MASKS = ["local_tight_static", "local_tight_movable", "local_tight_target_object",
         "local_tight_robot_region", "local_tight_goal_sample_region"]
OUT = 64


def match_episode(recs, oci, gt):
    """Match an H5 sample to its OWN episode: prefer episodes whose `valid` contains the sample's GT
    push (right goal), then the nearest object_center (right pushed-object). Returns (rec, dist_m).
    The caller MUST drop matches with dist > 0.01 m (that means the wrong pushed object)."""
    if not recs:
        return None, 1e9
    pool = [r for r in recs if gt in {tuple(t) for t in r["valid"]}] or recs
    rec = min(pool, key=lambda r: (r["object_center"][0] - oci[0]) ** 2 + (r["object_center"][1] - oci[1]) ** 2)
    return rec, math.hypot(rec["object_center"][0] - oci[0], rec["object_center"][1] - oci[1])


def bin_of(sr):
    """True difficulty from the matched episode's solve_rate (same thresholds as build_test_divisions)."""
    return "hard" if sr < 0.05 else ("med" if sr < 0.30 else "easy")


def mw_auc(positive, negative):
    """Mann-Whitney ROC-AUC — THE one AUC definition for NAMO. Returns None if either side is empty.

    Lives here for the same reason match_episode does: seven different "AUC"s once drifted across
    four scripts (docs/experiments/auc_metrics_reconciliation.md). The *definition* is now single-
    source; what still has to be stated at every call site is WHICH positives, WHICH negatives, and
    whether the pile is pooled across boards or averaged within them.
    """
    import numpy as np

    positive, negative = np.asarray(positive, float), np.asarray(negative, float)
    if not positive.size or not negative.size:
        return None
    ranks = np.concatenate([positive, negative]).argsort().argsort().astype(float) + 1
    return round(float((ranks[:positive.size].sum() - positive.size * (positive.size + 1) / 2)
                       / (positive.size * negative.size)), 4)


def floor_no_replacement(F, R, k):
    """Random floor: P(>=1 of F valid pushes in k DISTINCT draws from R reachable). Hypergeometric,
    exact. A sensible random agent never re-tries a known-failed push, so WITHOUT replacement is the
    right baseline for any distinct-attempt method (a ranker); not 1-(1-p)^k, which models an agent
    that forgets. Identical to the with-replacement curve only at k=1."""
    if F <= 0 or R <= 0:
        return 0.0
    if k >= R or R - F < k:
        return 1.0
    return 1.0 - comb(R - F, k) / comb(R, k)
