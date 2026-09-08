#!/usr/bin/env python3
"""Materialize the state after one keyhole is opened, as a standalone XML, so the NEXT keyhole can be labelled.

One round of an N-hop chain. Run it once per keyhole: round k reads the XMLs round k-1 emitted, opens
that scene's current first boundary, and writes the resulting state out as a fresh scene. A 2-hop scene
needs 1 round, a 3-hop scene 2, a 4-hop scene 3.

The problem this solves. `region_opening._explore_from_state` only ever sweeps `adjacency[robot_label]`,
so a boundary between two NON-robot regions is never swept. Every keyhole past the first is exactly such
a boundary, and its blocker is unreachable at t=0 in the overwhelming majority of scenes. So keyhole k>1
has no label computable from the original XML. Opening keyhole k-1 merges the robot region with the next
one along the path, which makes keyhole k an ordinary robot-adjacent boundary — and a scene written out
at that state is an ordinary region-opening problem the existing collection already handles.

Which opener. A scene has many valid openers and each leaves a DIFFERENT state, so the next keyhole is
undefined until one opener is fixed. Convention, applied independently at every round: take the
lexicographically smallest `(n_pushes, object_id, (edge_idx, depth) per push)` — shortest chain first,
then lexicographic. Deterministic, reproducible, model-free and seed-free; these labels must not depend
on the ranker being evaluated. `object_id` is in the key because a boundary is an OR over several
blocking objects, so `(edge, depth)` alone does not identify a push.

Two facts force a refinement, both measured (uniform 300-scene sample of the 2-hop pool):
  * Candidates come from the exhaustive `primitive_trial_log`, NOT from the planner's recorded
    solutions. The planner filters those to MINIMUM push cost (region_opening.py:2578), which on one
    scene left 9 of 57 valid openers visible — the convention would silently have meant "cheapest".
    Exhaustive trial rows therefore retain terminal success states as well as setup states.
  * An opener that passes the 20%-of-100-points test does not always ADVANCE the scene: the pushed
    object can land inside the region it just opened and split it, leaving the goal as far away as
    before (167 of 499 openers) or disconnecting it (54 of 499). The canonical opener is therefore the
    first candidate in the order above whose emitted XML verifies with the hop count reduced by one.

No replay. Every candidate's post-push state comes from the sweep's `primitive_trial_log`, so no push
is ever re-executed and the known replay divergence on collision pushes cannot occur. Older rows that
predate terminal-state logging fall back to `AttemptResult.resulting_state` for compatibility.

Collision checking. Object-object and object-wall contact never aborts a push, so there is nothing
to configure. This script never steps the sim outside the planner anyway.

Verification, per scene, recorded in the output row — nothing is assumed:
  * SE(2) round trip: reload the emitted XML in a fresh env and compare every movable AND the robot to
    the state it was written from. A writer that drops the car's yaw is the exact silent failure mode.
  * region graph: the emitted scene's shortest robot->goal region path must be one hop shorter, and its
    next boundary's blocking-object set is compared with the input scene's.
  * independence: next-boundary objects are protected automatically; ``--protect-object`` additionally
    protects an already-open gate while materializing a later gate.

  python scripts/pipeline/materialize_keyhole2.py \
      --manifest surviving_xmls_cspaths.txt --out-root <root>/round1 --workers 96 \
      --kh1-chain-depth 2 --kh1-timeout 1800
"""
import argparse
import json
import math
import os
import sys
import time
from collections import deque
from multiprocessing import Pool, Process, Queue
from queue import Empty

import yaml

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
for _p in (os.path.join(REPO, "build_python"), os.path.join(REPO, "python")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import namo_rl  # noqa: E402
from namo.core.base_planner import PlannerConfig  # noqa: E402
from namo.core.state_to_xml import write_state_xml  # noqa: E402
from namo.core.xml_goal_parser import extract_goal_from_xml  # noqa: E402
from namo.planners import get_region_snapshot  # noqa: E402
from namo.planners.opening.region_opening import RegionOpeningPlanner  # noqa: E402

# Algorithm-params keys the planner reads, and the YAML key each comes from. Mirrors
# modular_parallel_collection.main()'s algorithm_params dict for the keys this sweep uses; anything not
# listed keeps the planner's own default.
_ALGO_KEYS = (
    "region_max_chain_depth", "region_max_solutions_per_neighbor",
    "region_max_recorded_solutions_per_neighbor", "region_chain_link_cost",
    "region_min_reachable_fraction", "region_frontier_beam_width", "region_ml_ignore_blacklist",
    "region_selection_strategy", "region_exhaustive_mode", "region_label_mode", "region_sample_k",
    "region_sample_restarts", "region_timeout_per_neighbour_sec", "primitive_prefix",
    "target_goal_region", "shuffle_edges", "shuffle_seed",
)
INDEPENDENT_POSITION_TOLERANCE_M = 0.002
INDEPENDENT_ANGLE_TOLERANCE_RAD = math.radians(1.0)


def _rlstate(qpos, qvel):
    state = namo_rl.RLState()
    state.qpos = list(qpos)
    state.qvel = list(qvel)
    return state


def shortest_region_path(adjacency, src, tgt):
    """One deterministic shortest region path src->tgt, or None. Same rule as probe_static_topology."""
    if src == tgt:
        return [src]
    if src not in adjacency or not tgt:
        return None
    parent = {src: None}
    frontier = deque([src])
    while frontier:
        node = frontier.popleft()
        for nb in sorted(adjacency.get(node, ())):
            if nb in parent:
                continue
            parent[nb] = node
            if nb == tgt:
                path = [tgt]
                while parent[path[-1]] is not None:
                    path.append(parent[path[-1]])
                path.reverse()
                return path
            frontier.append(nb)
    return None


def boundary_objects(edge_objects, source, target):
    """The opener's own rule for "which objects sit on this boundary" (see probe_static_topology)."""
    forward = edge_objects.get(source, {}).get(target)
    reverse = edge_objects.get(target, {}).get(source)
    if forward is not None and reverse is not None and set(forward) != set(reverse):
        return []
    return sorted(set(forward if forward is not None else reverse or []))


def next_boundary_objects(snapshot, path):
    """Objects on the first boundary in ``path``, or none after the terminal opening."""
    if not path or len(path) < 2:
        return []
    return boundary_objects(snapshot["edge_objects"], path[0], path[1])


def protected_objects_for_opening(next_boundary, explicit):
    """Objects that the selected opener must leave mechanically unchanged."""
    return sorted(set(next_boundary) | set(explicit or []))


def _dtheta(a, b):
    return abs((a - b + math.pi) % (2 * math.pi) - math.pi)


def protected_boundary_motion(before, after, protected_objects):
    """Measure whether an opener disturbed objects belonging to the next keyhole."""
    metrics = {}
    failure = None
    max_position = 0.0
    max_angle = 0.0
    for object_id in protected_objects:
        before_key = object_id if object_id in before else f"{object_id}_pose"
        after_key = object_id if object_id in after else f"{object_id}_pose"
        if before_key not in before or after_key not in after:
            failure = failure or f"missing_{object_id}"
            metrics[object_id] = {"missing": True}
            continue
        position = math.dist(before[before_key][:2], after[after_key][:2])
        angle = _dtheta(before[before_key][2], after[after_key][2])
        max_position = max(max_position, position)
        max_angle = max(max_angle, angle)
        metrics[object_id] = {
            "position_delta_mm": round(1000.0 * position, 4),
            "angle_delta_deg": round(math.degrees(angle), 4),
        }
        if (
            position > INDEPENDENT_POSITION_TOLERANCE_M
            or angle > INDEPENDENT_ANGLE_TOLERANCE_RAD
        ):
            failure = failure or f"moved_{object_id}"
    return {
        "status": "failed" if failure else "passed",
        "failure": failure,
        "position_tolerance_mm": 1000.0 * INDEPENDENT_POSITION_TOLERANCE_M,
        "angle_tolerance_deg": math.degrees(INDEPENDENT_ANGLE_TOLERANCE_RAD),
        "max_position_delta_mm": round(1000.0 * max_position, 4),
        "max_angle_delta_deg": round(math.degrees(max_angle), 4),
        "objects": metrics,
    }


def build_planner_config(algo_yaml, chain_depth, seed=42, timeout_per_neighbour=None):
    cfg = yaml.safe_load(open(algo_yaml))
    params = {k: cfg[k] for k in _ALGO_KEYS if k in cfg}
    params["primitive_data_dir"] = cfg.get("primitive_data_dir", "data")
    params["region_max_chain_depth"] = chain_depth
    if timeout_per_neighbour is not None:
        params["region_timeout_per_neighbour_sec"] = float(timeout_per_neighbour)
    return PlannerConfig(
        max_depth=cfg.get("max_depth", 5),
        max_goals_per_object=cfg.get("max_goals_per_object", 5),
        max_terminal_checks=cfg.get("max_terminal_checks", 50000),
        max_search_time_seconds=cfg.get("search_timeout", 1800.0),
        goals_per_region=cfg.get("goals_per_region", 100),
        random_seed=seed,
        verbose=False,
        algorithm_params=params,
    ), cfg


def opener_key(attempt):
    """Sort key of a successful AttemptResult: (n_pushes, object_id, (edge,depth) per push).

    Sorting on this gives SHORTEST chain first, then lexicographic — so a 1-push opener always beats a
    2-push one, and among equals the smallest (object_id, edge_idx, depth) wins. A region-opening chain
    pushes ONE object, so object_id is a scalar. Returns None for an attempt with no executed push
    (`already_accessible`), which is not an opener.
    """
    goals = attempt.goal_chain or []
    if not goals or attempt.chosen_object_id is None:
        return None
    cells = []
    for g in goals:
        edge, depth = getattr(g, "edge_idx", None), getattr(g, "depth", None)
        if edge is None or depth is None:
            return None
        cells.append((int(edge), int(depth)))
    return (len(cells), attempt.chosen_object_id, tuple(cells))


def _jkey(k):
    """JSON-safe form of an opener key: [n_pushes, object_id, [[edge, depth], ...]]."""
    return [k[0], k[1], [list(c) for c in k[2]]]


def trial_log_openers(attempts):
    """Ordered states for every one- or two-push opener in the exhaustive trial log.

    The per-object log is identical across that object's AttemptResults, hence the dedup by object.
    Two-push keys reconstruct the setup cell from the terminal row's parent fields.
    """
    out, seen = [], set()
    for a in attempts:
        obj = a.chosen_object_id
        if obj is None or obj in seen:
            continue
        seen.add(obj)
        for t in a.primitive_trial_log or []:
            rs = t.get("resulting_state")
            if not t.get("success") or not rs:
                continue
            chain_depth = int(t.get("chain_depth", 0))
            if chain_depth == 1:
                cells = ((int(t["edge_idx"]), int(t["depth"])),)
            elif chain_depth == 2 and t.get("parent_edge") is not None and t.get("parent_depth") is not None:
                cells = (
                    (int(t["parent_edge"]), int(t["parent_depth"])),
                    (int(t["edge_idx"]), int(t["depth"])),
                )
            else:
                continue
            key = (chain_depth, obj, cells)
            out.append((key, _rlstate(rs["qpos"], rs["qvel"])))
    out.sort(key=lambda kv: kv[0])
    return out


def keyhole1_key(attempts, max_chain_depth):
    """Per-object exhaustive answer key from the trial logs, on the canonical scale.

      solve_rate_1push       = depth-1 cells that OPEN / depth-1 cells TRIED           (F)
      solve_rate_first_push  = distinct first-pushes ENABLING a depth-2 solve / expanded  (F1')

    Same derivation as build_2push_validset.py, so these join to the project's hard/med/easy bins.
    """
    key = {}
    for a in attempts:
        obj = a.chosen_object_id
        if obj is None or obj in key:
            continue                       # the trial log is object-level; identical on every attempt
        log = a.primitive_trial_log or []
        tried = {(t["edge_idx"], t["depth"]) for t in log if t.get("chain_depth") == 1}
        valid = {(t["edge_idx"], t["depth"]) for t in log if t.get("chain_depth") == 1 and t.get("success")}
        if not tried:
            continue
        rec = {"tried": len(tried), "valid": len(valid), "solve_rate_1push": len(valid) / len(tried),
               "timed_out": bool(getattr(a, "neighbour_timed_out", False))}
        if max_chain_depth >= 2:
            tried_fp = {(t["parent_edge"], t["parent_depth"]) for t in log
                        if t.get("chain_depth") == 2 and t.get("parent_edge") is not None}
            valid_fp = {(t["parent_edge"], t["parent_depth"]) for t in log
                        if t.get("chain_depth") == 2 and t.get("parent_edge") is not None and t.get("success")}
            rec.update(tried_first_push=len(tried_fp), valid_first_push=len(valid_fp),
                       solve_rate_first_push=(len(valid_fp) / len(tried_fp)) if tried_fp else None)
        key[obj] = rec
    return key


def process_one(args):
    (
        xml,
        cfg_file,
        algo_yaml,
        out_root,
        seed,
        kh1_chain_depth,
        kh1_timeout,
        explicit_protected_objects,
    ) = args
    row = {"xml_path": xml}
    t0 = time.time()
    try:
        env = namo_rl.RLEnvironment(xml, cfg_file, False)
        goal = extract_goal_from_xml(xml)
        env.set_robot_goal(*goal)

        # goal_radius stays at the collection default (None). Passing a number moves the goal into the
        # robot's own region in ~11% of rooms and silently breaks the manifest join.
        snap = get_region_snapshot(env, goals_per_region=0, goal_radius=None, local_info_only=False,
                                   seed=seed, use_cpp_unified=True, use_xml_goal=True)
        robot_label, goal_label = snap.get("robot_label") or "", snap.get("goal_label") or ""
        path = shortest_region_path(snap["adjacency"], robot_label, goal_label)
        row["region_path"] = path
        row["hop_count"] = (len(path) - 1) if path else -1
        if not path or len(path) < 2:
            row["status"] = "no_region_path"
            return _done(row, t0)
        hop_in = len(path) - 1
        kh1_target = path[1]
        row["kh1_boundary_objects"] = boundary_objects(snap["edge_objects"], robot_label, kh1_target)
        row["next_boundary_objects"] = next_boundary_objects(snap, path[1:])
        row["protected_objects"] = protected_objects_for_opening(
            row["next_boundary_objects"], explicit_protected_objects
        )
        initial_obs = {
            name: [float(value[0]), float(value[1]), float(value[2])]
            for name, value in env.get_observation().items()
        }

        # ---- keyhole 1: exhaustive sweep of the FIRST boundary only ----
        pcfg, _ = build_planner_config(algo_yaml, chain_depth=kh1_chain_depth, seed=seed,
                                       timeout_per_neighbour=kh1_timeout)
        planner = RegionOpeningPlanner(env, pcfg)
        result = planner.search(goal, target_neighbor=kh1_target)
        attempts = (result.algorithm_stats or {}).get("attempt_results") or []
        row["kh1_key"] = keyhole1_key(attempts, kh1_chain_depth)
        row["kh1_pushes"] = int((result.algorithm_stats or {}).get("total_primitives_attempted", 0))
        row["kh1_timed_out"] = any(getattr(a, "neighbour_timed_out", False) for a in attempts)
        row["kh1_failure_reasons"] = sorted({a.failure_reason for a in attempts if a.failure_reason})

        # Ordered candidate openers, shortest chain first then lexicographic. The exhaustive trial log
        # is the authority for both one- and two-push chains; planner AttemptResults are minimum-cost
        # filtered and remain only as compatibility for data collected before terminal states were
        # added to successful trial rows.
        cands = trial_log_openers(attempts)
        row["kh1_openers_1push"] = sum(key[0] == 1 for key, _state in cands)
        row["kh1_openers_2push"] = sum(key[0] == 2 for key, _state in cands)
        if not cands:
            cands = [(k, a.resulting_state) for k, a in
                     sorted((k, a) for k, a in ((opener_key(a), a) for a in attempts if a.success)
                            if k is not None)
                     if a.resulting_state is not None]
        row["kh1_candidate_openers"] = len(cands)
        if not cands:
            row["status"] = "no_kh1_opener"
            return _done(row, t0)
        row["lex_min_opener"] = _jkey(cands[0][0])

        # An opener that passes the 20%-of-100-points test does NOT always leave the goal one hop away:
        # the pushed object can end up inside the middle region and split it, so the piece touching the
        # goal becomes a NEW region and the scene stays two-hop (measured: 167 of 499 openers leave 2
        # hops, 54 disconnect the goal entirely). Keyhole 2 only exists at a one-hop post-opener state,
        # so the canonical opener is the FIRST candidate in the order above whose emitted XML verifies
        # as one hop. The emitted XML is the authority, not the live state — they disagreed on 1 of 47
        # scenes, where a 32 um difference flipped a wavefront cell.
        out_xml = os.path.join(out_root, "xmls", _scene_id(xml) + ".xml")
        rejected = []
        accepted = False
        fallback = None                      # first candidate overall, materialized only if none advances
        for k, st in cands:
            env.set_full_state(st)
            s = get_region_snapshot(env, goals_per_region=0, goal_radius=None, local_info_only=False,
                                    seed=seed, use_cpp_unified=True, use_xml_goal=True)
            p = shortest_region_path(s["adjacency"], s.get("robot_label") or "", s.get("goal_label") or "")
            hop = (len(p) - 1) if p else -1
            if hop != hop_in - 1:
                rejected.append([_jkey(k), hop, "live"])
                if fallback is None and hop >= 1:
                    fallback = (k, st, hop)   # opened a boundary but did not shorten the path
                continue

            src_obs = {n: [float(v[0]), float(v[1]), float(v[2])] for n, v in env.get_observation().items()}
            protection = protected_boundary_motion(
                initial_obs, src_obs, row["protected_objects"]
            )
            if protection["status"] != "passed":
                rejected.append([_jkey(k), hop, "moved_protected_object", protection])
                continue
            write_state_xml(src_obs, xml, out_xml)
            env2 = namo_rl.RLEnvironment(out_xml, cfg_file, False)
            env2.set_robot_goal(*goal)
            obs2 = env2.get_observation()
            snap2 = get_region_snapshot(env2, goals_per_region=0, goal_radius=None, local_info_only=False,
                                        seed=seed, use_cpp_unified=True, use_xml_goal=True)
            rl2, gl2 = snap2.get("robot_label") or "", snap2.get("goal_label") or ""
            path2 = shortest_region_path(snap2["adjacency"], rl2, gl2)
            hop2 = (len(path2) - 1) if path2 else -1
            if hop2 != hop_in - 1:
                rejected.append([_jkey(k), hop2, "xml"])
                continue

            dxy = {n: math.dist(src_obs[n][:2], obs2[n][:2]) for n in src_obs if n in obs2}
            dth = {n: _dtheta(src_obs[n][2], obs2[n][2]) for n in src_obs if n in obs2}
            objs2 = next_boundary_objects(snap2, path2)
            reach2 = set(env2.get_reachable_objects())
            row.update(
                canonical_opener=_jkey(k),
                canonical_is_lex_min=(_jkey(k) == row["lex_min_opener"]),
                out_xml=out_xml,
                missing_bodies=sorted(set(src_obs) - set(obs2)),
                max_dxy_mm=round(1000.0 * max(dxy.values()), 4),
                max_dtheta_deg=round(math.degrees(max(dth.values())), 4),
                robot_dxy_mm=round(1000.0 * dxy["robot_pose"], 4),
                robot_dtheta_deg=round(math.degrees(dth["robot_pose"]), 4),
                post_region_path=path2,
                post_hop_count=hop2,
                post_goal_in_free_space=bool(snap2.get("goal_in_free_space", False)),
                post_n_regions=len(set(snap2["region_labels"].values())),
                post_next_boundary_objects=objs2,
                next_boundary_matches=(objs2 == row["next_boundary_objects"]),
                post_next_reachable_objects=sorted(o for o in objs2 if o in reach2),
                next_boundary_protection=protection,
                protected_object_motion=protection,
                status="ok",
            )
            accepted = True
            break
        row["rejected_openers"] = rejected
        if not accepted:
            if os.path.exists(out_xml):
                os.remove(out_xml)          # the last write was a REJECTED state; never leave it behind
            row["status"] = "no_opener_decrements_hop"
            # Also emit the non-advancing opener under a SEPARATE root. Whether "the scene still has as
            # many hops to go" should end the chain or merely count as one more keyhole is a definition
            # call, and materializing both here means that call can be made without re-running the sweep.
            if fallback is not None:
                k, st, hop = fallback
                env.set_full_state(st)
                alt_xml = os.path.join(out_root, "xmls_nodecrement", _scene_id(xml) + ".xml")
                src_obs = {n: [float(v[0]), float(v[1]), float(v[2])]
                           for n, v in env.get_observation().items()}
                write_state_xml(src_obs, xml, alt_xml)
                env2 = namo_rl.RLEnvironment(alt_xml, cfg_file, False)
                env2.set_robot_goal(*goal)
                snap2 = get_region_snapshot(env2, goals_per_region=0, goal_radius=None,
                                            local_info_only=False, seed=seed, use_cpp_unified=True,
                                            use_xml_goal=True)
                p2 = shortest_region_path(snap2["adjacency"], snap2.get("robot_label") or "",
                                          snap2.get("goal_label") or "")
                row.update(nodecrement_opener=_jkey(k), nodecrement_out_xml=alt_xml,
                           nodecrement_hop_count=(len(p2) - 1) if p2 else -1)
    except Exception as exc:
        row["status"] = "error"
        row["error"] = f"{type(exc).__name__}: {exc}"
    return _done(row, t0)


def _done(row, t0):
    row["t_s"] = round(time.time() - t0, 2)
    return row


def _process_one_to_queue(task_id, task, result_queue):
    """Run one scene in its own process so the parent can enforce a hard deadline."""
    try:
        row = process_one(task)
    except BaseException as exc:  # keep an isolated worker failure from stalling the batch
        row = {
            "xml_path": task[0],
            "status": "worker_error",
            "error": f"{type(exc).__name__}: {exc}",
        }
    result_queue.put((task_id, row))


def process_tasks(tasks, *, workers, scene_timeout=None):
    """Yield scene rows, optionally terminating each isolated scene at a wall-clock deadline."""
    if workers < 1:
        raise ValueError("workers must be positive")
    if scene_timeout is None:
        if workers > 1:
            with Pool(workers) as pool:
                yield from pool.imap_unordered(process_one, tasks, chunksize=1)
        else:
            for task in tasks:
                yield process_one(task)
        return
    if scene_timeout <= 0:
        raise ValueError("scene_timeout must be positive")

    result_queue = Queue()
    pending = iter(enumerate(tasks))
    active = {}
    exhausted = False
    try:
        while active or not exhausted:
            while len(active) < workers and not exhausted:
                try:
                    task_id, task = next(pending)
                except StopIteration:
                    exhausted = True
                    break
                process = Process(
                    target=_process_one_to_queue,
                    args=(task_id, task, result_queue),
                )
                process.start()
                active[task_id] = (process, task, time.monotonic())

            try:
                task_id, row = result_queue.get(timeout=0.02)
            except Empty:
                pass
            else:
                entry = active.pop(task_id, None)
                if entry is not None:
                    entry[0].join()
                    yield row

            now = time.monotonic()
            for task_id, (process, task, started) in list(active.items()):
                if now - started < scene_timeout:
                    continue
                process.terminate()
                process.join(timeout=5.0)
                if process.is_alive():
                    process.kill()
                    process.join()
                del active[task_id]
                yield {
                    "xml_path": task[0],
                    "status": "scene_timeout",
                    "scene_timeout_s": scene_timeout,
                    "t_s": round(scene_timeout, 2),
                }
    finally:
        for process, _task, _started in active.values():
            if process.is_alive():
                process.terminate()
            process.join()
        result_queue.close()
        result_queue.join_thread()


def _scene_id(xml):
    """Stable, collision-free id. Round 1 flattens the pool-relative path; later rounds inherit the id
    that is already the emitted file's basename, so a scene keeps ONE identity down the whole chain."""
    p = os.path.realpath(xml)
    parts = p.split(os.sep)
    if len(parts) >= 2 and parts[-2] == "xmls":
        return parts[-1][:-4] if parts[-1].endswith(".xml") else parts[-1]
    tail = parts[-4:] if len(parts) >= 4 else parts
    return "__".join(tail).replace(".xml", "")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--manifest", required=True, help="file of scene XML paths, one per line")
    ap.add_argument("--out-root", required=True, help="root for emitted XMLs + rows.jsonl")
    ap.add_argument("--algo-yaml",
                    default=os.path.join(REPO, "python/namo/data_collection/"
                                               "region_opening_exhaustive_2push_multihop_car.yaml"))
    ap.add_argument("--config", default=os.path.join(REPO, "config/namo_config_complete_skill15_car_1x.yaml"))
    ap.add_argument("--start", type=int, default=0)
    ap.add_argument("--end", type=int, default=None)
    ap.add_argument("--workers", type=int, default=1)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--out-name", default="rows.jsonl")
    ap.add_argument("--kh1-chain-depth", type=int, default=1,
                    help="1 = only 1-push keyhole-1 openers (cheap); 2 = also exhaust 2-push chains "
                         "(needed for the ~70%% of scenes with no 1-push opener)")
    ap.add_argument("--kh1-timeout", type=float, default=None,
                    help="override region_timeout_per_neighbour_sec for the keyhole-1 sweep")
    ap.add_argument(
        "--scene-timeout",
        type=float,
        default=None,
        help="hard wall-clock deadline in seconds for each scene; timed-out scenes emit a row",
    )
    ap.add_argument(
        "--protect-object",
        action="append",
        default=[],
        help=(
            "movable object the selected opener must leave within the 2 mm / 1 degree "
            "independence tolerance; repeat for multiple objects"
        ),
    )
    a = ap.parse_args()

    xmls = [ln.strip() for ln in open(a.manifest) if ln.strip()]
    xmls = xmls[a.start:(a.end if a.end is not None else len(xmls))]
    os.makedirs(a.out_root, exist_ok=True)
    out_path = os.path.join(a.out_root, a.out_name)
    tasks = [(x, a.config, a.algo_yaml, a.out_root, a.seed, a.kh1_chain_depth, a.kh1_timeout,
              a.protect_object)
             for x in xmls]

    t0 = time.time()
    with open(out_path, "w") as f:
        for i, row in enumerate(
            process_tasks(tasks, workers=a.workers, scene_timeout=a.scene_timeout), 1
        ):
            f.write(json.dumps(row) + "\n")
            f.flush()
            print(f"{i}/{len(tasks)} {time.time()-t0:.0f}s {row['status']}", flush=True)
    print(f"done {len(tasks)} rows -> {out_path} in {time.time()-t0:.0f}s", flush=True)


if __name__ == "__main__":
    main()
