#!/bin/bash
# NAMO arm on rb_00034 (febcb94c), original layout. GUI visible (no --headless).
# Usage: run_namo_arm.sh <model_policy|model_search|uniform_search> <trialN> <seed>
arm="$1"; trial="$2"; seed="$3"
cd /home/dhruv/projects_dhruv/namo/namo_cpp && set -a && . env.robotlearning.sh && set +a
cd /home/dhruv/projects_dhruv/namo/robot_control/.claude/worktrees/safety-filter
# Planner Python from the main namo_cpp checkout, now clean at the merged commit that
# gives greedy_policy every blocker on the boundary. _diag_setup stamps it from NAMO_REPO.
export NAMO_REPO=/home/dhruv/projects_dhruv/namo/namo_cpp
# Package from this robot_control worktree, not the pip -e install in the shared checkout.
export PYTHONPATH="src:$NAMO_REPO/python:$NAMO_REPO/build_python:$NAMO_REPO/scripts:$NAMO_REPO/scripts/sandbox:$NAMO_REPO/scripts/pipeline:$SAGE_REPO"
/home/dhruv/miniconda3/envs/namo312/bin/python -c "import namo.planners.full_namo.full_namo_planner as m; print('planner from', m.__file__)"
echo "arm=$arm trial=$trial seed=$seed NAMO_REPO=$NAMO_REPO NAMO_SCRATCH=$NAMO_SCRATCH"
DIAG=/home/dhruv/projects_dhruv/namo/robot_control/real_exp/1hop_multi_int/results/real/2push__env__obstacle_0_movable__febcb94c/variants/original
CKPT=/home/dhruv/projects_dhruv/namo/ranking/models/HY5U_s2.ckpt
case "$arm" in
  model_policy)
    ARM_ARGS=(--best-first-prior model --scorer-ckpt "$CKPT" --exec-mode greedy_policy) ;;
  model_search)
    ARM_ARGS=(--best-first-prior model --scorer-ckpt "$CKPT" --max-planning-retries 1 --capture-sim-success) ;;
  uniform_search)
    ARM_ARGS=(--best-first-prior uniform --max-planning-retries 1 --capture-sim-success) ;;
  *) echo "unknown arm $arm"; exit 2 ;;
esac
exec /home/dhruv/miniconda3/envs/namo312/bin/python -u scripts/run_namo.py \
  --config config/real.yaml \
  --camera-service tcp://localhost:5556 \
  --algorithm full_namo \
  --local-search best_first \
  "${ARM_ARGS[@]}" \
  --shuffle-seed "$seed" \
  --goal 38.4845 65.4918 \
  --diag-path "$DIAG" \
  --run-name "$arm/$trial" \
  --capture-scene --record-video
