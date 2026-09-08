#!/bin/bash
#SBATCH --job-name=render_rb22hl
#SBATCH --partition=main
#SBATCH --account=general
#SBATCH --qos=normal
#SBATCH --array=0-1
#SBATCH --cpus-per-task=1
#SBATCH --mem=4G
#SBATCH --time=00:20:00

set -euo pipefail

RUN_ROOT=/scratch/tdn39/real_exp/results/navigation_sim/2push__v3_snap__hard_loose__rb_00022__f110e8c4_navigation_realstack_v1
export MUJOCO_GL=egl
export LD_LIBRARY_PATH="/scratch/tdn39/envs/namo/lib:/scratch/tdn39/mujoco_pip/lib:${LD_LIBRARY_PATH:-}"
export OMP_NUM_THREADS=1
export OPENCV_NUM_THREADS=1

case "$SLURM_ARRAY_TASK_ID" in
  0)
    mode=ignore
    renderer=render_ignore.py
    ;;
  1)
    mode=penalise
    renderer=render_penalise.py
    ;;
  *)
    echo "unexpected array task: $SLURM_ARRAY_TASK_ID" >&2
    exit 2
    ;;
esac

/scratch/tdn39/envs/namo/bin/python -u "$RUN_ROOT/$renderer" \
  --xml "$RUN_ROOT/env.xml" \
  --trajectory "$RUN_ROOT/nav_baseline_$mode/trajectory.jsonl" \
  --output "$RUN_ROOT/rb_00022__nav_baseline_${mode}__sim.mp4"
