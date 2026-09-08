#!/bin/bash
#SBATCH --job-name=nav_wheel_diag
#SBATCH --partition=main
#SBATCH --account=general
#SBATCH --qos=normal
#SBATCH --cpus-per-task=1
#SBATCH --mem=4G
#SBATCH --time=00:05:00

set -euo pipefail

NAMO_REPO=/scratch/tdn39/real_exp/code/namo_cpp_nav_realstack_v1
export LD_LIBRARY_PATH="/scratch/tdn39/envs/namo/lib:/scratch/tdn39/mujoco_pip/lib:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="$NAMO_REPO/build_python"

/scratch/tdn39/envs/namo/bin/python \
  /scratch/tdn39/real_exp/results/twohop_v3_multiobject/navigation_realstack_all100_margin_0p1cm_v1/jobs/diagnose_wheel_direction.py \
  "$NAMO_REPO" \
  /scratch/tdn39/real_exp/environments/twohop_candidates_topology_distinct_100/hard1-hard1/twohop_multi_00524/env.xml

