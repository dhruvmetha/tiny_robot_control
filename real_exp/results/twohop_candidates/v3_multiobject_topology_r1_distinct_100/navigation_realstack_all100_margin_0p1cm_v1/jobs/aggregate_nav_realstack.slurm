#!/bin/bash
#SBATCH --job-name=nav_realstack_reduce
#SBATCH --partition=main
#SBATCH --account=general
#SBATCH --qos=normal
#SBATCH --cpus-per-task=1
#SBATCH --mem=4G
#SBATCH --time=00:15:00

set -euo pipefail

OUT=${OUT:-/scratch/tdn39/real_exp/results/twohop_v3_multiobject/navigation_realstack_all100_margin_0p1cm_v1}
SCENE_MANIFEST=${SCENE_MANIFEST:-$OUT/scene_manifest.txt}

/scratch/tdn39/envs/namo/bin/python \
  "$OUT/jobs/aggregate_nav_realstack.py" \
  --root "$OUT" \
  --manifest "$SCENE_MANIFEST" \
  --expected-count 100
