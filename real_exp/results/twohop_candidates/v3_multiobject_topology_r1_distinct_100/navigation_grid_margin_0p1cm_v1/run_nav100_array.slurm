#!/bin/bash
# Offline pure-navigation screen for a fixed scene manifest.
# Required exports: OUT, SCENE_MANIFEST, ROBOT_CONTROL, NAMO_REPO, NAMO_BINDINGS.
# Optional: SCENES_PER_TASK (default 5).
#SBATCH --partition=main
#SBATCH --time=00:15:00
#SBATCH --mem=4G
#SBATCH --cpus-per-task=1

set -euo pipefail

: "${OUT:?set OUT}"
: "${SCENE_MANIFEST:?set SCENE_MANIFEST}"
: "${ROBOT_CONTROL:?set ROBOT_CONTROL}"
: "${NAMO_REPO:?set NAMO_REPO}"
: "${NAMO_BINDINGS:?set NAMO_BINDINGS}"

SCENES_PER_TASK=${SCENES_PER_TASK:-5}
START_LINE=$((SLURM_ARRAY_TASK_ID * SCENES_PER_TASK + 1))
END_LINE=$(((SLURM_ARRAY_TASK_ID + 1) * SCENES_PER_TASK))
mapfile -t SCENES < <(sed -n "${START_LINE},${END_LINE}p" "$SCENE_MANIFEST")

if [ "${#SCENES[@]}" -eq 0 ]; then
  echo "No scenes assigned to task ${SLURM_ARRAY_TASK_ID}" >&2
  exit 2
fi

export LD_LIBRARY_PATH="/scratch/tdn39/envs/namo/lib:/scratch/tdn39/mujoco_pip/lib:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="$ROBOT_CONTROL/src:$NAMO_BINDINGS:$NAMO_REPO/python"
export OMP_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export MKL_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1
export OPENCV_NUM_THREADS=1

mkdir -p "$OUT/shards" "$OUT/images"
cd "$ROBOT_CONTROL"
/scratch/tdn39/envs/namo/bin/python scripts/run_navigation_baseline.py \
  "${SCENES[@]}" \
  --config "$NAMO_REPO/config/namo_config_complete_skill15_car_1x.yaml" \
  --csv "$OUT/shards/shard_${SLURM_ARRAY_TASK_ID}.csv" \
  --images "$OUT/images"
