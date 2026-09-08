#!/bin/bash
#SBATCH --job-name=nav_realstack100
#SBATCH --partition=main
#SBATCH --account=general
#SBATCH --qos=normal
#SBATCH --cpus-per-task=1
#SBATCH --mem=4G
#SBATCH --time=00:15:00

set -euo pipefail

OUT=${OUT:-/scratch/tdn39/real_exp/results/twohop_v3_multiobject/navigation_realstack_all100_margin_0p1cm_v1}
SCENE_MANIFEST=${SCENE_MANIFEST:-$OUT/scene_manifest.txt}
ROBOT_CONTROL=${ROBOT_CONTROL:-/scratch/tdn39/real_exp/code/robot_control_nav_realstack_v1}
NAMO_REPO=${NAMO_REPO:-/scratch/tdn39/real_exp/code/namo_cpp_nav_realstack_v1}

line_number=$((SLURM_ARRAY_TASK_ID + 1))
scene_dir=$(sed -n "${line_number}p" "$SCENE_MANIFEST")
if [ -z "$scene_dir" ]; then
  echo "No scene on manifest line $line_number" >&2
  exit 2
fi
if [ ! -f "$scene_dir/env.xml" ]; then
  echo "Scene XML does not exist: $scene_dir/env.xml" >&2
  exit 2
fi

profile=$(basename "$(dirname "$scene_dir")")
scene_id=$(basename "$scene_dir")
scene_out="$OUT/scenes/$profile/$scene_id"

export NAMO_CPP_DIR="$NAMO_REPO"
export NAMO_SCRATCH=/scratch/tdn39
export LD_LIBRARY_PATH="/scratch/tdn39/envs/namo/lib:/scratch/tdn39/mujoco_pip/lib:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="$ROBOT_CONTROL/src"
export OMP_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export MKL_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1
export OPENCV_NUM_THREADS=1

mkdir -p "$scene_out"
cd "$ROBOT_CONTROL"
overwrite_args=()
if [ "${ALLOW_OVERWRITE:-0}" = "1" ]; then
  overwrite_args+=(--allow-overwrite)
fi
/scratch/tdn39/envs/namo/bin/python -u \
  scripts/run_navigation_baseline_sim.py \
  --xml "$scene_dir/env.xml" \
  --out "$scene_out" \
  --mode both \
  --speed 0.4 \
  --timeout 90 \
  --max-sim-time 120 \
  "${overwrite_args[@]}" \
  --real-config config/real.yaml \
  --controller-config config/controller.yaml \
  --namo-config "$NAMO_REPO/config/namo_config_complete_skill15_car_1x.yaml"
