#!/bin/bash
#SBATCH --job-name=nav_rb22hl
#SBATCH --partition=main
#SBATCH --account=general
#SBATCH --qos=normal
#SBATCH --cpus-per-task=1
#SBATCH --mem=4G
#SBATCH --time=00:15:00

set -euo pipefail

RUN_ROOT=/scratch/tdn39/real_exp/results/navigation_sim/2push__v3_snap__hard_loose__rb_00022__f110e8c4_navigation_realstack_v1
ROBOT_CONTROL=/scratch/tdn39/real_exp/code/robot_control_nav_realstack_v1
NAMO_REPO=/scratch/tdn39/real_exp/code/namo_cpp_nav_realstack_v1

export NAMO_CPP_DIR="$NAMO_REPO"
export NAMO_SCRATCH=/scratch/tdn39
export LD_LIBRARY_PATH="/scratch/tdn39/envs/namo/lib:/scratch/tdn39/mujoco_pip/lib:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="$ROBOT_CONTROL/src"
export OMP_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export MKL_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1
export OPENCV_NUM_THREADS=1

cd "$ROBOT_CONTROL"
/scratch/tdn39/envs/namo/bin/python -u scripts/run_navigation_baseline_sim.py \
  --xml "$RUN_ROOT/env.xml" \
  --out "$RUN_ROOT" \
  --mode both \
  --speed 0.3 \
  --timeout 90 \
  --max-sim-time 120 \
  --real-config config/real.yaml \
  --controller-config config/controller.yaml \
  --namo-config "$NAMO_REPO/config/namo_config_complete_skill15_car_1x.yaml"
