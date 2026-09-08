#!/bin/bash
#SBATCH --job-name=nav_bridge_build
#SBATCH --partition=main
#SBATCH --account=general
#SBATCH --qos=normal
#SBATCH --cpus-per-task=16
#SBATCH --mem=24G
#SBATCH --time=00:30:00

set -euo pipefail

NAMO_REPO=/scratch/tdn39/real_exp/code/namo_cpp_nav_realstack_v1
NAMO_ENV=/scratch/tdn39/envs/namo

export NAMO_REPO
export NAMO_PARENT=/scratch/tdn39/Robotics/namo
export NAMO_SAGE=/scratch/tdn39/Robotics/namo/sage_learning
export NAMO_CONDA_ENV="$NAMO_ENV"
export NAMO_SCRATCH=/scratch/tdn39
export MJ_PATH=/scratch/tdn39/mujoco_pip
export CPLUS_INCLUDE_PATH="$NAMO_ENV/include:${CPLUS_INCLUDE_PATH:-}"
export LIBRARY_PATH="$NAMO_ENV/lib:${LIBRARY_PATH:-}"
export CMAKE_PREFIX_PATH="$NAMO_ENV:${CMAKE_PREFIX_PATH:-}"
export PYTHON_BIN="$NAMO_ENV/bin/python"

source /scratch/tdn39/Robotics/namo/namo_cpp/scripts/amarel/activate.sh
cd "$NAMO_REPO"

NAMO_MARCH=x86-64-v3 ./build_python_bindings.sh
cmake --build build_python --target test_wheel_spin test_config_manager_schema -j16
./build_python/test_wheel_spin
./build_python/test_config_manager_schema

export PYTHONPATH="$NAMO_REPO/build_python:$NAMO_REPO/python"
"$NAMO_ENV/bin/python" -m pytest -q \
  python/tests/test_wheel_control_bridge.py \
  python/tests/test_car_reset_baseline.py

"$NAMO_ENV/bin/python" -c \
  'import namo_rl; print("binding:", namo_rl.__file__)'
sha256sum build_python/namo_rl*.so config/wavefront_inflation.yaml
