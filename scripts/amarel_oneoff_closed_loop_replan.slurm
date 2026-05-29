#!/bin/bash
#SBATCH --partition=main-redhat
#SBATCH --job-name=namo-replan
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=1
#SBATCH --mem=8G
#SBATCH --time=24:00:00
#SBATCH --output=%x-%j.out

set -euo pipefail

REPO_ROOT="${REPO_ROOT:?REPO_ROOT is required}"
SESSION_DIR="${SESSION_DIR:?SESSION_DIR is required}"
RUN_NAME="${RUN_NAME:?RUN_NAME is required}"
ITERATION="${ITERATION:?ITERATION is required}"
CONDA_ENV_SPEC="${CONDA_ENV_SPEC:-${CONDA_ENV_NAME:-namo312}}"
CONDA_SH_PATH="${CONDA_SH_PATH:-}"
MJ_PATH="${MJ_PATH:-}"

if [[ -z "${PYTHON_BIN:-}" ]]; then
  if [[ -d "$CONDA_ENV_SPEC" ]]; then
    PYTHON_BIN="$CONDA_ENV_SPEC/bin/python"
  else
    PYTHON_BIN="$HOME/miniconda3/envs/${CONDA_ENV_SPEC}/bin/python"
  fi
fi

if [[ -z "$CONDA_SH_PATH" ]]; then
  for candidate in \
    "/scratch/$USER/miniforge3/etc/profile.d/conda.sh" \
    "$HOME/miniforge3/etc/profile.d/conda.sh" \
    "$HOME/miniconda3/etc/profile.d/conda.sh"; do
    if [[ -f "$candidate" ]]; then
      CONDA_SH_PATH="$candidate"
      break
    fi
  done
fi

cd "$REPO_ROOT"
if [[ -n "$CONDA_SH_PATH" && -f "$CONDA_SH_PATH" ]]; then
  # shellcheck disable=SC1090
  source "$CONDA_SH_PATH"
  conda activate "$CONDA_ENV_SPEC"
fi

export PYTHONPATH="$REPO_ROOT/robot_control/src:$REPO_ROOT/namo_cpp/python:$REPO_ROOT/namo_cpp/build_python${PYTHONPATH:+:$PYTHONPATH}"
export PATH="/usr/bin:$(dirname "$PYTHON_BIN"):$PATH"
export PYTHONNOUSERSITE=1
export OMP_NUM_THREADS="${SLURM_CPUS_PER_TASK:-1}"
export OPENBLAS_NUM_THREADS="${SLURM_CPUS_PER_TASK:-1}"
export MKL_NUM_THREADS="${SLURM_CPUS_PER_TASK:-1}"
export NUMEXPR_NUM_THREADS="${SLURM_CPUS_PER_TASK:-1}"
export VECLIB_MAXIMUM_THREADS="${SLURM_CPUS_PER_TASK:-1}"
if [[ -n "$MJ_PATH" && -d "$MJ_PATH/lib" ]]; then
  export LD_LIBRARY_PATH="$MJ_PATH/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi

echo "SESSION_DIR=$SESSION_DIR"
echo "RUN_NAME=$RUN_NAME"
echo "ITERATION=$ITERATION"

"$PYTHON_BIN" robot_control/scripts/closed_loop_session.py replan-full-search-only \
  --session-dir "$SESSION_DIR" \
  --run "$RUN_NAME" \
  --iteration "$ITERATION"
