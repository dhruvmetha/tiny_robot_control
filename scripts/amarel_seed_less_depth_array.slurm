#!/bin/bash
#SBATCH --partition=main-redhat
#SBATCH --job-name=namo-lessdepth
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=1
#SBATCH --mem=8G
#SBATCH --time=24:00:00
#SBATCH --output=%x-%A_%a.out

set -euo pipefail

REPO_ROOT="${REPO_ROOT:-/scratch/$USER/namo_less_depth}"
SESSION_ROOT="${SESSION_ROOT:-$REPO_ROOT/robot_control/closed_loop_sessions_less_depth}"
SUMMARY_ROOT="${SUMMARY_ROOT:-$REPO_ROOT/_amarel_summaries}"
CONDA_ENV_SPEC="${CONDA_ENV_SPEC:-${CONDA_ENV_NAME:-namo312}}"
CONDA_SH_PATH="${CONDA_SH_PATH:-}"
MJ_PATH="${MJ_PATH:-}"
EXCLUDE_SUBSTR="${EXCLUDE_SUBSTR:-/2push/2hop/env1/}"

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

if [[ -z "${PYTHON_BIN:-}" ]]; then
  if [[ -d "$CONDA_ENV_SPEC" ]]; then
    PYTHON_BIN="$CONDA_ENV_SPEC/bin/python"
  else
    PYTHON_BIN="$HOME/miniconda3/envs/${CONDA_ENV_SPEC}/bin/python"
  fi
fi

mkdir -p "$SUMMARY_ROOT"
cd "$REPO_ROOT"
if [[ -n "$CONDA_SH_PATH" && -f "$CONDA_SH_PATH" ]]; then
  # shellcheck disable=SC1091
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

mapfile -t SESSIONS < <(
  find "$SESSION_ROOT" -mindepth 5 -maxdepth 5 -type d -path '*/sessions/*' | sort | grep -v "$EXCLUDE_SUBSTR"
)
RUN_ORDER=(
  primitive_run1
  primitive_run2
  primitive_run3
  random_rollout_run1
  random_rollout_run2
  random_rollout_run3
)
TARGETS=()

for SESSION in "${SESSIONS[@]}"; do
  for RUN_NAME in "${RUN_ORDER[@]}"; do
    TARGETS+=("${SESSION}|${RUN_NAME}")
  done
done

if [[ "${#TARGETS[@]}" -eq 0 ]]; then
  echo "No eligible run targets found under $SESSION_ROOT" >&2
  exit 1
fi

if [[ -z "${SLURM_ARRAY_TASK_ID:-}" ]]; then
  echo "SLURM_ARRAY_TASK_ID is not set" >&2
  exit 1
fi

TARGET="${TARGETS[$SLURM_ARRAY_TASK_ID]}"
SESSION="${TARGET%%|*}"
RUN_NAME="${TARGET##*|}"
REL_SESSION="${SESSION#$SESSION_ROOT/}"
SAFE_NAME="$(echo "${REL_SESSION}__${RUN_NAME}" | tr '/ ' '__')"
SUMMARY_PATH="$SUMMARY_ROOT/${SAFE_NAME}.json"

echo "SESSION=$SESSION"
echo "RUN_NAME=$RUN_NAME"
echo "SUMMARY_PATH=$SUMMARY_PATH"

"$PYTHON_BIN" robot_control/scripts/seed_iter001_worker.py \
  --closed-loop-root "$SESSION_ROOT" \
  --session-dir "$SESSION" \
  --run-name "$RUN_NAME" \
  --summary-path "$SUMMARY_PATH" \
  --exclude-session-substr "$EXCLUDE_SUBSTR"
