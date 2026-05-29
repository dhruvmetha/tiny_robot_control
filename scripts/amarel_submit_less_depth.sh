#!/usr/bin/env bash
set -euo pipefail

REMOTE_HOST="${REMOTE_HOST:-tdn39@amarel.rutgers.edu}"
REMOTE_ROOT="${REMOTE_ROOT:-/scratch/tdn39/namo_less_depth}"
SSH_SOCKET="${SSH_SOCKET:-/tmp/amarel-cm.sock}"
PARTITION="${PARTITION:-main-redhat}"
TIME_LIMIT="${TIME_LIMIT:-24:00:00}"
MEM_PER_TASK="${MEM_PER_TASK:-8G}"
CPUS_PER_TASK="${CPUS_PER_TASK:-1}"
CONDA_ENV_SPEC="${CONDA_ENV_SPEC:-${CONDA_ENV_NAME:-namo312}}"
CONDA_SH_PATH="${CONDA_SH_PATH:-}"
PYTHON_BIN="${PYTHON_BIN:-}"
MJ_PATH="${MJ_PATH:-}"
EXCLUDE_SUBSTR="${EXCLUDE_SUBSTR:-/2push/2hop/env1/}"
ARRAY_CONCURRENCY="${ARRAY_CONCURRENCY:-}"

if [[ -z "$CONDA_SH_PATH" ]]; then
  for candidate in \
    "/scratch/${REMOTE_HOST%@*}/miniforge3/etc/profile.d/conda.sh" \
    "/scratch/tdn39/miniforge3/etc/profile.d/conda.sh" \
    '$HOME/miniforge3/etc/profile.d/conda.sh' \
    '$HOME/miniconda3/etc/profile.d/conda.sh'; do
    if [[ "$candidate" != '$HOME/miniforge3/etc/profile.d/conda.sh' && "$candidate" != '$HOME/miniconda3/etc/profile.d/conda.sh' ]]; then
      CONDA_SH_PATH="$candidate"
      break
    fi
  done
fi

if [[ -z "$PYTHON_BIN" ]]; then
  if [[ "$CONDA_ENV_SPEC" = /* ]]; then
    PYTHON_BIN="$CONDA_ENV_SPEC/bin/python"
  else
    PYTHON_BIN="\$HOME/miniconda3/envs/${CONDA_ENV_SPEC}/bin/python"
  fi
fi

SSH_BASE=(ssh)
if [[ -S "$SSH_SOCKET" ]]; then
  SSH_BASE+=(-S "$SSH_SOCKET")
fi

REMOTE_CMD=$(cat <<EOF
set -euo pipefail
cd "$REMOTE_ROOT"
SESSION_ROOT="$REMOTE_ROOT/robot_control/closed_loop_sessions_less_depth"
SESSION_COUNT=\$(find "\$SESSION_ROOT" -mindepth 5 -maxdepth 5 -type d -path '*/sessions/*' | sort | grep -v "$EXCLUDE_SUBSTR" | wc -l)
RUNS_PER_SESSION=6
COUNT=\$((SESSION_COUNT * RUNS_PER_SESSION))
if [[ "\$COUNT" -eq 0 ]]; then
  echo "No eligible run targets found under \$SESSION_ROOT" >&2
  exit 1
fi
ARRAY_MAX=\$((COUNT - 1))
echo "Submitting \$COUNT run-level tasks across \$SESSION_COUNT sessions"
ARRAY_SPEC="0-\$ARRAY_MAX"
if [[ -n "$ARRAY_CONCURRENCY" ]]; then
  ARRAY_SPEC="\${ARRAY_SPEC}%$ARRAY_CONCURRENCY"
fi
sbatch --partition="$PARTITION" --time="$TIME_LIMIT" --mem="$MEM_PER_TASK" --cpus-per-task="$CPUS_PER_TASK" --array="\$ARRAY_SPEC" --export=ALL,REPO_ROOT="$REMOTE_ROOT",SESSION_ROOT="$REMOTE_ROOT/robot_control/closed_loop_sessions_less_depth",SUMMARY_ROOT="$REMOTE_ROOT/_amarel_summaries",PYTHON_BIN="$PYTHON_BIN",CONDA_ENV_SPEC="$CONDA_ENV_SPEC",CONDA_SH_PATH="$CONDA_SH_PATH",MJ_PATH="$MJ_PATH",EXCLUDE_SUBSTR="$EXCLUDE_SUBSTR" robot_control/scripts/amarel_seed_less_depth_array.slurm
EOF
)

"${SSH_BASE[@]}" "$REMOTE_HOST" "$REMOTE_CMD"
