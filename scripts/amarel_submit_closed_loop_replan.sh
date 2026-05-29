#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="${REPO_ROOT:-/home/dhruv/projects_dhruv/namo}"
REMOTE_HOST="${REMOTE_HOST:-tdn39@amarel.rutgers.edu}"
BASE_REMOTE_ROOT="${BASE_REMOTE_ROOT:-/scratch/tdn39/namo_less_depth}"
REMOTE_ROOT="${REMOTE_ROOT:-/scratch/tdn39/namo_less_depth_runtime}"
SSH_SOCKET="${SSH_SOCKET:-/tmp/amarel-cm.sock}"
PARTITION="${PARTITION:-main-redhat}"
TIME_LIMIT="${TIME_LIMIT:-24:00:00}"
MEM_PER_TASK="${MEM_PER_TASK:-8G}"
CPUS_PER_TASK="${CPUS_PER_TASK:-1}"
CONDA_ENV_SPEC="${CONDA_ENV_SPEC:-/scratch/tdn39/envs/namo}"
CONDA_SH_PATH="${CONDA_SH_PATH:-/scratch/tdn39/miniforge3/etc/profile.d/conda.sh}"
PYTHON_BIN="${PYTHON_BIN:-/scratch/tdn39/envs/namo/bin/python}"
MJ_PATH="${MJ_PATH:-/home/tdn39/local/mujoco-3.3.6}"
ALLOW_REPLACE_REMOTE_REPLAN_REQUEST="${ALLOW_REPLACE_REMOTE_REPLAN_REQUEST:-0}"

SESSION_DIR="${SESSION_DIR:-}"
RUN_NAME="${RUN_NAME:-}"
ITERATION="${ITERATION:-}"
if [[ -z "$SESSION_DIR" || -z "$RUN_NAME" || -z "$ITERATION" ]]; then
  echo "Set SESSION_DIR, RUN_NAME, and ITERATION before calling this script." >&2
  exit 1
fi

LOCAL_SESSION_DIR="$(python - <<'PY' "$SESSION_DIR"
from pathlib import Path
import sys
print(Path(sys.argv[1]).expanduser().resolve())
PY
)"

ROBOT_CONTROL_DIR="$REPO_ROOT/robot_control"
SESSION_ROOT_NAME=""
REL_SESSION=""
if [[ "$LOCAL_SESSION_DIR" == "$ROBOT_CONTROL_DIR"/closed_loop_sessions*/* ]]; then
  LOCAL_SUFFIX="${LOCAL_SESSION_DIR#"$ROBOT_CONTROL_DIR"/}"
  SESSION_ROOT_NAME="${LOCAL_SUFFIX%%/*}"
  REL_SESSION="${LOCAL_SUFFIX#"$SESSION_ROOT_NAME"/}"
else
  echo "SESSION_DIR must live under robot_control/closed_loop_sessions*" >&2
  exit 1
fi

LOCAL_RUN_DIR="$LOCAL_SESSION_DIR/$RUN_NAME"
LOCAL_ITER_DIR="$LOCAL_RUN_DIR/iter_$(printf '%03d' "$ITERATION")"
if [[ ! -d "$LOCAL_ITER_DIR/scene_before" ]]; then
  echo "$LOCAL_ITER_DIR/scene_before missing" >&2
  exit 1
fi
for required in env.xml mid_obs.jsonl env.png scene.jpg; do
  if [[ ! -f "$LOCAL_ITER_DIR/scene_before/$required" ]]; then
    echo "$LOCAL_ITER_DIR/scene_before/$required missing. A reconstructable scene snapshot is required before remote replan submission." >&2
    exit 1
  fi
done

ITER_DIR_NAME="iter_$(printf '%03d' "$ITERATION")"
REQUEST_PATH="$LOCAL_ITER_DIR/remote_replan_request.json"

SSH_BASE=(ssh)
if [[ -S "$SSH_SOCKET" ]]; then
  SSH_BASE+=(-S "$SSH_SOCKET")
fi

RSYNC_RSH="ssh"
if [[ -S "$SSH_SOCKET" ]]; then
  RSYNC_RSH="ssh -S $SSH_SOCKET"
fi

if [[ -f "$REQUEST_PATH" ]]; then
  EXISTING_JOB_ID="$(python - <<'PY' "$REQUEST_PATH"
from pathlib import Path
import json
import sys

data = json.loads(Path(sys.argv[1]).read_text())
print(data.get("job_id", ""))
PY
)"
  EXISTING_JOB_ID="$(printf '%s' "$EXISTING_JOB_ID" | tr -d '[:space:]')"
  if [[ "$ALLOW_REPLACE_REMOTE_REPLAN_REQUEST" != "1" ]]; then
    echo "$REQUEST_PATH already exists for this iteration. Collect/clear the existing remote replan request first, or set ALLOW_REPLACE_REMOTE_REPLAN_REQUEST=1 to replace it." >&2
    if [[ -n "$EXISTING_JOB_ID" ]]; then
      echo "Existing remote job id: $EXISTING_JOB_ID" >&2
    fi
    exit 1
  fi
  if [[ -n "$EXISTING_JOB_ID" ]]; then
    EXISTING_STATE="$("${SSH_BASE[@]}" "$REMOTE_HOST" "sacct -j '$EXISTING_JOB_ID' --format=State -n | head -1 | tr -d ' '")"
    if [[ -z "$EXISTING_STATE" ]]; then
      EXISTING_STATE="$("${SSH_BASE[@]}" "$REMOTE_HOST" "squeue -h -j '$EXISTING_JOB_ID' -o '%T' | head -1 | tr -d ' '")"
    fi
    case "$EXISTING_STATE" in
      PENDING|CONFIGURING|RUNNING|COMPLETING|SUSPENDED|STAGE_OUT|RESIZING|SIGNALING)
        "${SSH_BASE[@]}" "$REMOTE_HOST" "scancel '$EXISTING_JOB_ID'" >/dev/null 2>&1 || true
        ;;
    esac
  fi
fi

LOCAL_NAMO_HASH="$(python - <<'PY' "$REPO_ROOT/namo_cpp"
from pathlib import Path
import hashlib
import sys

root = Path(sys.argv[1])
skip_dirs = {"build", "build_python", ".git", "__pycache__", ".pytest_cache"}
h = hashlib.sha256()
for path in sorted(p for p in root.rglob("*") if p.is_file()):
    rel = path.relative_to(root)
    if any(part in skip_dirs for part in rel.parts):
        continue
    if path.suffix == ".pyc":
        continue
    h.update(str(rel).encode("utf-8"))
    h.update(b"\0")
    h.update(path.read_bytes())
    h.update(b"\0")
print(h.hexdigest())
PY
)"
REMOTE_NAMO_HASH="$("${SSH_BASE[@]}" "$REMOTE_HOST" "'$PYTHON_BIN' - <<'PY' '$BASE_REMOTE_ROOT/namo_cpp'
from pathlib import Path
import hashlib
import sys

root = Path(sys.argv[1])
skip_dirs = {'build', 'build_python', '.git', '__pycache__', '.pytest_cache'}
h = hashlib.sha256()
for path in sorted(p for p in root.rglob('*') if p.is_file()):
    rel = path.relative_to(root)
    if any(part in skip_dirs for part in rel.parts):
        continue
    if path.suffix == '.pyc':
        continue
    h.update(str(rel).encode('utf-8'))
    h.update(b'\0')
    h.update(path.read_bytes())
    h.update(b'\0')
print(h.hexdigest())
PY")"
REMOTE_NAMO_HASH="$(printf '%s' "$REMOTE_NAMO_HASH" | tr -d '[:space:]')"
if [[ "$LOCAL_NAMO_HASH" != "$REMOTE_NAMO_HASH" ]]; then
  echo "Local namo_cpp does not match $BASE_REMOTE_ROOT/namo_cpp on Amarel. Resync/rebuild the Amarel seed root before submitting remote fallback replans." >&2
  exit 1
fi

mapfile -t REMOTE_BUILD_FIELDS < <("${SSH_BASE[@]}" "$REMOTE_HOST" "'$PYTHON_BIN' - <<'PY' '$BASE_REMOTE_ROOT/namo_cpp/build_python/namo_rl_build_manifest.json'
from pathlib import Path
import json
import sys

path = Path(sys.argv[1])
if not path.exists():
    print('')
    print('0')
    raise SystemExit(0)
data = json.loads(path.read_text())
print(data.get('source_hash', ''))
print('1' if data.get('artifacts') else '0')
PY")
REMOTE_BUILD_HASH="${REMOTE_BUILD_FIELDS[0]:-}"
REMOTE_BUILD_HAS_ARTIFACTS="${REMOTE_BUILD_FIELDS[1]:-0}"
REMOTE_BUILD_HASH="$(printf '%s' "$REMOTE_BUILD_HASH" | tr -d '[:space:]')"
REMOTE_BUILD_HAS_ARTIFACTS="$(printf '%s' "$REMOTE_BUILD_HAS_ARTIFACTS" | tr -d '[:space:]')"
if [[ -z "$REMOTE_BUILD_HASH" || "$REMOTE_BUILD_HAS_ARTIFACTS" != "1" ]]; then
  echo "$BASE_REMOTE_ROOT/namo_cpp/build_python is missing a valid namo_rl_build_manifest.json with artifacts on Amarel. Rebuild the base remote root before submitting remote fallback replans." >&2
  exit 1
fi
if [[ "$REMOTE_BUILD_HASH" != "$REMOTE_NAMO_HASH" ]]; then
  echo "Amarel base namo_rl build manifest does not match $BASE_REMOTE_ROOT/namo_cpp source. Rebuild the base remote root before submitting remote fallback replans." >&2
  exit 1
fi

REQUEST_TOKEN="$(python - <<'PY'
import time
import uuid

print(f"{int(time.time())}_{uuid.uuid4().hex[:8]}")
PY
)"
REMOTE_JOB_ROOT="$REMOTE_ROOT/$SESSION_ROOT_NAME/$REL_SESSION/$RUN_NAME/$ITER_DIR_NAME/request_$REQUEST_TOKEN"
REMOTE_SESSION_DIR="$REMOTE_JOB_ROOT/robot_control/$SESSION_ROOT_NAME/$REL_SESSION"

"${SSH_BASE[@]}" "$REMOTE_HOST" "rm -rf '$REMOTE_JOB_ROOT' && mkdir -p '$REMOTE_JOB_ROOT/robot_control' '$REMOTE_SESSION_DIR'"

rsync -az --delete \
  --exclude='__pycache__/' \
  --exclude='*.pyc' \
  --exclude='.git/' \
  --exclude='.pytest_cache/' \
  -e "$RSYNC_RSH" \
  "$REPO_ROOT/robot_control/src" \
  "$REPO_ROOT/robot_control/scripts" \
  "$REPO_ROOT/robot_control/config" \
  "$REMOTE_HOST:$REMOTE_JOB_ROOT/robot_control/"

"${SSH_BASE[@]}" "$REMOTE_HOST" "rsync -a --delete '$BASE_REMOTE_ROOT/namo_cpp/' '$REMOTE_JOB_ROOT/namo_cpp/'"

rsync -az -e "$RSYNC_RSH" \
  "$LOCAL_SESSION_DIR/session_meta.json" \
  "$REMOTE_HOST:$REMOTE_SESSION_DIR/"

rsync -az -e "$RSYNC_RSH" \
  "$LOCAL_RUN_DIR/" \
  "$REMOTE_HOST:$REMOTE_SESSION_DIR/$RUN_NAME/"

REMOTE_CMD=$(cat <<EOF
set -euo pipefail
cd "$REMOTE_JOB_ROOT"
sbatch --parsable --partition="$PARTITION" --time="$TIME_LIMIT" --mem="$MEM_PER_TASK" --cpus-per-task="$CPUS_PER_TASK" \
  --export=ALL,REPO_ROOT="$REMOTE_JOB_ROOT",SESSION_DIR="$REMOTE_SESSION_DIR",RUN_NAME="$RUN_NAME",ITERATION="$ITERATION",PYTHON_BIN="$PYTHON_BIN",CONDA_ENV_SPEC="$CONDA_ENV_SPEC",CONDA_SH_PATH="$CONDA_SH_PATH",MJ_PATH="$MJ_PATH" \
  robot_control/scripts/amarel_oneoff_closed_loop_replan.slurm
EOF
)

SBATCH_OUTPUT="$("${SSH_BASE[@]}" "$REMOTE_HOST" "$REMOTE_CMD")"
SBATCH_OUTPUT="${SBATCH_OUTPUT//$'\r'/}"
JOB_ID="${SBATCH_OUTPUT%%;*}"
JOB_ID="$(printf '%s' "$JOB_ID" | tr -d '[:space:]')"
if [[ -z "$JOB_ID" ]]; then
  echo "failed to parse sbatch job id from output: $SBATCH_OUTPUT" >&2
  exit 1
fi

python - <<'PY' "$LOCAL_ITER_DIR" "$REQUEST_PATH" "$JOB_ID" "$PARTITION" "$REMOTE_ROOT" "$REMOTE_JOB_ROOT" "$REMOTE_SESSION_DIR" "$REQUEST_TOKEN" "$LOCAL_NAMO_HASH" "$LOCAL_SESSION_DIR/session_meta.json" "$LOCAL_RUN_DIR/run_meta.json"
from pathlib import Path
import hashlib
import json
import sys
import time

iter_dir = Path(sys.argv[1])
request_path = Path(sys.argv[2])
job_id = sys.argv[3]
partition = sys.argv[4]
remote_root = sys.argv[5]
remote_job_root = sys.argv[6]
remote_session_dir = sys.argv[7]
request_token = sys.argv[8]
local_namo_hash = sys.argv[9]
session_meta_path = Path(sys.argv[10])
run_meta_path = Path(sys.argv[11])

scene_before = iter_dir / "scene_before"
hashes = {}
for name in ("env.xml", "mid_obs.jsonl", "env.png", "scene.jpg"):
    path = scene_before / name
    hashes[name] = hashlib.sha256(path.read_bytes()).hexdigest()

payload = {
    "job_id": job_id,
    "partition": partition,
    "submitted_at_epoch": time.time(),
    "remote_root_base": remote_root,
    "remote_job_root": remote_job_root,
    "remote_session_dir": remote_session_dir,
    "request_token": request_token,
    "local_namo_cpp_hash": local_namo_hash,
    "session_meta_hash": hashlib.sha256(session_meta_path.read_bytes()).hexdigest(),
    "run_meta_hash": hashlib.sha256(run_meta_path.read_bytes()).hexdigest(),
    "scene_before_hashes": hashes,
}
request_path.write_text(json.dumps(payload, indent=2) + "\n")
PY

printf 'Submitted batch job %s\n' "$JOB_ID"
printf 'Remote job root: %s\n' "$REMOTE_JOB_ROOT"
