#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="${REPO_ROOT:-/home/dhruv/projects_dhruv/namo}"
REMOTE_HOST="${REMOTE_HOST:-tdn39@amarel.rutgers.edu}"
REMOTE_ROOT="${REMOTE_ROOT:-/scratch/tdn39/namo_less_depth}"
SSH_SOCKET="${SSH_SOCKET:-/tmp/amarel-cm.sock}"
REMOTE_SESSION_ROOT="${REMOTE_SESSION_ROOT:-$REMOTE_ROOT/robot_control/closed_loop_sessions_less_depth}"
LOCAL_SESSION_ROOT="${LOCAL_SESSION_ROOT:-$REPO_ROOT/robot_control/closed_loop_sessions_less_depth_exec}"
EXCLUDE_SUBSTR="${EXCLUDE_SUBSTR:-/2push/2hop/}"
REMOTE_PYTHON_BIN="${REMOTE_PYTHON_BIN:-/scratch/tdn39/envs/namo/bin/python}"
LOCAL_PYTHON_BIN="${LOCAL_PYTHON_BIN:-${PY:-python}}"
LOCAL_PYTHONPATH="${LOCAL_PYTHONPATH:-$REPO_ROOT/robot_control/src}"
ALLOW_PULL_INTO_SEED_ROOT="${ALLOW_PULL_INTO_SEED_ROOT:-0}"

LOCAL_SEED_ROOT="$(python - <<'PY' "$REPO_ROOT"
from pathlib import Path
import sys
print((Path(sys.argv[1]) / "robot_control" / "closed_loop_sessions_less_depth").resolve())
PY
)"
LOCAL_SESSION_ROOT_RESOLVED="$(python - <<'PY' "$LOCAL_SESSION_ROOT"
from pathlib import Path
import sys
print(Path(sys.argv[1]).expanduser().resolve())
PY
)"
if [[ "$LOCAL_SESSION_ROOT_RESOLVED" == "$LOCAL_SEED_ROOT" && "$ALLOW_PULL_INTO_SEED_ROOT" != "1" ]]; then
  echo "Refusing to pull prepared runs into the live seed root ($LOCAL_SEED_ROOT). Set LOCAL_SESSION_ROOT to an execution root or ALLOW_PULL_INTO_SEED_ROOT=1." >&2
  exit 1
fi

SSH_BASE=(ssh)
if [[ -S "$SSH_SOCKET" ]]; then
  SSH_BASE+=(-S "$SSH_SOCKET")
fi

RSYNC_RSH="ssh"
if [[ -S "$SSH_SOCKET" ]]; then
  RSYNC_RSH="ssh -S $SSH_SOCKET"
fi

mapfile -t TARGETS < <("${SSH_BASE[@]}" "$REMOTE_HOST" "$REMOTE_PYTHON_BIN" - <<'PY' "$REMOTE_SESSION_ROOT" "$EXCLUDE_SUBSTR"
from pathlib import Path
import json, sys
root = Path(sys.argv[1])
exclude = sys.argv[2]
for status_path in sorted(root.glob('**/iter_001/status.json')):
    if exclude and exclude in str(status_path):
        continue
    try:
        data = json.loads(status_path.read_text())
    except Exception:
        continue
    if data.get('state') != 'real_push_prepared':
        continue
    run_dir = status_path.parent.parent
    session_dir = run_dir.parent
    print(str(session_dir.relative_to(root)))
    print(run_dir.name)
PY
)

if [[ "${#TARGETS[@]}" -eq 0 ]]; then
  echo "No prepared remote iter_001 runs found under $REMOTE_SESSION_ROOT" >&2
  exit 1
fi

for ((i=0; i<${#TARGETS[@]}; i+=2)); do
  REL_SESSION="${TARGETS[$i]}"
  RUN_NAME="${TARGETS[$((i+1))]}"
  REMOTE_BASE="$REMOTE_SESSION_ROOT/$REL_SESSION"
  mkdir -p "$LOCAL_SESSION_ROOT/$REL_SESSION/$RUN_NAME/iter_001"
  rsync -az -e "$RSYNC_RSH" \
    "$REMOTE_HOST:$REMOTE_BASE/session_meta.json" \
    "$LOCAL_SESSION_ROOT/$REL_SESSION/"
  rsync -az -e "$RSYNC_RSH" \
    "$REMOTE_HOST:$REMOTE_BASE/$RUN_NAME/run_meta.json" \
    "$LOCAL_SESSION_ROOT/$REL_SESSION/$RUN_NAME/"
  rsync -az -e "$RSYNC_RSH" \
    "$REMOTE_HOST:$REMOTE_BASE/$RUN_NAME/iter_001/" \
    "$LOCAL_SESSION_ROOT/$REL_SESSION/$RUN_NAME/iter_001/"

  PYTHONPATH="$LOCAL_PYTHONPATH" "$LOCAL_PYTHON_BIN" \
    "$REPO_ROOT/robot_control/scripts/closed_loop_session.py" prepare-real-push \
    --session-dir "$LOCAL_SESSION_ROOT/$REL_SESSION" \
    --run "$RUN_NAME" \
    --iteration 1
done

echo "Pulled prepared iter_001 runs into $LOCAL_SESSION_ROOT"
