#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="${REPO_ROOT:-/home/dhruv/projects_dhruv/namo}"
REMOTE_HOST="${REMOTE_HOST:-tdn39@amarel.rutgers.edu}"
REMOTE_ROOT="${REMOTE_ROOT:-/scratch/tdn39/namo_less_depth_runtime}"
SSH_SOCKET="${SSH_SOCKET:-/tmp/amarel-cm.sock}"
WAIT_FOR_JOB="${WAIT_FOR_JOB:-1}"
JOB_ID="${JOB_ID:-}"
PY_LOCAL="${PY_LOCAL:-/home/dhruv/miniconda3/envs/namo312/bin/python}"

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

LOCAL_ITER_DIR="$LOCAL_SESSION_DIR/$RUN_NAME/iter_$(printf '%03d' "$ITERATION")"
REQUEST_PATH="$LOCAL_ITER_DIR/remote_replan_request.json"
if [[ ! -f "$REQUEST_PATH" ]]; then
  echo "$REQUEST_PATH missing; collect requires a prior amarel_submit_closed_loop_replan.sh request manifest" >&2
  exit 1
fi

mapfile -t REQUEST_FIELDS < <("$PY_LOCAL" - <<'PY' "$REQUEST_PATH"
from pathlib import Path
import json
import sys

data = json.loads(Path(sys.argv[1]).read_text())
print(data.get("job_id", ""))
print(data.get("remote_job_root", ""))
print(data.get("remote_session_dir", ""))
print(data.get("session_meta_hash", ""))
print(data.get("run_meta_hash", ""))
hashes = data.get("scene_before_hashes") or {}
print(hashes.get("env.xml", ""))
print(hashes.get("mid_obs.jsonl", ""))
print(hashes.get("env.png", ""))
print(hashes.get("scene.jpg", ""))
PY
)

JOB_ID_FROM_REQUEST="${REQUEST_FIELDS[0]:-}"
REMOTE_JOB_ROOT="${REQUEST_FIELDS[1]:-}"
REMOTE_SESSION_DIR="${REQUEST_FIELDS[2]:-}"
EXPECTED_SESSION_META_HASH="${REQUEST_FIELDS[3]:-}"
EXPECTED_RUN_META_HASH="${REQUEST_FIELDS[4]:-}"
EXPECTED_ENV_HASH="${REQUEST_FIELDS[5]:-}"
EXPECTED_MIDOBS_HASH="${REQUEST_FIELDS[6]:-}"
EXPECTED_ENV_PNG_HASH="${REQUEST_FIELDS[7]:-}"
EXPECTED_SCENE_JPG_HASH="${REQUEST_FIELDS[8]:-}"
if [[ -z "$REMOTE_JOB_ROOT" || -z "$REMOTE_SESSION_DIR" || -z "$EXPECTED_SESSION_META_HASH" || -z "$EXPECTED_RUN_META_HASH" || -z "$EXPECTED_ENV_HASH" || -z "$EXPECTED_MIDOBS_HASH" || -z "$EXPECTED_ENV_PNG_HASH" || -z "$EXPECTED_SCENE_JPG_HASH" ]]; then
  echo "$REQUEST_PATH missing required remote replan metadata" >&2
  exit 1
fi
if [[ -z "$JOB_ID" ]]; then
  JOB_ID="$JOB_ID_FROM_REQUEST"
fi

LOCAL_SESSION_META_HASH="$(sha256sum "$LOCAL_SESSION_DIR/session_meta.json" | awk '{print $1}')"
LOCAL_RUN_META_HASH="$(sha256sum "$LOCAL_SESSION_DIR/$RUN_NAME/run_meta.json" | awk '{print $1}')"
LOCAL_ENV_HASH="$(sha256sum "$LOCAL_ITER_DIR/scene_before/env.xml" | awk '{print $1}')"
LOCAL_MIDOBS_HASH="$(sha256sum "$LOCAL_ITER_DIR/scene_before/mid_obs.jsonl" | awk '{print $1}')"
LOCAL_ENV_PNG_HASH="$(sha256sum "$LOCAL_ITER_DIR/scene_before/env.png" | awk '{print $1}')"
LOCAL_SCENE_JPG_HASH="$(sha256sum "$LOCAL_ITER_DIR/scene_before/scene.jpg" | awk '{print $1}')"
if [[ "$LOCAL_SESSION_META_HASH" != "$EXPECTED_SESSION_META_HASH" || "$LOCAL_RUN_META_HASH" != "$EXPECTED_RUN_META_HASH" || "$LOCAL_ENV_HASH" != "$EXPECTED_ENV_HASH" || "$LOCAL_MIDOBS_HASH" != "$EXPECTED_MIDOBS_HASH" || "$LOCAL_ENV_PNG_HASH" != "$EXPECTED_ENV_PNG_HASH" || "$LOCAL_SCENE_JPG_HASH" != "$EXPECTED_SCENE_JPG_HASH" ]]; then
  echo "Local scene_before changed after remote replan submission; refusing to collect stale remote result." >&2
  exit 1
fi

REMOTE_ITER_DIR="$REMOTE_SESSION_DIR/$RUN_NAME/iter_$(printf '%03d' "$ITERATION")"

SSH_BASE=(ssh)
if [[ -S "$SSH_SOCKET" ]]; then
  SSH_BASE+=(-S "$SSH_SOCKET")
fi

RSYNC_RSH="ssh"
if [[ -S "$SSH_SOCKET" ]]; then
  RSYNC_RSH="ssh -S $SSH_SOCKET"
fi

if [[ "$WAIT_FOR_JOB" == "1" && -n "$JOB_ID" ]]; then
  while true; do
    STATE="$("${SSH_BASE[@]}" "$REMOTE_HOST" "sacct -j '$JOB_ID' --format=State -n | head -1 | tr -d ' '")"
    if [[ -z "$STATE" ]]; then
      QUEUE_STATE="$("${SSH_BASE[@]}" "$REMOTE_HOST" "squeue -h -j '$JOB_ID' -o '%T' | head -1 | tr -d ' '")"
      if [[ -n "$QUEUE_STATE" ]]; then
        sleep 10
        continue
      fi
      break
    fi
    case "$STATE" in
      PENDING|CONFIGURING|RUNNING|COMPLETING|SUSPENDED|STAGE_OUT|RESIZING|SIGNALING)
        sleep 10
        ;;
      *)
        break
        ;;
    esac
  done
elif [[ "$WAIT_FOR_JOB" == "1" ]]; then
  echo "WAIT_FOR_JOB=1 requires a job id; none was provided and none was found in $REQUEST_PATH" >&2
  exit 1
fi

mapfile -t REMOTE_HASHES < <("${SSH_BASE[@]}" "$REMOTE_HOST" "sha256sum '$REMOTE_SESSION_DIR/session_meta.json' '$REMOTE_SESSION_DIR/$RUN_NAME/run_meta.json' '$REMOTE_ITER_DIR/scene_before/env.xml' '$REMOTE_ITER_DIR/scene_before/mid_obs.jsonl' '$REMOTE_ITER_DIR/scene_before/env.png' '$REMOTE_ITER_DIR/scene_before/scene.jpg' | awk '{print \$1}'")
REMOTE_SESSION_META_HASH="${REMOTE_HASHES[0]:-}"
REMOTE_RUN_META_HASH="${REMOTE_HASHES[1]:-}"
REMOTE_ENV_HASH="${REMOTE_HASHES[2]:-}"
REMOTE_MIDOBS_HASH="${REMOTE_HASHES[3]:-}"
REMOTE_ENV_PNG_HASH="${REMOTE_HASHES[4]:-}"
REMOTE_SCENE_JPG_HASH="${REMOTE_HASHES[5]:-}"
if [[ "$REMOTE_SESSION_META_HASH" != "$EXPECTED_SESSION_META_HASH" || "$REMOTE_RUN_META_HASH" != "$EXPECTED_RUN_META_HASH" || "$REMOTE_ENV_HASH" != "$EXPECTED_ENV_HASH" || "$REMOTE_MIDOBS_HASH" != "$EXPECTED_MIDOBS_HASH" || "$REMOTE_ENV_PNG_HASH" != "$EXPECTED_ENV_PNG_HASH" || "$REMOTE_SCENE_JPG_HASH" != "$EXPECTED_SCENE_JPG_HASH" ]]; then
  echo "Remote scene_before does not match the submitted local scene; refusing to collect." >&2
  exit 1
fi

mkdir -p "$LOCAL_ITER_DIR"
rsync -az -e "$RSYNC_RSH" \
  "$REMOTE_HOST:$REMOTE_ITER_DIR/status.json" \
  "$LOCAL_ITER_DIR/status.json"
rsync -az -e "$RSYNC_RSH" \
  "$REMOTE_HOST:$REMOTE_ITER_DIR/replan_candidate1_driver.log" \
  "$LOCAL_ITER_DIR/replan_candidate1_driver.log"
rsync -az --delete -e "$RSYNC_RSH" \
  "$REMOTE_HOST:$REMOTE_ITER_DIR/sim_candidates/" \
  "$LOCAL_ITER_DIR/sim_candidates/"

if "$PY_LOCAL" - <<'PY' "$LOCAL_ITER_DIR/status.json"
from pathlib import Path
import json, sys
path=Path(sys.argv[1])
data=json.loads(path.read_text())
raise SystemExit(0 if data.get("state") == "planned" else 1)
PY
then
  "$PY_LOCAL" "$REPO_ROOT/robot_control/scripts/closed_loop_session.py" prepare-real-push \
    --session-dir "$LOCAL_SESSION_DIR" \
    --run "$RUN_NAME" \
    --iteration "$ITERATION"
fi
