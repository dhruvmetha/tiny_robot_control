#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="${REPO_ROOT:-/home/dhruv/projects_dhruv/namo}"
REMOTE_HOST="${REMOTE_HOST:-tdn39@amarel.rutgers.edu}"
REMOTE_ROOT="${REMOTE_ROOT:-/scratch/tdn39/namo_less_depth}"
SSH_SOCKET="${SSH_SOCKET:-/tmp/amarel-cm.sock}"

SSH_BASE=(ssh)
if [[ -S "$SSH_SOCKET" ]]; then
  SSH_BASE+=(-S "$SSH_SOCKET")
fi

RSYNC_RSH="ssh"
if [[ -S "$SSH_SOCKET" ]]; then
  RSYNC_RSH="ssh -S $SSH_SOCKET"
fi

"${SSH_BASE[@]}" "$REMOTE_HOST" "mkdir -p '$REMOTE_ROOT/robot_control' '$REMOTE_ROOT/namo_cpp'"

rsync -az --delete \
  --exclude='__pycache__/' \
  --exclude='*.pyc' \
  --exclude='.git/' \
  --exclude='.pytest_cache/' \
  -e "$RSYNC_RSH" \
  "$REPO_ROOT/robot_control/src" \
  "$REPO_ROOT/robot_control/scripts" \
  "$REPO_ROOT/robot_control/config" \
  "$REPO_ROOT/robot_control/real_test_envs" \
  "$REPO_ROOT/robot_control/closed_loop_sessions_less_depth" \
  "$REMOTE_HOST:$REMOTE_ROOT/robot_control/"

rsync -az --delete \
  --exclude='__pycache__/' \
  --exclude='*.pyc' \
  --exclude='.git/' \
  --exclude='.pytest_cache/' \
  --exclude='build/' \
  --exclude='build_python/' \
  -e "$RSYNC_RSH" \
  "$REPO_ROOT/namo_cpp/" \
  "$REMOTE_HOST:$REMOTE_ROOT/namo_cpp/"

echo "Synced to $REMOTE_HOST:$REMOTE_ROOT"
