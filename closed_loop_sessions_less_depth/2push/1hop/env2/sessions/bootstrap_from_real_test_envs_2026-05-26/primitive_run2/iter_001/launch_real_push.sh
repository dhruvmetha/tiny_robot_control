#!/usr/bin/env bash
set -euo pipefail
PYTHON_BIN="${PYTHON_BIN:-python}"
cd '/home/dhruv/projects_dhruv/namo/robot_control'
"$PYTHON_BIN" '/home/dhruv/projects_dhruv/namo/robot_control/scripts/execute_real_push.py' \
  --config '/home/dhruv/projects_dhruv/namo/robot_control/config/real.yaml' \
  --camera-service tcp://localhost:5556 \
  --trial-spec '/home/dhruv/projects_dhruv/namo/robot_control/closed_loop_sessions_less_depth/2push/1hop/env2/sessions/bootstrap_from_real_test_envs_2026-05-26/primitive_run2/iter_001/selected_trial_spec.yaml' \
  --diag-path '/home/dhruv/projects_dhruv/namo/robot_control/closed_loop_sessions_less_depth/2push/1hop/env2/sessions/bootstrap_from_real_test_envs_2026-05-26/primitive_run2/iter_001' \
  --run-name real_push \
  --headless \
  --capture-scene \
  --record-video \
  --nav-speed 0.4 \
  --push-speed 0.4 \
  --no-reset-check \
  --allow-overwrite
