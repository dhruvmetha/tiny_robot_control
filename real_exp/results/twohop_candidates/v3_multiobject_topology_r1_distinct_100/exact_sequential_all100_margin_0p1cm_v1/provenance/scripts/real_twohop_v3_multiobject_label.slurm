#!/usr/bin/env bash
#SBATCH --partition=main
#SBATCH --cpus-per-task=1
#SBATCH --mem=8G
#SBATCH --time=01:00:00
#SBATCH --job-name=namo-2hop-label
#SBATCH --output=logs/real-twohop-v3-label-%A_%a.out

set -euo pipefail

: "${RUN_ENV:?set RUN_ENV to the untracked run environment}"
: "${SLURM_ARRAY_TASK_ID:?submit this entrypoint as a Slurm array}"
: "${LABEL_PROFILE:?set LABEL_PROFILE to hard1-hard1 or med2-med2}"
: "${LABEL_OUT_BASE:?set LABEL_OUT_BASE to the stage output root}"
source "$RUN_ENV"
source "$NAMO_ACTIVATE"
: "${NAMO_BINDINGS:?set NAMO_BINDINGS to the compatible compiled binding directory}"
export PYTHONPATH="$NAMO_BINDINGS:$PYTHONPATH"
if [[ "${NAMO_STAGE_SCIPY_LOCAL:-0}" == "1" ]]; then
  source "$NAMO_REPO/scripts/amarel/stage_scipy_local.sh"
  stage_scipy_local
  python -c 'from scipy.integrate._rules import ProductNestedFixed; import scipy.integrate._ivp.lsoda; import lightning.pytorch.loops.optimization.automatic; import torchmetrics.image.d_lambda; import torchmetrics.functional.text.edit'
fi

case "$LABEL_PROFILE:$LABEL_STAGE" in
  hard1-hard1:k1)
    protected=(--protect-object obstacle_1_movable --protect-object obstacle_2_movable)
    ;;
  hard1-hard1:k2)
    protected=(--protect-object obstacle_0_movable)
    ;;
  med2-med2:k1)
    protected=(--protect-object obstacle_1_movable)
    ;;
  med2-med2:k2)
    protected=(--protect-object obstacle_0_movable --protect-object obstacle_2_movable)
    ;;
  *)
    echo "unsupported label contract LABEL_PROFILE=$LABEL_PROFILE LABEL_STAGE=$LABEL_STAGE" >&2
    exit 2
    ;;
esac

task_shard=$(printf "task_%04d" "$SLURM_ARRAY_TASK_ID")
LABEL_OUT="$LABEL_OUT_BASE/shards/$task_shard"
label_xml=$(sed -n "$((SLURM_ARRAY_TASK_ID + 1))p" "$LABEL_MANIFEST")
if [[ -z "$label_xml" || ! -f "$label_xml" ]]; then
  echo "array index $SLURM_ARRAY_TASK_ID has no existing XML in $LABEL_MANIFEST" >&2
  exit 2
fi
if [[ "$LABEL_STAGE" == "k2" \
      && "$label_xml" != */k1/xmls/*.xml \
      && "$label_xml" != */k1/shards/task_*/xmls/*.xml ]]; then
  echo "K2 labels must consume the post-K1 XML manifest, not the original scenes" >&2
  exit 2
fi

python scripts/pipeline/materialize_keyhole2.py \
  --manifest "$LABEL_MANIFEST" \
  --out-root "$LABEL_OUT" \
  --start "$SLURM_ARRAY_TASK_ID" \
  --end "$((SLURM_ARRAY_TASK_ID + 1))" \
  --workers 1 \
  --kh1-chain-depth "$CHAIN_DEPTH" \
  --scene-timeout "$SCENE_TIMEOUT" \
  "${protected[@]}"
