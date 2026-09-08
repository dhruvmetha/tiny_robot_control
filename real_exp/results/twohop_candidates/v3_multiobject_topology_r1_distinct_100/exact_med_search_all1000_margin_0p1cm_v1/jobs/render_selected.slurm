#!/bin/bash
#SBATCH --partition=main
#SBATCH --cpus-per-task=1
#SBATCH --mem=4G
#SBATCH --time=00:15:00

set -euo pipefail

: "${RUN_ENV:?set RUN_ENV to the frozen experiment environment}"
: "${SCENE_MANIFEST:?set SCENE_MANIFEST to one scene directory per line}"
: "${OUTPUT_DIR:?set OUTPUT_DIR for the rendered PNGs}"

set -a
source "$RUN_ENV"
set +a
source "$NAMO_ACTIVATE"
: "${NAMO_BINDINGS:?set NAMO_BINDINGS to the compatible compiled binding directory}"
export PYTHONPATH="$NAMO_BINDINGS:$PYTHONPATH"

mkdir -p "$OUTPUT_DIR"
while IFS= read -r scene_dir; do
    [ -n "$scene_dir" ] || continue
    scene_id=$(basename "$scene_dir")
    scene_out="$OUTPUT_DIR/$scene_id"
    mkdir -p "$scene_out"
    "$NAMO_PYTHON" -u scripts/render_env_with_regions.py \
        "$scene_dir/env.xml" \
        --namo-config "$NAMO_CONFIG" \
        --out-dir "$scene_out"
done < "$SCENE_MANIFEST"
