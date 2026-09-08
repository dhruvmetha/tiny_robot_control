#!/bin/bash
#SBATCH --job-name=nav_realstack_pack
#SBATCH --partition=main
#SBATCH --account=general
#SBATCH --qos=normal
#SBATCH --cpus-per-task=4
#SBATCH --mem=4G
#SBATCH --time=00:15:00

set -euo pipefail

OUT=/scratch/tdn39/real_exp/results/twohop_v3_multiobject/navigation_realstack_all100_margin_0p1cm_v1
METADATA_ARCHIVE=navigation_realstack_metadata.tar.gz
TRAJECTORY_ARCHIVE=navigation_realstack_trajectories.tar.gz

cd "$OUT"
test ! -e "$METADATA_ARCHIVE"
test ! -e "$TRAJECTORY_ARCHIVE"

tar \
  --exclude='scenes/*/*/nav_baseline_*/trajectory.jsonl' \
  -czf "$METADATA_ARCHIVE" \
  scenes jobs logs summary.json navigation_realstack.csv \
  both_navigation_baselines_failed.csv run_config.json scene_manifest.txt \
  effective_wavefront_inflation.yaml

find scenes -type f -name trajectory.jsonl -print0 \
  | sort -z \
  | tar --null --files-from=- -czf "$TRAJECTORY_ARCHIVE"

sha256sum "$METADATA_ARCHIVE" "$TRAJECTORY_ARCHIVE" \
  > portable_archives.sha256
ls -lh "$METADATA_ARCHIVE" "$TRAJECTORY_ARCHIVE" portable_archives.sha256
