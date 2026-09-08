# Exact sequential K1/K2 audit at 0.1 cm

This directory is the authoritative exact-label audit for the 100 distinct
multi-object two-hop candidates in the parent bundle. K1 was measured on each
original XML and K2 was measured on the exact XML materialized after the valid
K1 opening. The discovery directory names are retained for traceability; they
are not measured paper labels.

## Result

- 100 inputs: 50 from each discovery profile.
- 58 complete sequential K1/K2 pairs; all 58 passed the full gate-member
  mechanical-independence check at 2 mm / 1 degree.
- 42 incomplete pairs because a required transition or uncensored gate
  classification was unavailable.
- 0 exact `hard1-hard1` pairs.
- 0 exact `med2-med2` pairs.

The complete row-level join is `all100_exact_navigation_inventory.csv`; the
aggregate is `all100_exact_navigation_summary.json`. `run_metadata.json` fixes
the code, binding, configuration, inputs, job IDs, thresholds, and hashes used
for this result.

## Four usable alternatives

`recommended_2push2push_both_nav_failed/scenes/` contains four mechanically
independent scenes that require a two-push chain at both gates and for which
both real-stack pure-navigation baselines failed. Each scene directory has:

- `env.xml`: original two-hop state.
- `post_k1.xml`: state after the canonical K1 opening.
- `post_k2.xml`: state after the canonical K2 opening.
- `build_sheet.json` and `screen.json`: generation and screening metadata.
- `env_regions.png`: visual overview.

`recommended_2push2push_both_nav_failed/candidates.csv` contains their four
complete inventory rows, and `manifest.relative.txt` lists their portable
original-XML paths.

The candidates are `twohop_multi_00284`, `twohop_multi_00385`,
`twohop_multi_00711`, and `twohop_multi_00933`. They are inspection candidates,
not substitutes for the requested exact paper-label cells.

## Layout

- `hard1-hard1/` and `med2-med2/`: merged stage rows, per-task materialized
  XMLs, reducer validations, and empty exact-selector outputs.
- `recommended_2push2push_both_nav_failed/`: portable four-scene inspection
  package and renders.
- `provenance/`: exact configuration, run environment, input manifests,
  labeling/selection scripts, and compiled-binding build record.
- `tools/`: audited reducer and inventory builder used to validate and join the
  results.

Only the tier-1 base margin changed to `0.001 m`. Navigation contribution
(`0`), push-approach margin (`0.003 m`), XML minimum collision margin
(`0.005 m`), and table inflation (`0.08 m`) remained unchanged.
