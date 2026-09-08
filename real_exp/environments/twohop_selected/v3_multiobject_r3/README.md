# Multi-object two-hop real scenes (r3)

These are the two geometry-diverse scenes retained from the September 2 discovery run. Both passed bounded model-guided Full NAMO with replay hop trace `[2, 1, 0]` and cross-gate mechanical-independence checks.

The hard scene is exhaustively certified as 1-push K1 plus 1-push K2. The medium scene is exhaustively certified as 2-push K1; its bounded Full NAMO/replay used two pushes at K2, but the separate exhaustive K2 sweep reached its 900-second cap, so the exact K2 minimum remains unconfirmed.

## Physical inventory

Each scene uses `wall_9`, `wall_10`, `wall_11`, `wall_12`, `obj_1`, the existing `obj_4`, and a same-size clone called `obj_4b`. Generated sheets call the existing object `obj_4a`; `scripts/check_build.py` maps that role back to the camera/config name `obj_4`.

`obj_4b` still needs its own ArUco tag and an `obj_4b` entry in `config/objects.yaml` before a real run. Its dimensions are the same as `obj_4` (12.0 x 7.5 x 5.0 cm).

## Live setup commands on dhruv-linux

From `/home/dhruv/projects_dhruv/namo/robot_control`:

```bash
PYTHONPATH=src /home/dhruv/miniconda3/envs/namo312/bin/python scripts/check_build.py \
  --sheet real_exp/environments/twohop_selected/v3_multiobject_r3/hard1-hard1/twohop_multi_00015/build_sheet.json \
  --build-id twohop_multi_00015 --gui --auto 5
```

The hard scene's goal flag is `--goal 8.0 64.0`.

```bash
PYTHONPATH=src /home/dhruv/miniconda3/envs/namo312/bin/python scripts/check_build.py \
  --sheet real_exp/environments/twohop_selected/v3_multiobject_r3/med2-med2/twohop_multi_00071/build_sheet.json \
  --build-id twohop_multi_00071 --gui --auto 5
```

The medium scene's goal flag is `--goal 41.0 64.0`.

The `audit/` directory contains the exact screening records and selection reports. The `renders/` directory contains labeled overhead previews. `selected_manifest.dhruv-linux.txt` contains absolute XML paths for both scenes.
