# Distinct multi-object two-hop candidates (topology r1)

This bundle contains 70 model-screened, physically distinct candidate scenes:

- `hard1-hard1/`: 40 candidates selected from 135 bounded-screen passes.
- `med2-med2/`: 30 candidates selected from 224 bounded-screen passes.
- `showcase_10_per_profile.relative.txt`: the first 10 candidates from each profile, with a larger pairwise diversity margin and PNGs under `renders/`.
- `manifest.relative.txt`: all 70 portable scene paths.
- `audit/`: immutable promotion manifests, rejection counts, effective thresholds, and selection metadata.

The source screen covered all 2,001 topology-valid generated scenes (1,061 hard-profile and 940 medium-profile). Every terminal record has a nonzero simulator-call count and the pinned scorer SHA-256. Every pass replayed the ordered region trace `2 -> 1 -> 0`.

The 70 selected scenes passed 2,415 all-pairs comparisons after normalizing global translation and left/right reflection. Both gates, wall assignment, opener geometry, auxiliary-object geometry, dimensions, and topology are compared. Donor IDs, gate spacing, and task-anchor movement do not create diversity. No selected scene is a near duplicate of the two previously retained multi-object scenes. The selection thresholds are 5 cm / 15 degrees for walls, 5 cm / 20 degrees for auxiliary objects, and 0.5 cm for dimensions.

Important: `hard1-hard1` and `med2-med2` are discovery/screening profiles, not final paper difficulty labels. Exact sequential K1 then K2 labeling and real-robot replay are still required before a candidate is called hard or medium in the paper. The renders are for human inspection; the C++ simulation/replay evidence is authoritative.

Code revision: `823576a` on `feat/real-twohop-multiobject-diversity`.
