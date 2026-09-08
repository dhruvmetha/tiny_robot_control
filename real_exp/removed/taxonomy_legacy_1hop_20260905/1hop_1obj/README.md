# One-hop, one-object real-robot environments

This is the authoritative register of the one-hop, one-movable-object scenes
found for the real-robot study as of 2026-09-05. There are currently exactly
two. Use the complete `uid` below in notes, result manifests, and paper data;
the short build IDs are reused across the `1push` and `hmax2` sheets.

Both cards contain exactly one movable object and only two labeled free
regions, `robot` and `goal`. They are therefore `1hop_1obj` scenes. The
`hmax2` axis on `easy_020` means that the action solution may require a chain
of two pushes on that one object. It does not mean that the region graph has
two hops.

| UID | Source XML | Collected NAMO results | Real navigation baseline | Dynamic simulation navigation |
| --- | --- | --- | --- | --- |
| `hmax2__easy_020__5f0639e3` | `real_buildable/pool1/med/rb_00217/env.xml` | HY5U policy 5/5; model search 5/5; uniform search 5/5 | `ignore` failed; `penalise` failed; qualifies | Not tested |
| `1push__hard_021__bf3e3cdf` | `real_buildable/pool2/med_304/rb_00042/env.xml` | model-prior search 5/5; uniform-prior search 5/5 | `ignore` failed; `penalise` failed; qualifies | Not tested |

The `hard_021` model runs used best-first model-prior search, not the newer
zero-rollout HY5U policy arm. Do not report them as policy trials.

`environments.json` contains the full source paths, hashes, build metadata,
result locations, and navigation evidence. `cards/` preserves the exact
gallery exports named by these UIDs.
