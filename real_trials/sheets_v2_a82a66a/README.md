# Real-table build sheets, v2

593 scenes, 100 per tier per horizon except `hmax2/hard`, which ships 93 because that is every scene in the pool that qualifies.

Same CSV schema as the v1 sheets one directory up, which the hardware side validated 600 out of 600 against. Nothing about how a row is read has changed. The scenes themselves are different and the difficulty numbers are on a different footing, so **v1 and v2 build ids do not refer to the same scenes**. `easy_007` in v2 is not `easy_007` in v1.

## What changed

v1 tiers came from the search sweep, which stops expanding a push once it jams and leaves holes: of 44278 first pushes across the original 478 scenes, 1965 clean pushes never had a single second push tried on them, so roughly 162k second pushes were never simulated. A scene's solve rate was measured against whatever the search happened to touch.

v2 tiers come from pure enumeration at depth 2. Every reachable push at the start state gets executed, and every one that neither opens the goal nor jams gets every reachable second push tried until one opens or all of them fail. So each first push ends in one of four states, and "dead" means genuinely exhausted rather than abandoned. 1478 scenes, 170095 first pushes, 2.4M simulations.

The tiers move a lot. On the original 478 the median hmax=2 solve rate went 0.311 to 0.704, and 39 scenes labelled hard turned out to be easy. Cuts are unchanged: hard below 0.05, medium below 0.30, easy at or above, over every enumerated push.

## What a tier means, in one line each

`1push` counts single pushes that open the goal region. `hmax2` counts those plus setups that open it on the second push. A scene is far easier on the hmax2 axis than the 1push axis, because a setup rescues most of what one push cannot do: of 376 scenes that are medium on one push, 372 are easy at depth 2.

`push_kind` says which the scene needs. `one_push` means some single push works. `needs_2_chain` means none does and the robot has to set up first.

## post_push_clearance.csv, read this before picking a build order

The generator only ever checks that the route is roomy with the movable object DELETED. Nothing checks the gap that exists after the push, with the block sitting wherever it ended up. Success only requires 20% of the goal region's sampled points to become reachable, not for the block to leave the doorway, so a scene can pass the generator's margin test and still finish in a squeeze.

`best_corridor_cm` is the width of the tightest point on the roomiest working solution. `worst_corridor_cm` is the same for the least roomy one measured. The normal wavefront rule needs 8.0 cm to call a gap passable and the generator's margin test asks for 11.0.

98 of the 593 have a best corridor under 11 cm, and they are concentrated where you would expect: 48 in `1push/hard` and 23 in `hmax2/hard` against 2 and 4 in the easy tiers. Roughly 69 scenes across the full 2226-card pool have at least one *working* push that leaves under 8 cm, which matters because the search may return one of those.

**This is a measurement, not a filter, and it is deliberately not used to drop anything.** The threshold it would be compared against comes from `compute_rotation_safe_robot_radius_m`, whose name says diagonal and whose body returns `max(hx, hy)` = 3.5 cm. Believing that name once cost a whole scene pool. Cutting scenes on a number that model may have wrong would throw away exactly the scenes where sim and hardware are most likely to disagree, which are the informative ones.

Suggested order: build the roomy scenes first so a failure is about the planner, then run a handful of the tightest on purpose to find out what the real threshold is. That turns the risk into a calibration.

Two honest caveats on the number. It saturates at a 30 cm search ceiling on about two thirds of scenes, which just means those routes are wide open and it stops distinguishing among them. And it is a uniform-inflation bottleneck on a 2 mm grid, measured from both the original robot start and the robot's post-push pose with the roomier kept, because each anchor alone is biased: the post-push pose reads its own contact clearance, and the original start reads as cut off when the push leaves the robot on the far side.

## Known gaps

The goal MARKER and the goal REGION are not the same target. Success is measured on the region, and on the v1 600, 178 scenes had a labelled push that cleared the region bar while leaving the marker itself unreachable. If a hardware run counts as won only when the robot reaches the marker, those scenes need their markers moved onto reachable points inside the region. `marker_reachable.csv` one directory up records which v1 scenes have the problem; the equivalent has not been computed for v2.

About 1.8% of scenes carry label noise where the simulator disagrees with its own recorded verdict on the same push, same config, same sequence. The cause is unidentified. It is concentrated in two-push chains.
