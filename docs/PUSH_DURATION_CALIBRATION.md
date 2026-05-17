# Push Duration Calibration (`push_steps`)

How to tune `push_steps` so a real-robot push moves the object by the same amount as a sim push.

---

## The problem in one sentence

The planner's digital twin assumes a push primitive lasts **2.5 seconds** (one "NAMO unit"). The real robot's per-unit duration is set by `controller.yaml: push.push_steps` (default 75 ticks at 30 Hz = 2.5 sec). If real-world friction, wheel slip, or motor torque doesn't match sim, the same 2.5-second push moves the object a different distance — and every plan the system makes is based on the sim's prediction, so the gap accumulates.

We want to find the value of `push_steps` where:

```
real Δobject_position  ≈  sim Δobject_position
```

for the same primitive.

---

## What data we need

### From sim (do this once per primitive)

For a single push primitive `(object_id, edge_idx, depth=0)`:

| Field | Why we need it |
|---|---|
| Object pose **before** push: `(x, y, θ)` | Reference start |
| Object pose **after** push: `(x, y, θ)` | Compute `Δ_sim = pose_after − pose_before` |
| Object dimensions: `(width, depth)` | To replicate the same object on real |
| Robot pose at push start: `(x, y, θ)` | To reproduce the contact geometry on real |
| Edge index | Identifies which face is being pushed |

**Sim is deterministic** given the same start state — you only need to run it once per primitive.

### From real (do this N times per `push_steps` value)

For each trial:

| Field | Why we need it |
|---|---|
| Object pose **before** push: `(x, y, θ)` from camera | Reference start (try to match sim's start) |
| Object pose **after** push: `(x, y, θ)` from camera | Compute `Δ_real_i = pose_after − pose_before` |
| The `push_steps` value used | The independent variable we're tuning |
| Trial number `i` | For variance estimation |

Real-world is noisy — you need **N ≥ 5 trials** per `push_steps` value to estimate the mean and standard deviation.

---

## The metric we're minimizing

Per trial:

```
error_i  =  ‖ Δ_real_i  −  Δ_sim ‖     (Euclidean distance in cm)
```

Optionally include rotation:

```
error_i  =  ‖ Δ_real_xy_i − Δ_sim_xy ‖   +   w · |Δ_real_θ_i − Δ_sim_θ|
```

with `w ≈ 0.5 cm/degree` to make units commensurate.

Over N trials at a given `push_steps`:

```
mean_error(push_steps)  =  (1/N) · Σ error_i
std_error(push_steps)   =  std(error_i)
```

We want **mean_error as small as possible** while **std_error stays under some bound** (otherwise we're fitting noise).

---

## The procedure (single scalar parameter sweep)

Because `push_steps` is one scalar on a known range, we don't need fancy optimization — just a sweep:

```
sweep_values = [50, 60, 75, 90, 105, 120]   # 6 settings

For each push_steps in sweep_values:
    Set controller.yaml: push.push_steps = push_steps
    For trial i = 1..N (N ≥ 5):
        1. Manually place object at the canonical start pose (±0.5cm, ±2°)
        2. Verify start pose from camera
        3. Trigger ONE push: PushSubgoal(object_id, edge_idx, push_steps=1)
        4. Wait for executor to finish (PUSH + RETREAT)
        5. Record final object pose from camera
        6. Compute error_i = ‖Δ_real_i - Δ_sim‖
    Record mean_error and std_error for this push_steps
```

After the sweep:

```
optimal_push_steps  =  argmin_{push_steps in sweep_values}  mean_error(push_steps)
```

If the sweep grid is coarse and the minimum looks promising at, e.g., `push_steps=90`, do a **finer sweep** around it: `[82, 86, 90, 94, 98]`. Usually 2 rounds of bisection is enough.

---

## What the result looks like

You'll end up with a small table:

| `push_steps` | Mean error (cm) | Std (cm) | N trials |
|---|---|---|---|
| 50 | 6.2 | 0.8 | 5 |
| 60 | 4.5 | 0.6 | 5 |
| 75 | 2.1 | 0.7 | 5 |
| 90 | **0.9** | 0.5 | 5 |
| 105 | 1.4 | 0.9 | 5 |
| 120 | 3.2 | 1.1 | 5 |

In this example, the optimum is **90**. (Real robot needs longer push than sim — common because of friction/slip on the floor.)

Plot it: it should be roughly a U-curve with the minimum at the optimal value.

---

## Things that *must* stay constant during calibration

If any of these change, your results become noise:

- **The object** — same physical piece, same weight, same friction patches
- **The surface** — same floor area, same cleanliness
- **The robot's battery state** — recharge between sessions; voltage drop changes wheel torque
- **`PushConfig.max_speed`** — leave it at 0.3 (or whatever; just don't change mid-sweep)
- **`PushConfig.dynamic_direction`** — leave it `True`
- **The edge index** — calibrate on one edge first (e.g. `edge_idx=0`)
- **The starting pose** — within ±0.5 cm and ±2°. Re-verify from camera between trials

---

## Sources of variance to expect

Even with a perfectly controlled setup, expect:

| Source | Typical magnitude |
|---|---|
| ArUco pose jitter (use 30-frame median) | ±0.5 cm, ±1° |
| Initial pose placement by hand | ±0.5 cm, ±2° |
| Wheel slip on contact | 1–3 cm of variance over 5-second push |
| Surface friction variation (day to day) | up to 20% of total displacement |
| Motor torque drift (battery) | grows over a session |

Realistically, **a "good" minimum mean error is ~10% of the sim displacement**, not 0. Don't chase the last percent.

---

## What ONE primitive captures, and what it doesn't

Calibrating on one `(object_id, edge_idx, depth=0)` gives you a value of `push_steps` that's correct for **that single primitive**.

**Concerns:**

1. **Different objects respond differently.** A 5×5 cube and a 15×7 rectangle have different inertia and contact patches. If you can, pick the **object you push most often** as the calibration target.

2. **Different edges may differ.** Pushing along the long axis vs the short axis of a rectangle differs. Pick a representative edge (or average across 2–3 edges).

3. **Depth=0 vs depth=N.** A 5-unit push doesn't necessarily move the object 5× as far as a 1-unit push — slippage accumulates. The `push_steps` you find for depth=0 is *approximately* correct for higher depths, but not exactly. Accept this for now; the planner re-plans every iteration in MPC mode, so per-primitive accuracy matters more than per-multi-primitive accuracy.

---

## Optional: rotation matters too

The metric above mixes translation and rotation, but you can treat them separately:

```
translation_error  =  ‖ Δ_real_xy − Δ_sim_xy ‖   (cm)
rotation_error     =  | Δ_real_θ − Δ_sim_θ |     (degrees)
```

If real pushes consistently rotate the object more than sim, that's a different bug — likely a contact-geometry problem (the robot isn't centered on the face), not a duration problem. `push_steps` won't fix it.

---

## Quick implementation recipe

To execute this calibration, you need:

1. **A sim ground-truth value** — one-off script that calls `RLEnvironment.step()` once and dumps `Δ_sim`.
2. **A real-robot trial runner** — script that:
   - Reads target push from CLI
   - Issues `PushSubgoal(object_id, edge_idx, push_steps=1)` to the executor
   - Captures before/after object pose via the camera
   - Logs to CSV: `trial, push_steps, before_x, before_y, before_θ, after_x, after_y, after_θ`
3. **A sweep wrapper** — bash loop over `push_steps` values that edits `controller.yaml`, runs the trial runner N times, collects results.
4. **A simple analysis script** — pandas/CSV → compute mean and std of error per `push_steps`, plot, pick argmin.

None of these exist yet — they're small scripts to write when calibration is the active task.

---

## What if calibration doesn't help?

If your best `push_steps` still has mean error > 30% of sim displacement, the gap probably isn't a duration problem. Likely culprits:

- **Robot losing contact** during the push (no contact detection — see [PUSH state machine](./PUSH_GEOMETRY.md))
- **Push axis misaligned** (object rotates instead of translating)
- **Object pose perception is off** (ArUco offset wrong)
- **Wheel slip rate is too high** (no amount of duration tuning fixes a slipping wheel)

Don't keep tuning `push_steps` past the point of diminishing returns — diagnose the root cause instead.

---

## Summary

| Question | Answer |
|---|---|
| What are we tuning? | `controller.yaml: push.push_steps` |
| Why? | Match real Δobject_position to sim Δobject_position |
| What data from sim? | One per primitive: before/after object pose |
| What data from real? | N≥5 trials per `push_steps`: before/after object pose |
| Metric? | `mean ‖Δ_real - Δ_sim‖` over trials |
| Procedure? | Coarse sweep, then bisect around the minimum |
| When to stop? | When mean error is ~10% of sim displacement or std becomes dominant |
| Limitations? | Per-object, per-edge, mostly per-depth=0; chains accumulate residual |
