# Push Controller Geometry

This document explains the geometric concepts used in the push controller, particularly edge point generation and push line definitions.

## Coordinate System

- **Origin**: Bottom-left corner of workspace
- **X-axis**: Positive rightward (depth direction when theta=0)
- **Y-axis**: Positive upward (width direction when theta=0)
- **Theta**: Counter-clockwise from +X axis (degrees)
- **Units**: Centimeters (cm)

## Object Dimensions

```
Dimension convention (matching GUI/micromvp):
    - depth = X dimension (along marker heading when theta=0)
    - width = Y dimension (perpendicular to marker heading)

For a 12cm x 4cm object defined as width=12, depth=4:
    - When theta=0: 4cm along X (depth), 12cm along Y (width)
```

## Edge Point Generation

Edge points are robot approach positions for pushing. They are generated around the object perimeter at a `standoff` distance from each face.

### Index Layout

For `points_per_face=n`, there are `4*n` total edge points:

```
Indices 0 to 2n-1:     Top/Bottom pairs (sample along X)
Indices 2n to 4n-1:    Right/Left pairs (sample along Y)
```

**Example for points_per_face=3 (12 total points):**

| Index | Face       | Sample | Push Direction |
|-------|------------|--------|----------------|
| 0     | Top (+Y)   | 0      | toward -Y      |
| 1     | Bottom (-Y)| 0      | toward +Y      |
| 2     | Top (+Y)   | 1      | toward -Y      |
| 3     | Bottom (-Y)| 1      | toward +Y      |
| 4     | Top (+Y)   | 2      | toward -Y      |
| 5     | Bottom (-Y)| 2      | toward +Y      |
| 6     | Right (+X) | 0      | toward -X      |
| 7     | Left (-X)  | 0      | toward +X      |
| 8     | Right (+X) | 1      | toward -X      |
| 9     | Left (-X)  | 1      | toward +X      |
| 10    | Right (+X) | 2      | toward -X      |
| 11    | Left (-X)  | 2      | toward +X      |

### Consecutive Pairing

Consecutive indices (0-1, 2-3, 4-5, ...) form **pairs** that share a common mid-point:
- Even index = one face (Top or Right)
- Odd index = opposite face (Bottom or Left)

## Mid-Point Calculation

The mid-point is the **average** of the edge point and its mate (opposite face):

```python
mate = i + 1 if i % 2 == 0 else i - 1
mid_point = (
    0.5 * (edge_points[i] + edge_points[mate])
)
```

### Critical Insight: Mid-Point is NOT Object Center

For Top/Bottom pairs at sample index j:

```python
u = sample_lin(-hd, hd, n, j)  # X position along face

Top edge:    (u, +hw + standoff)
Bottom edge: (u, -hw - standoff)
mid_point:   (u, 0)  # NOT (0, 0) unless j is center sample
```

The mid-point lies on the object's **center line** (Y=0 in local coords), but at X position `u` which varies by sample.

## Target Pushing Line

Each edge_idx defines a unique **target pushing line** - the line along which the robot pushes the object.

### Definition

The target pushing line connects:
1. **edge_pt** (robot approach position)
2. **mid_pt** (on object center line)
3. **mate_edge_pt** (opposite face)

This line is **perpendicular to the face** being pushed.

### Visualization (Local Coordinates, theta=0)

```
        For points_per_face=3:

        j=0: u=-hd        j=1: u=0         j=2: u=+hd
            │                 │                 │
            ●                 ●                 ●  ← edge_pts (Top face)
            │                 │                 │
   ┌────────┼─────────────────┼─────────────────┼────────┐
   │        │                 │                 │        │
   │        ● mid_pt          ● mid_pt          ● mid_pt │
   │     (-hd,0)            (0,0)            (+hd,0)     │
   │        │            (obj center)           │        │
   └────────┼─────────────────┼─────────────────┼────────┘
            │                 │                 │
            ●                 ●                 ●  ← edge_pts (Bottom face)
            │                 │                 │
            │                 │                 │
         push line         push line         push line
         (X = -hd)         (X = 0)           (X = +hd)
```

### Key Properties

1. **Each edge_idx has its own push line** - parallel lines at different positions
2. **Only the center sample (j=n/2) passes through object center**
3. **Push lines are perpendicular to the face** (vertical for Top/Bottom, horizontal for Right/Left)
4. **Consecutive pairs share the same push line** (indices 0-1, 2-3, etc.)

## The Full Push Line

The push line extends infinitely in both directions through mid_pt:

```
                APPROACH SIDE (t > 0)
                Robot approaches from here
                        │
                        │  ← search for free positions here
                        │
                        ● potential approach (t = standoff + hw + extra)
                        │
                        ● edge_pt (t = standoff + hw)
                        │
           ┌────────────┼────────────┐
           │            │            │
           │         mid_pt          │   OBJECT
           │         (t = 0)         │
           │            │            │
           └────────────┼────────────┘
                        │
                        ● mate_edge_pt (t = -(standoff + hw))
                        │
                        │
                        ● extended_pt (t = -50cm)
                        │
                        │
                PUSH TARGET SIDE (t < 0)
                Object moves this direction
```

### Line Parameterization

```python
# The push line parameterized from mid_pt
direction = normalize(edge_pt - mid_pt)  # Unit vector toward approach side

point(t) = mid_pt + t * direction

# Where:
#   t > 0  → approach side (toward/beyond edge_pt)
#   t < 0  → push side (toward extended_pt, where object moves)
#   t = 0  → mid_pt (inside object)
```

## Approach Position Validation Algorithm

When validating/finding an approach position, we search **along the push line** on the **approach side only**.

### Algorithm

```
1. FIRST: Check original edge_pt
   └── Query wavefront: is_free(edge_pt)?
   └── If FREE → return edge_pt (done, this is the ideal position)
   └── If BLOCKED → continue to step 2

2. SAMPLE along push line (approach side only):
   - Start from edge_pt
   - Step AWAY from object (increasing t)
   - Granularity = wavefront resolution (e.g., 0.5cm matching ~5mm wavefront)
   - Max search distance (e.g., 15cm)

3. For each sample point:
   └── Query wavefront: is_free(sample_pt)?
   └── If FREE → return this point (first free = nearest valid to object)
   └── If BLOCKED → continue to next sample

4. If no free position found within max distance → FAIL (approach not possible)
```

### Visual Example

```
        Sample:   5th      4th      3rd      2nd      1st
        Offset: +2.0cm   +1.5cm   +1.0cm   +0.5cm    0
                  │        │        │        │        │
                  ●────────●────────●────────●────────●────────┼────────
                  │        │        │        │        │        │
               BLOCKED  BLOCKED    FREE   BLOCKED  BLOCKED   Object
                                    ▲
                                    │
                              RETURN THIS
                         (first free = nearest to object)

        Granularity: 0.5cm (matching wavefront resolution)
        Max search: 15cm from edge_pt
```

### Why This Approach?

**Check edge_pt first:**
- edge_pt is the ideal position (correct standoff distance)
- Most of the time it will be free
- Avoids unnecessary sampling

**Sample with wavefront granularity:**
- Matches the resolution of collision checking
- No point sampling finer than what wavefront can resolve
- Efficient and consistent

**Return first free (nearest to object):**
- Minimizes travel distance during push phase
- Robot is as close as safely possible
- Maintains intended standoff concept

### Why NOT Search in Arbitrary Directions?

```
WRONG: Search in any direction for nearest free
─────────────────────────────────────────────────

                    ← arbitrary search
                   ╱
      blocked     ●     ends up OFF the push line!
        ●─────────┼─────
     edge_pt      │
                  │
    ┌─────────────┴─────────────┐
    │          Object           │
    └───────────────────────────┘

    Problem: Robot is no longer aligned with push direction.
             Requires complex re-alignment during push phase.


CORRECT: Search along the target pushing line only
─────────────────────────────────────────────────

                  push line
                     │
      blocked        │     search this way only
        ●────────────┼────────●────────●
     edge_pt         │      FREE     FREE
     (blocked)       │    (return    (backup)
                     │     this)
    ┌────────────────┼──────────────────┐
    │                │                  │
    │             mid_pt                │
    │                │                  │
    └────────────────┼──────────────────┘
                     │
                     ▼
              push direction

    Benefit: Robot stays on push line.
             Can drive straight forward to push.
             No re-alignment needed.
```

### Benefits of Line-Constrained Search

1. **Maintains push trajectory** - robot stays aligned with intended push direction
2. **No re-alignment needed** - robot can drive straight forward to push
3. **Predictable behavior** - approach position is always on the intended path
4. **Simpler approach phase** - straight-line approach to object

## Implementation Notes

### Computing the Push Line Direction

```python
# Direction along push line, away from object (toward approach side)
dx = edge_pt[0] - mid_pt[0]
dy = edge_pt[1] - mid_pt[1]
length = hypot(dx, dy)
dir_x, dir_y = dx / length, dy / length
```

### Finding Free Approach Position

```python
def find_approach_on_push_line(
    wavefront,
    edge_pt: Tuple[float, float],
    mid_pt: Tuple[float, float],
    step_cm: float = 0.5,      # Match wavefront resolution
    max_dist_cm: float = 15.0,  # Max search distance
) -> Optional[Tuple[float, float]]:
    """Find nearest free approach position along the push line."""

    # Direction: mid_pt → edge_pt (toward approach side)
    dx = edge_pt[0] - mid_pt[0]
    dy = edge_pt[1] - mid_pt[1]
    length = hypot(dx, dy)
    if length < 1e-6:
        return None
    dir_x, dir_y = dx / length, dy / length

    # Step 1: Check edge_pt first (ideal position)
    if wavefront.is_free(edge_pt[0] / 100, edge_pt[1] / 100):
        return edge_pt

    # Step 2: Sample along line, away from object
    dist = step_cm
    while dist <= max_dist_cm:
        candidate_x = edge_pt[0] + dir_x * dist
        candidate_y = edge_pt[1] + dir_y * dist

        # Convert to meters for wavefront query
        if wavefront.is_free(candidate_x / 100, candidate_y / 100):
            return (candidate_x, candidate_y)

        dist += step_cm

    # No free position found
    return None
```

### World Coordinate Transformation

Edge points and mid-points are generated in local (object-relative) coordinates, then transformed to world coordinates:

```python
# Local to world transformation
wx = obj.x + lx * cos(theta) - ly * sin(theta)
wy = obj.y + lx * sin(theta) + ly * cos(theta)
```

The push line direction transforms the same way, maintaining perpendicularity to the face in world coordinates.

## Configuration Parameters

| Parameter | Description | Default |
|-----------|-------------|---------|
| `standoff_multiplier` | Multiplier for standoff distance (standoff = multiplier * car_size) | 1.0 |
| `points_per_face` | Number of sample points per face (1, 3, or 15) | 15 |

## Related Files

- `robot_control/src/robot_control/controller/edge_points.py` - Edge point generation
- `robot_control/src/robot_control/controller/push.py` - Push controller implementation
- `namo_cpp/src/planning/namo_push_controller.cpp` - Reference C++ implementation
