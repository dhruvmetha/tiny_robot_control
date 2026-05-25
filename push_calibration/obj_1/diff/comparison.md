# Real vs Sim — push calibration diff (tree walk)

**Headline loss `L = 17.239 cm`**  (w_theta = 0.5 cm/deg, n_leaves = 11)

## Loss breakdown

Loss is object pose only. Robot pose gap is reported below for diagnostic value but does not enter the headline.

| Component | cm |
|---|---:|
| object xy           | 6.672 |
| object θ (weighted) | 10.567 |

## Per-field statistics (over leaves)

| Field | n | mean | median | p90 | max |
|---|---:|---:|---:|---:|---:|
| gap_object_xy_cm | 11 | 6.672 | 2.663 | 13.442 | 27.453 |
| gap_object_theta_deg | 11 | 21.134 | 0.771 | 61.974 | 82.070 |
| gap_robot_xy_cm | 11 | 2.963 | 2.105 | 6.539 | 7.203 |
| gap_robot_theta_deg | 11 | 18.855 | 19.884 | 20.582 | 22.581 |

## Per-leaf gaps (real trials averaged)

| edge | depth | n_real | obj xy (cm) | obj θ (°) | rob xy (cm) | rob θ (°) |
|---:|---:|---:|---:|---:|---:|---:|
| 1 | 2 | 3 | 2.378 | 11.99 | 1.659 | 19.73 |
| 1 | 6 | 3 | 9.328 | 61.97 | 2.762 | 17.25 |
| 1 | 9 | 3 | 13.442 | 82.07 | 6.539 | 20.11 |
| 15 | 2 | 3 | 0.405 | 0.47 | 2.457 | 15.71 |
| 15 | 6 | 3 | 0.497 | 0.24 | 1.808 | 20.58 |
| 15 | 9 | 3 | 1.301 | 0.39 | 2.105 | 19.88 |
| 45 | 2 | 3 | 0.383 | 0.36 | 1.801 | 20.16 |
| 45 | 6 | 3 | 2.663 | 0.65 | 1.593 | 15.03 |
| 45 | 9 | 3 | 4.740 | 0.77 | 3.281 | 22.58 |
| 59 | 2 | 3 | 10.806 | 43.88 | 1.383 | 16.35 |
| 59 | 6 | 3 | 27.453 | 29.67 | 7.203 | 20.03 |

## Worst-5 leaves by object xy + θ gap

| edge | depth | obj xy (cm) | obj θ (°) | rob xy (cm) | rob θ (°) |
|---:|---:|---:|---:|---:|---:|
| 1 | 9 | 13.442 | 82.07 | 6.539 | 20.11 |
| 59 | 6 | 27.453 | 29.67 | 7.203 | 20.03 |
| 1 | 6 | 9.328 | 61.97 | 2.762 | 17.25 |
| 59 | 2 | 10.806 | 43.88 | 1.383 | 16.35 |
| 1 | 2 | 2.378 | 11.99 | 1.659 | 19.73 |

## ⚠ Controller-setting drift (expected zero)

Non-zero values mean real and sim ran with different controller settings. Fix the config drift before trusting the calibration numbers above.

| edge | depth | trial | setting | sim − real |
|---:|---:|---|---|---:|
| 1 | 2 | trial1 | push_controller_max_speed | -0.3464 |
| 1 | 2 | trial1 | push_path_length_cm | +0.3000 |
| 1 | 2 | trial2 | push_controller_max_speed | -0.3464 |
| 1 | 2 | trial2 | push_path_length_cm | +0.3000 |
| 1 | 2 | trial3 | push_controller_max_speed | -0.3464 |
| 1 | 2 | trial3 | push_path_length_cm | +0.3000 |
| 1 | 6 | trial1 | push_controller_max_speed | -0.3464 |
| 1 | 6 | trial1 | push_path_length_cm | +0.3000 |
| 1 | 6 | trial2 | push_controller_max_speed | -0.3464 |
| 1 | 6 | trial2 | push_path_length_cm | +0.3000 |
| 1 | 6 | trial3 | push_controller_max_speed | -0.3464 |
| 1 | 6 | trial3 | push_path_length_cm | +0.3000 |
| 1 | 9 | trial1 | push_controller_max_speed | -0.3464 |
| 1 | 9 | trial1 | push_path_length_cm | +0.3000 |
| 1 | 9 | trial2 | push_controller_max_speed | -0.3464 |
| 1 | 9 | trial2 | push_path_length_cm | +0.3000 |
| 1 | 9 | trial3 | push_controller_max_speed | -0.3464 |
| 1 | 9 | trial3 | push_path_length_cm | +0.3000 |
| 15 | 2 | trial1 | push_controller_max_speed | -0.3464 |
| 15 | 2 | trial1 | push_path_length_cm | +0.3000 |

