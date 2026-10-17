# Model Card: Tiny Search Policy V2

## Intended use

This model ranks reachable camera viewpoints during the target-search stage of
an indoor maze-navigation task. It is intended to run as a high-level search
policy. Nav2 remains responsible for collision-aware path planning and motion
control. The model must not directly publish motor commands.

## Model architecture

```text
12 input features -> 32 ReLU -> 16 ReLU -> 1 score
```

- Trainable parameters: 961
- Export format: NumPy `.npz`
- Weight precision: float32
- Candidate-list length during training: 29 to 32
- Selection rule: choose the reachable candidate with maximum predicted score

## Input features

The feature order, units and definitions must remain identical during
deployment.

| Index | Feature | Unit | Meaning |
| ---: | --- | --- | --- |
| 0 | `gain_area_m2` | m^2 | Previously unseen area visible from the candidate |
| 1 | `gain_fraction` | ratio | Candidate gain divided by remaining unseen area |
| 2 | `path_distance_m` | m | Planned path distance from the current pose |
| 3 | `turn_angle_rad` | rad | Absolute heading change required |
| 4 | `travel_time_s` | s | Estimated translation, rotation and observation time |
| 5 | `clearance_m` | m | Obstacle clearance at the candidate position |
| 6 | `visit_count` | count | Number of previous visits to the candidate |
| 7 | `coverage_fraction` | ratio | Fraction of observable area already searched |
| 8 | `visible_area_m2` | m^2 | Total area visible from the candidate |
| 9 | `redundancy_fraction` | ratio | Fraction of the candidate view already searched |
| 10 | `euclidean_distance_m` | m | Straight-line distance from current position |
| 11 | `path_detour_ratio` | ratio | Path distance divided by Euclidean distance |

Before inference, each feature is normalized using the exported arrays:

```text
x_normalized = (x - feature_mean) / feature_std
```

Do not recompute these statistics on the Raspberry Pi.

## Exported arrays

| Key | Shape | Type |
| --- | --- | --- |
| `feature_mean` | `(12,)` | float32 |
| `feature_std` | `(12,)` | float32 |
| `feature_names` | `(12,)` | Unicode strings |
| `weight_1` | `(32, 12)` | float32 |
| `bias_1` | `(32,)` | float32 |
| `weight_2` | `(16, 32)` | float32 |
| `bias_2` | `(16,)` | float32 |
| `weight_3` | `(1, 16)` | float32 |
| `bias_3` | `(1,)` | float32 |

## Training data and split

- Synthetic training maps: 30
- Decision states: 3,881
- Candidate rows: 124,187
- Features per candidate: 12
- Training maps: 18
- Validation maps: 6
- Held-out test maps: 6
- Expert: finite two-step look-ahead heuristic

## Supervised evaluation

| Split | States | Top-1 | Top-3 | Regret |
| --- | ---: | ---: | ---: | ---: |
| Training | 2,315 | 62.59% | 86.87% | 0.04117 |
| Validation | 827 | 62.39% | 84.52% | 0.04316 |
| Test | 739 | 60.62% | 84.98% | 0.03962 |

Training stopped early at epoch 174. The NumPy and PyTorch implementations had
a maximum output difference of `4.768e-07` during export validation.

## Held-out rollout evaluation

Evaluation used six unseen maps, three shared initial poses per map, and 18
paired trials per policy.

| Policy | Coverage | Expected detection time | Total time | Path length | Rotation |
| --- | ---: | ---: | ---: | ---: | ---: |
| Coverage only | 99.60% | 428.84 +/- 188.90 s | 1434.64 +/- 643.99 s | 339.54 +/- 155.69 m | 3085 +/- 929 deg |
| Immediate time-aware | 99.56% | 125.91 +/- 39.43 s | 481.22 +/- 165.58 s | 100.81 +/- 34.90 m | 3110 +/- 1098 deg |
| Two-step expert | 99.60% | 103.71 +/- 29.12 s | 402.07 +/- 124.44 s | 83.24 +/- 26.24 m | 2685 +/- 807 deg |
| Tiny MLP | 99.58% | 101.04 +/- 28.07 s | 370.93 +/- 125.55 s | 73.30 +/- 25.53 m | 3065 +/- 1010 deg |

The Tiny MLP reduced expected detection time by 19.75% and path length by
27.29% relative to the immediate time-aware baseline. Its expected detection
time was comparable to the two-step teacher. The 2.57% numerical difference in
favour of the MLP is not treated as evidence of statistically significant
superiority.

## Frozen artifact

```text
Filename: tiny_search_policy_weights.npz
Size: 5,747 bytes
SHA256: 3f2c9e784f71d3d5921f8e699d0013ca377034ff70d6f87722b73062d132a3b5
```

Always verify the checksum after transfer.

## Limitations

The current evidence is based on synthetic occupancy maps and assumes:

- a 70-degree horizontal camera field of view;
- a 3.0 m observation range;
- correct and static occupancy maps;
- uniformly distributed target locations over observable free space;
- nominal linear and angular speeds;
- successful low-level navigation;
- perfect observation within the simulated visibility mask.

The evaluation does not yet model localization drift, wheel slip, dynamic
obstacles, camera calibration error, object-detector false positives or false
negatives, or target-coordinate estimation error.

## Required deployment checks

1. Verify model SHA256.
2. Verify the exact 12 feature names and order against `feature_names`.
3. Compare NumPy inference against a saved desktop reference vector.
4. Measure Raspberry Pi inference and full decision latency.
5. Confirm every selected pose is reachable before sending a Nav2 goal.
6. Fall back to the immediate time-aware policy if inference is invalid.
7. Validate target-search success and coordinate error on the physical robot.
