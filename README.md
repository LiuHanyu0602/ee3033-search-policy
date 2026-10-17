# EE3033 Learned Target-Search Policy

This repository contains the offline training and evaluation pipeline for a
lightweight learned viewpoint-selection policy for an EE3033 mobile-robot
target-search task.

The robot first constructs a 2-D occupancy map. After the target is placed,
the policy scores candidate camera viewpoints, Nav2 drives the robot to the
selected pose, and the searched-area memory is updated. Object detection and
target-coordinate estimation are separate downstream modules.

## Current status

- Synthetic maze generation: complete
- Safety-mask and candidate-viewpoint generation: complete
- Two-step heuristic teacher and expert-dataset generation: complete
- Tiny MLP training and held-out-map evaluation: complete
- NumPy model export: complete
- Raspberry Pi ROS 2 integration: in progress

The policy network has architecture `12 -> 32 -> 16 -> 1` with 961 trainable
parameters. It assigns one scalar score to each candidate viewpoint. The
highest-scoring reachable candidate is sent to the navigation layer.

## Held-out evaluation

The current V2 model was trained using 30 generated maze maps. Six maps were
held out for testing. Multi-map evaluation used three shared initial poses per
test map, giving 18 paired rollout trials.

| Policy | Coverage | Expected detection time | Total time | Path length |
| --- | ---: | ---: | ---: | ---: |
| Coverage only | 99.60% | 428.84 s | 1434.64 s | 339.54 m |
| Immediate time-aware | 99.56% | 125.91 s | 481.22 s | 100.81 m |
| Two-step expert | 99.60% | 103.71 s | 402.07 s | 83.24 m |
| Tiny MLP | 99.58% | 101.04 s | 370.93 s | 73.30 m |

Compared with the immediate time-aware baseline, the Tiny MLP reduced expected
detection time by 19.75% and path length by 27.29%, while maintaining 99.58%
observable-space coverage. Its rollout performance was comparable to the more
computationally expensive two-step heuristic teacher.

These are simulation results. They do not yet include localization drift,
Nav2 failures, camera occlusion, detector errors, or physical robot dynamics.

## Repository layout

```text
.
├── scripts/                 Offline generation, training and evaluation
├── maps/                    Small reference map used during development
├── models/                  Frozen deployment weights
├── results/v2/              Selected reproducible result files
├── MODEL_CARD.md            Model inputs, metrics and limitations
├── requirements-training.txt
└── requirements-runtime.txt
```

Generated datasets and map intermediates under `outputs/` are intentionally
excluded from Git. They can be regenerated from the scripts and recorded
configuration.

## Reproduce the V2 pipeline

Run all commands with Ubuntu system Python unless a different environment is
explicitly required.

### 1. Generate maps

```bash
/usr/bin/python3 scripts/generate_training_maps_v2.py \
  --count 30 \
  --base-seed 13033 \
  --name-prefix general_maze \
  --start-index 0 \
  --output-root outputs/training_maps_v2
```

### 2. Generate teacher decisions

```bash
/usr/bin/python3 scripts/generate_expert_dataset.py \
  --root outputs/training_maps_v2 \
  --episodes-per-map 3 \
  --max-steps 120 \
  --coverage-target 0.995 \
  --shortlist-size 32 \
  --successor-count 16 \
  --linear-speed 0.25 \
  --angular-speed 0.8 \
  --observation-time 0.25 \
  --time-discount 20.0 \
  --seed 23033
```

### 3. Train the Tiny MLP

```bash
/usr/bin/python3 scripts/train_tiny_mlp.py \
  --dataset outputs/training_maps_v2/expert_dataset.npz \
  --epochs 300 \
  --batch-size 64 \
  --learning-rate 0.001 \
  --weight-decay 0.0001 \
  --teacher-temperature 0.15 \
  --mse-weight 0.2 \
  --patience 40 \
  --seed 33033
```

### 4. Evaluate held-out maps

```bash
/usr/bin/python3 scripts/evaluate_policy_rollouts.py \
  --training-root outputs/training_maps_v2 \
  --episodes 3 \
  --seed 44033
```

## Frozen V2 deployment model

```text
models/tiny_search_policy_weights.npz
SHA256: 3f2c9e784f71d3d5921f8e699d0013ca377034ff70d6f87722b73062d132a3b5
```

The `.npz` file includes feature normalization statistics and all three layer
weights and biases. PyTorch is not required for NumPy inference.

See `MODEL_CARD.md` before using the model because feature order and units must
match training exactly.
