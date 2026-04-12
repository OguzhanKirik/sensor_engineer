# IMM

## What The Algorithm Is

The Interacting Multiple Model filter is designed for targets whose motion regime changes over time. A single model is often too restrictive because a vehicle may:

- cruise at near-constant velocity
- accelerate or decelerate
- turn at roughly constant turn rate
- turn while accelerating

IMM addresses this by running several motion models in parallel and maintaining a probability for each model.

## Conceptual Steps

The standard IMM cycle is:

1. Start with model probabilities from the previous step.
2. Use the model-transition matrix to compute mixing probabilities.
3. Build one mixed prior for each candidate model.
4. Run each model-specific filter forward.
5. Compute a likelihood for the latest measurement under each model.
6. Update model probabilities with Bayes' rule.
7. Fuse all model-conditioned posteriors into one estimate.

So IMM is not one filter formula. It is a supervisory architecture around several filters.

## Why It Fits Tracking

A highway target does not always obey one motion model. IMM is useful because:

- a straight-driving segment may favor CV
- a maneuver may favor CTRV or CTRA
- acceleration may favor CA or CTRA

Rather than choosing one model manually, IMM shifts probability among them over time.

## This Repo's Model Set

Files:

- `IMM.hpp`
- `IMM.cpp`

The fused model bank here contains four motion models:

- CV
- CA
- CTRV
- CTRA

Those implementations live in `src/motion_models/`.

To combine them, the repo maps each model into a common fusion state:

`[px, py, v, yaw, yaw_rate]`

That shared state is what the highway harness consumes.

## This Implementation

Implementation details:

- `PI_` is a fixed 4x4 transition matrix
- diagonal terms are larger, so the system prefers staying in the same mode
- related models have larger cross-transition probabilities than unrelated ones
- `MixStates()` computes:
  - normalization constants `c_j`
  - conditional mixing probabilities `mu_ij`
  - mixed state/covariance for each model
- `Predict()` runs each model-specific filter
- `Update()` applies the latest measurement to each model-specific filter
- `UpdateModelProbabilities()` uses per-model likelihoods to update `mu_`
- `FuseEstimate()` merges all model-conditioned estimates into one final state and covariance

This implementation is structurally clear and useful for comparison, though it uses a fixed transition matrix rather than an adaptive one.

## Strengths

- handles motion-regime changes better than a single-model filter
- often best overall accuracy when targets truly switch behavior
- provides interpretable model probabilities
- strong conceptual bridge toward practical maneuvering-target tracking

## Weaknesses

- more code and tuning complexity
- computational cost scales with number of models
- performance depends on model-set quality and transition matrix design
- if the true motion is not well captured by the bank, IMM still struggles

## When To Prefer IMM Here

Use IMM in this repo when:

- you want the most adaptive recursive estimator in the codebase
- targets may switch among cruising, accelerating, and turning
- you want to compare single-model and multi-model tracking directly
