# IMM

## Theory

The Interacting Multiple Model filter handles systems that can switch between different motion regimes. Instead of committing to a single process model, it keeps several models active in parallel and updates a probability for each one.

The standard IMM cycle is:

1. Mix previous model-conditioned states into one prior for each candidate model.
2. Predict each model with its own dynamics.
3. Update each model with the latest measurement.
4. Compute model likelihoods.
5. Update model probabilities.
6. Fuse the model-conditioned posteriors into one overall estimate.

IMM is useful when targets can alternate among cruising, accelerating, and turning.

## This Implementation

Files:

- `IMM.hpp`
- `IMM.cpp`

Implementation choices in this repo:

- Four models are fused:
  - CV
  - CA
  - CTRV
  - CTRA
- Model-specific filters live in `src/motion_models/`.
- All model states are converted into a shared 5D fusion state:
  - `[px, py, v, yaw, yaw_rate]`
- A fixed transition matrix favors staying in the current model, with smaller probabilities of switching to related models.
- The final fused output is a probability-weighted mean and covariance over the four model-conditioned posteriors.

Practical note:

- IMM is the most adaptive estimator in this repo, but it is also the most structurally complex among the recursive filters.
