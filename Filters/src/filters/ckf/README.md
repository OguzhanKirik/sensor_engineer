# CKF

## What The Algorithm Is

The Cubature Kalman Filter is another nonlinear Gaussian filter in the same family as UKF. It replaces Jacobians with deterministic integration points, but unlike UKF it is based on a cubature rule for approximating Gaussian-weighted integrals.

The core idea is:

- instead of linearizing the nonlinear function
- and instead of manually tuning an unscented transform
- approximate the relevant Gaussian integrals with spherical-radial cubature points

CKF is often described as a parameter-free nonlinear Gaussian filter.

## Why It Exists Beside UKF

EKF, UKF, and CKF are all trying to solve the same problem:

- propagate a mean and covariance through nonlinear dynamics and measurements

They differ in how they approximate the transformed distribution:

- EKF: first-order Taylor expansion
- UKF: unscented sigma points
- CKF: cubature integration points

In practice, CKF often performs similarly to UKF but with a simpler point construction and equal weights.

## Cubature Idea

For an `n`-dimensional Gaussian, CKF uses `2n` points:

- `x + sqrt(n) * column_i(sqrt(P))`
- `x - sqrt(n) * column_i(sqrt(P))`

Each point has equal weight.

Those points are propagated through the nonlinear process or measurement model, and the transformed mean and covariance are reconstructed from them.

## This Repo's Setup

State:

`[px, py, v, yaw, yaw_rate]`

Process model:

- CTRV

Measurements:

- lidar: Cartesian position
- radar: range, bearing, range rate

So CKF is used here as a nonlinear alternative to UKF with a different approximation philosophy.

## This Implementation

Files:

- `ckf.h`
- `ckf.cpp`

Implementation details:

- `GenerateCubaturePoints()` builds `2n` points from the Cholesky factor of `P`.
- `ProcessModel()` propagates one point through CTRV dynamics.
- `PredictCubaturePoints()` applies that process model point by point.
- `Predict()` reconstructs mean and covariance from the propagated cubature set.
- `UpdateLidar()` and `UpdateRadar()` both transform cubature points into measurement space and compute the update from those transformed sets.

One repo-specific simplification:

- process noise is added directly to the predicted covariance after cubature propagation rather than augmenting the point set with process-noise variables the way UKF often does

That keeps the implementation compact, though it is a somewhat simpler formulation than a fully augmented CKF.

## Strengths

- nonlinear filtering without Jacobians
- no UKF alpha/beta/kappa parameter discussion
- mathematically clean point set with equal weights
- good comparison point against UKF in this repo

## Weaknesses

- more expensive than EKF
- still Gaussian, so it cannot capture multimodal posteriors
- this implementation uses a simplified process-noise treatment

## When To Prefer CKF Here

Use CKF in this repo when:

- you want a nonlinear Gaussian filter without Jacobians
- you want an alternative to UKF with simpler point-weight structure
- you want to compare sigma-point-style methods without going to particles
