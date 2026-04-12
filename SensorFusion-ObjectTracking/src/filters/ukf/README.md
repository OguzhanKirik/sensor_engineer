# UKF

## What The Algorithm Is

The Unscented Kalman Filter is a recursive Gaussian estimator for nonlinear systems that avoids explicit first-order linearization. Instead of Jacobians, it uses a deterministic set of sigma points to capture how a Gaussian distribution moves through a nonlinear function.

The main assumption is still:

- the posterior can be represented well by a single Gaussian

But instead of approximating the function, UKF approximates the transformed distribution.

That is the key difference from EKF.

## Core Unscented Transform Idea

Suppose you have a Gaussian random variable with mean `x` and covariance `P`. The UKF:

1. chooses sigma points around `x`
2. propagates those points through the nonlinear model
3. reconstructs the transformed mean and covariance from the transformed points

This often gives better nonlinear behavior than EKF because it captures curvature effects beyond a single tangent approximation.

## Prediction And Update Cycle

For each time step the UKF does:

1. Build an augmented state that includes process-noise variables.
2. Generate sigma points from the augmented covariance.
3. Propagate every sigma point through the nonlinear process model.
4. Reconstruct predicted state mean and covariance.
5. Transform predicted sigma points into measurement space.
6. Reconstruct predicted measurement mean and covariance.
7. Compute state-measurement cross-covariance.
8. Form Kalman gain and update the state.

Important conceptual difference versus EKF:

- EKF linearizes `f` and `h`
- UKF propagates representative samples through `f` and `h`

## Why It Fits This Repo

This repo uses:

- nonlinear CTRV dynamics
- nonlinear radar measurements

That is exactly the kind of setting where UKF usually outperforms EKF in consistency and accuracy.

## This Repo's State And Noise Model

State:

`[px, py, v, yaw, yaw_rate]`

Augmented state:

- 5 state variables
- 2 process-noise variables

So:

- `n_x = 5`
- `n_aug = 7`

The added noise terms represent:

- longitudinal acceleration noise
- yaw-acceleration noise

## This Implementation

Files:

- `ukf.h`
- `ukf.cpp`

Implementation details:

- `Prediction()` constructs the augmented Gaussian explicitly.
- Sigma points are generated using the Cholesky factor of the augmented covariance.
- The process model is CTRV.
- Radar and lidar updates are both implemented through sigma-point measurement transforms.
- The code stores:
  - `last_x_pred_`
  - `last_P_pred_`
  - `last_P_cross_`

Those cached quantities are later reused by smoothers.

Angle handling:

- yaw residuals are normalized during covariance reconstruction
- radar bearing residuals are normalized during update

## Smoother Support Added In This Repo

The UKF stores forward-pass snapshots in `UKFRTSStepData`:

- filtered posterior
- one-step prediction
- state cross-covariance `P_cross`

That history is consumed by:

- `rts/ukf_rts_smoother.*`
- `lag_smoother/ukf_fixed_lag_smoother.*`

This is an important implementation detail. For a proper UKF smoother, the cross-covariance matters more than a Jacobian.

## Strengths

- strong nonlinear performance
- no measurement Jacobians required
- usually better radar behavior than EKF
- natural fit for the smoothing extensions added in this repo

## Weaknesses

- more expensive than EKF/IEKF
- still assumes approximate Gaussianity
- can be sensitive to sigma-point scaling choices in more general implementations
- heavier to debug than Jacobian-based filters

## When To Prefer UKF Here

Use UKF in this repo when:

- radar nonlinearities matter
- you want a strong default nonlinear recursive filter
- you plan to compare against UKF RTS or UKF fixed-lag smoothing
- you want better nonlinear behavior without moving to particle methods or optimization-based estimation
