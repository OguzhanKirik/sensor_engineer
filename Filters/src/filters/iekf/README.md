# IEKF

## What The Algorithm Is

The Iterated Extended Kalman Filter keeps the EKF structure but improves the measurement update for nonlinear observations. Instead of linearizing once and accepting that local approximation, it relinearizes multiple times around the latest iterate.

Conceptually, IEKF is trying to reduce the error caused by:

- computing the radar Jacobian too far away from the final corrected state
- using one local tangent for a measurement function that is noticeably curved

This is especially relevant for radar measurements because bearing and range-rate can change sharply with position and heading.

## Core Idea

A standard EKF radar update does:

1. linearize at the predicted state
2. compute one innovation
3. update once

IEKF instead does:

1. start from the predicted state
2. build the Jacobian at the current iterate
3. compute the innovation at that iterate
4. compute a new corrected iterate
5. repeat until convergence or a maximum iteration count

So IEKF is not a different process model. It is mainly a more careful nonlinear measurement update.

## Why It Helps

If the measurement model is curved enough, the EKF tangent can point in the wrong direction. IEKF improves that by moving the tangent point during the update. In practice this often gives:

- better radar consistency than EKF
- lower update bias under stronger nonlinearity
- better behavior when the predicted state is not very close to the true state

## This Repo's Setup

State:

`[px, py, v, yaw, yaw_rate]`

Process model:

- CTRV, same as EKF

Measurements:

- lidar: linear position update
- radar: nonlinear `[rho, phi, rho_dot]`

So the only place IEKF differs materially from EKF in this repo is the radar update path.

## This Implementation

Files:

- `iekf.hpp`
- `iekf.cpp`

Implementation details:

- `Prediction()` is effectively the same CTRV prediction logic used by the EKF.
- `UpdateLidar()` is the same style of linear update as EKF.
- `UpdateRadar()` performs iterative relinearization.

Key tuning parameters:

- `max_iterations_ = 3`
- `convergence_threshold_ = 1e-5`

Update flow used here:

1. Save the predicted state and covariance.
2. For each iteration:
   - recompute the radar Jacobian at the current iterate
   - recompute predicted radar measurement
   - form innovation
   - compute Kalman gain using the frozen predicted covariance
   - update the iterate
3. Stop when the state change is small enough or the iteration limit is hit.
4. Update covariance once using the final Jacobian.

This is a pragmatic IEKF implementation, not a more formal second-order or damped Gauss-Newton variant.

## Strengths

- more accurate radar update than plain EKF in many cases
- still much lighter than full sigma-point methods
- preserves the recursive structure and low-memory profile

## Weaknesses

- still fundamentally a local linearization method
- performance depends on the quality of the predicted prior
- not as generally robust to strong nonlinearities as UKF/CKF in some cases
- iterative update adds some cost and tuning burden

## When To Prefer IEKF Here

Use IEKF in this repo when:

- EKF is a bit too crude for radar
- you want to stay in the Jacobian-based family
- you want a middle step between EKF and UKF
- you care about keeping the implementation conceptually close to EKF
