# CKF

## Theory

The Cubature Kalman Filter is another sigma-point-style nonlinear Gaussian filter. Instead of the unscented transform, it uses a spherical-radial cubature rule to approximate Gaussian-weighted integrals.

Compared with UKF:

- It avoids explicit alpha/beta/kappa tuning.
- It uses `2n` cubature points for an `n`-dimensional state.
- All cubature points have equal weights.

Like UKF, CKF is designed for nonlinear process and measurement models without resorting to Jacobians.

## This Implementation

Files:

- `ckf.h`
- `ckf.cpp`

Implementation choices in this repo:

- State is `[px, py, v, yaw, yaw_rate]`.
- Process model is CTRV.
- Cubature points are generated from the Cholesky factor of the covariance.
- Equal weights are used for all cubature points.
- Process noise is added directly to the predicted covariance after cubature propagation.
- Both lidar and radar use the cubature-point measurement transform path.

Practical note:

- In this codebase CKF is positioned as a nonlinear alternative to UKF with simpler parameter handling.
