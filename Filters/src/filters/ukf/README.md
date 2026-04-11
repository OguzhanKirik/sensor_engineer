# UKF

## Theory

The Unscented Kalman Filter avoids first-order linearization of the process and measurement models. Instead, it propagates a deterministic set of sigma points through the nonlinear functions and reconstructs the posterior mean and covariance from those transformed points.

At each step the UKF:

1. Builds an augmented Gaussian that includes process noise.
2. Generates sigma points from that distribution.
3. Propagates the sigma points through the nonlinear process model.
4. Reconstructs the predicted mean and covariance.
5. Transforms predicted sigma points into measurement space.
6. Uses the resulting predicted measurement statistics for the update.

This usually handles nonlinear radar measurements better than EKF.

## This Implementation

Files:

- `ukf.h`
- `ukf.cpp`

Implementation choices in this repo:

- State is `[px, py, v, yaw, yaw_rate]`.
- Process model is CTRV.
- Augmented dimension is 7:
  - 5 state variables
  - longitudinal acceleration noise
  - yaw-acceleration noise
- Radar and lidar updates are both handled through sigma-point transformations.
- The class stores forward-pass snapshots in `UKFRTSStepData`, including:
  - filtered posterior
  - one-step prediction
  - state cross-covariance `P_cross`

That history is reused by:

- `rts/ukf_rts_smoother.*`
- `lag_smoother/ukf_fixed_lag_smoother.*`

Practical note:

- This implementation keeps the online UKF estimate separate from the offline smoothing passes.
