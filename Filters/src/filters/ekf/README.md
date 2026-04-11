# EKF

## Theory

The Extended Kalman Filter linearizes a nonlinear state-space model around the current estimate. For each time step it:

1. Predicts the next state with the nonlinear process model.
2. Linearizes that process locally with a Jacobian.
3. Propagates the covariance through the linearized model.
4. Uses either a linear measurement model or a locally linearized measurement model to update the prediction.

In this project the state is:

`[px, py, v, yaw, yaw_rate]`

The process model is CTRV (Constant Turn Rate and Velocity). Lidar is treated as a linear position measurement, while radar is handled through a nonlinear polar measurement model.

## This Implementation

Files:

- `ekf.hpp`
- `ekf.cpp`

Implementation choices in this repo:

- The EKF predicts with a CTRV motion model instead of a simple constant-velocity Cartesian model.
- Radar updates use an explicit Jacobian for `[rho, phi, rho_dot]`.
- Lidar updates use a fixed linear measurement matrix that observes only `px` and `py`.
- The filter stores forward-pass snapshots in `EKFRTSStepData` so offline smoothers can reuse:
  - filtered posterior `x_k|k`, `P_k|k`
  - one-step prediction `x_k+1|k`, `P_k+1|k`
  - process Jacobian `F_k`
- That history is consumed by:
  - `rts/ekf_rts_smoother.*`
  - `lag_smoother/ekf_fixed_lag_smoother.*`

Practical note:

- This EKF is causal and runs online inside the highway simulation.
- The smoother-related history cache adds offline post-processing capability without changing the live estimate path.
