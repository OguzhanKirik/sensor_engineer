# EKF

## What The Algorithm Is

The Extended Kalman Filter is the standard Gaussian estimator for nonlinear systems when you still want a recursive filter with low computational cost. It assumes:

- the full posterior can be approximated by a single Gaussian
- the process model and measurement model are smooth enough to linearize locally
- the first-order Taylor approximation is good enough around the current estimate

For a nonlinear system

- `x_k = f(x_k-1, u_k, w_k)`
- `z_k = h(x_k, v_k)`

the EKF replaces the nonlinear model locally with Jacobians:

- `F_k = df/dx`
- `H_k = dh/dx`

and then runs a Kalman-style prediction and update on that local linear model.

## Prediction And Update Cycle

At each step the EKF does:

1. Predict the mean with the nonlinear process model:
   - `x_k|k-1 = f(x_k-1|k-1)`
2. Linearize the process around the previous estimate:
   - `F_k = df/dx`
3. Propagate the covariance:
   - `P_k|k-1 = F_k P_k-1|k-1 F_k^T + Q_k`
4. Predict the measurement:
   - `z_k|k-1 = h(x_k|k-1)`
5. Linearize the measurement model if needed:
   - `H_k = dh/dx`
6. Compute innovation and Kalman gain:
   - `y_k = z_k - z_k|k-1`
   - `S_k = H_k P_k|k-1 H_k^T + R_k`
   - `K_k = P_k|k-1 H_k^T S_k^-1`
7. Correct the state and covariance:
   - `x_k|k = x_k|k-1 + K_k y_k`
   - `P_k|k = (I - K_k H_k) P_k|k-1`

The quality of the EKF depends heavily on how accurate those local Jacobian-based approximations are.

## Why It Fits This Repo

This project tracks vehicles with a 5D state:

`[px, py, v, yaw, yaw_rate]`

That state is nonlinear in both motion and radar measurement space, but not so nonlinear that a first-order method becomes unusable. So EKF is a reasonable baseline.

The main tradeoff:

- fast and simple
- but more vulnerable to linearization error than UKF or CKF

## This Repo's Process And Measurement Models

### Process Model

The process model is CTRV:

- constant turn rate
- approximately constant speed over one prediction interval

That means:

- if `yaw_rate` is not near zero, the motion is propagated on a circular arc
- if `yaw_rate` is near zero, the propagation falls back to near-straight motion

### Lidar Measurement Model

Lidar measures Cartesian position directly:

- `z_lidar = [px, py]`

So the lidar observation model is linear and the EKF update reduces to a standard linear Kalman update.

### Radar Measurement Model

Radar measures:

- range `rho`
- bearing `phi`
- range rate `rho_dot`

The observation function is nonlinear:

- `rho = sqrt(px^2 + py^2)`
- `phi = atan2(py, px)`
- `rho_dot = (px vx + py vy) / rho`

This is where the EKF uses a Jacobian.

## This Implementation

Files:

- `ekf.hpp`
- `ekf.cpp`

Implementation details in this repo:

- The state is `VectorXd(5)` with:
  - `x_(0) = px`
  - `x_(1) = py`
  - `x_(2) = v`
  - `x_(3) = yaw`
  - `x_(4) = yaw_rate`
- `Prediction()` applies the nonlinear CTRV state propagation.
- `CalculateRadarJacobian()` computes the 3x5 Jacobian for the radar model.
- `UpdateLidar()` uses the fixed linear matrix `H_laser_`.
- `UpdateRadar()` builds the innovation in radar space and normalizes the bearing residual.
- The yaw angle is normalized after prediction and radar update to avoid wrap-around errors near `+-pi`.

Noise handling:

- `Q_` is built from longitudinal acceleration noise and yaw-acceleration noise
- `R_laser_` and `R_radar_` are fixed sensor-noise matrices

## Smoother Support Added In This Repo

This EKF also stores forward-pass history in `EKFRTSStepData`:

- `x_filtered`
- `P_filtered`
- `x_predicted`
- `P_predicted`
- `F_jacobian`

That history is consumed by:

- `rts/ekf_rts_smoother.*`
- `lag_smoother/ekf_fixed_lag_smoother.*`

This is important because the smoothers do not need to rerun the forward linearization logic.

## Strengths

- low computational cost
- easy to debug
- good baseline for comparison
- natural fit for real-time recursive estimation

## Weaknesses

- first-order linearization can be inaccurate in strongly nonlinear regimes
- radar updates can degrade when the estimate is poor
- can become inconsistent if Jacobians poorly capture the true local geometry

## When To Prefer EKF Here

Use EKF in this repo when:

- you want a strong baseline with minimal complexity
- you want to compare against UKF/CKF/IEKF
- you care about speed more than best nonlinear accuracy
- you want to run RTS or fixed-lag smoothing on top of a simple forward filter
