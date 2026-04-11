# IEKF

## Theory

The Iterated Extended Kalman Filter improves on the EKF measurement update by repeating the local linearization several times during a single update. This matters when the measurement function is strongly nonlinear.

The idea is:

1. Start from the predicted state.
2. Linearize the measurement model around the current iterate.
3. Compute an update.
4. Re-linearize around the refined iterate.
5. Stop after convergence or a fixed iteration count.

This is especially useful for radar, where range and bearing measurements are nonlinear in Cartesian state variables.

## This Implementation

Files:

- `iekf.hpp`
- `iekf.cpp`

Implementation choices in this repo:

- The process model is the same CTRV model used by the EKF.
- Lidar still uses a standard linear Kalman update.
- Radar uses iterative refinement with:
  - `max_iterations_ = 3`
  - `convergence_threshold_ = 1e-5`
- The covariance update is performed once using the final linearization from the last iterate.

Practical note:

- This implementation is still lightweight compared with UKF.
- It is a good middle ground when EKF is too crude for radar but a sigma-point method is not desired.
