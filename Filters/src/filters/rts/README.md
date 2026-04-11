# RTS Smoothers

## Theory

Rauch-Tung-Striebel smoothing is a backward correction pass applied after a forward filter has already processed the full measurement sequence.

The workflow is:

1. Run a forward filter over the sequence.
2. Store the filtered posterior and one-step prediction at each time step.
3. Starting from the last state, move backward and correct earlier states using future information.

This improves trajectory consistency and often reduces RMSE, but it is not causal. A true RTS estimate at time `k` depends on measurements after `k`.

## This Implementation

Files:

- `ekf_rts_smoother.h`
- `ekf_rts_smoother.cpp`
- `ukf_rts_smoother.h`
- `ukf_rts_smoother.cpp`

This repo contains two variants:

### EKF RTS

- Uses EKF forward-pass history from `EKFRTSStepData`
- Smoother gain uses the stored process Jacobian:
  - `C_k = P_k|k F_k^T (P_k+1|k)^-1`

### UKF RTS

- Uses UKF forward-pass history from `UKFRTSStepData`
- Smoother gain uses stored state cross-covariance:
  - `C_k = P_cross (P_k+1|k)^-1`

Shared implementation notes:

- Both variants normalize yaw-related residuals during the backward pass.
- Both are used offline after the highway simulation ends.
- The live viewer still shows the forward filter result; smoothed RMSE is reported after the run.

Practical note:

- RTS is the full offline smoother in this repo.
- If lower latency is needed, the fixed-lag smoothers under `lag_smoother/` are the bounded-window alternative.
