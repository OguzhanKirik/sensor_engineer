# Fixed-Lag Smoothers

## Theory

A fixed-lag smoother sits between a purely online filter and a full offline smoother. Instead of waiting until the end of the whole sequence, it revises each state using only a bounded amount of future information.

For a lag `L`, the estimate at time `k` is corrected using information up to approximately `k + L`.

Benefits:

- lower latency than full RTS
- less memory than full-sequence smoothing
- better trajectory quality than a pure forward filter

Tradeoff:

- less accurate than full RTS because it does not use the entire future trajectory

## This Implementation

Files:

- `ekf_fixed_lag_smoother.h`
- `ekf_fixed_lag_smoother.cpp`
- `ukf_fixed_lag_smoother.h`
- `ukf_fixed_lag_smoother.cpp`

This repo contains two variants:

### EKF Fixed-Lag

- Reuses `EKFRTSStepData`
- Applies the same backward recursion as RTS
- Truncates the backward pass to a local window `[k, k + lag]`

### UKF Fixed-Lag

- Reuses `UKFRTSStepData`
- Uses stored unscented state cross-covariances
- Also truncates smoothing to the local lag window

Implementation notes:

- Both variants rebuild a local smoothed trajectory for each target index.
- Both normalize yaw residuals during the local backward pass.
- In the highway harness the smoother runs after the sequence ends and reports filtered vs smoothed RMSE.
- The current visualization still shows the forward filter online.

Practical note:

- This is the right module to use when you want smoother estimates without fully committing to end-of-sequence RTS smoothing.
