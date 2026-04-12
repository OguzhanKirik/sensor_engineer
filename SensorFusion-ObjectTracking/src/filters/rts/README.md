# RTS Smoothers

## What The Algorithm Is

The Rauch-Tung-Striebel smoother is a backward pass applied after a forward filter has already processed the full data sequence. It belongs to the classical fixed-interval smoothing family.

Key point:

- filtering estimates `x_k` using measurements up to time `k`
- smoothing revises `x_k` using measurements after time `k` as well

So RTS is not causal. It needs future information.

## Why Smoothing Helps

A forward filter makes irreversible local decisions as measurements arrive. If a later measurement reveals that the earlier estimate was slightly off, the filter usually cannot go back and fix the past. RTS can.

That typically improves:

- trajectory consistency
- velocity smoothness
- RMSE over the whole sequence

## Generic RTS Recursion

After a forward pass, you need for each time step:

- filtered posterior `x_k|k`, `P_k|k`
- one-step prediction `x_k+1|k`, `P_k+1|k`

Then the smoother runs backward:

- `x_k|N = x_k|k + C_k (x_k+1|N - x_k+1|k)`
- `P_k|N = P_k|k + C_k (P_k+1|N - P_k+1|k) C_k^T`

where `C_k` is the smoother gain.

The exact form of `C_k` depends on the forward filter.

## This Repo's Two Variants

Files:

- `ekf_rts_smoother.h`
- `ekf_rts_smoother.cpp`
- `ukf_rts_smoother.h`
- `ukf_rts_smoother.cpp`

### EKF RTS

For the EKF-based smoother, the gain uses the stored local process Jacobian:

- `C_k = P_k|k F_k^T (P_k+1|k)^-1`

So the EKF forward pass must store:

- filtered state/covariance
- predicted state/covariance
- process Jacobian

That is what `EKFRTSStepData` contains in this repo.

### UKF RTS

For the UKF-based smoother, the gain uses the unscented state cross-covariance:

- `C_k = P_cross (P_k+1|k)^-1`

So the UKF forward pass must store:

- filtered state/covariance
- predicted state/covariance
- `P_cross`

That is what `UKFRTSStepData` contains in this repo.

This is important: a proper UKF smoother should not just imitate the EKF Jacobian formula.

## This Implementation

Implementation details shared by both variants:

- each smoother can either:
  - rerun a fresh forward filter from a measurement log
  - smooth from already stored step history
- angle residuals are normalized during the backward pass
- smoothing is run after the highway simulation ends
- the live viewer still shows the forward filter output
- smoothed RMSE is reported post-run in the terminal

## Strengths

- usually improves trajectory quality
- mathematically clean extension of a forward filter
- especially useful for evaluation and offline analysis
- natural next step after EKF or UKF is already implemented

## Weaknesses

- not real-time
- requires storing forward-pass history
- full benefit only appears after the whole sequence is available

## When To Prefer RTS Here

Use RTS in this repo when:

- you want the best offline trajectory estimate from EKF or UKF
- you want to compare filtering versus smoothing clearly
- latency is not a constraint
