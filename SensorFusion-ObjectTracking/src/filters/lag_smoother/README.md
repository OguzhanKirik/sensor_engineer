# Fixed-Lag Smoothers

## What The Algorithm Is

A fixed-lag smoother is the bounded-delay version of smoothing. It sits between:

- a pure online filter
- a full fixed-interval smoother like RTS

Instead of waiting until the full sequence is complete, it corrects state `x_k` using only a finite future window. If the lag is `L`, the estimate for `x_k` uses data roughly up to `k + L`.

So it gives:

- better accuracy than pure filtering
- lower latency than full RTS

## Why It Matters

Full RTS is excellent offline, but often impractical if you want output before the whole sequence ends. Fixed-lag smoothing is the standard compromise:

- delayed, but not fully offline
- better estimates, but not maximum possible smoothing

This is often closer to what a practical tracking system can tolerate.

## Generic Fixed-Lag Idea

At time `k`, you keep a short history window and revise only states inside that window. Once a state becomes older than the lag horizon, it is no longer modified.

So compared with full RTS:

- memory is bounded
- latency is bounded
- computation is local
- the result is less accurate than using the entire future sequence

## This Repo's Two Variants

Files:

- `ekf_fixed_lag_smoother.h`
- `ekf_fixed_lag_smoother.cpp`
- `ukf_fixed_lag_smoother.h`
- `ukf_fixed_lag_smoother.cpp`

### EKF Fixed-Lag

- reuses `EKFRTSStepData`
- uses the same RTS-style backward correction formula locally
- uses the stored Jacobian-based smoother gain

### UKF Fixed-Lag

- reuses `UKFRTSStepData`
- uses the stored state cross-covariance
- applies the same unscented smoother logic locally

## This Implementation

Current implementation choice in this repo:

- each target index builds a local smoothing window
- the backward recursion is run only from the end of that local window back to the target index
- yaw residuals are normalized during the local backward pass

This is implemented as a post-run smoothing pass in the highway harness, even though the algorithm itself is conceptually suitable for bounded-delay operation.

That means:

- the current executables still report results after the run
- the code structure already reflects a fixed-lag algorithm
- the visualization is not yet delayed-live playback

## Strengths

- practical compromise between filtering and full smoothing
- bounded delay
- bounded memory
- directly comparable with full RTS in the same codebase

## Weaknesses

- less accurate than full RTS
- still requires future information within the lag window
- current harness uses it offline rather than in a true delayed streaming mode

## When To Prefer Fixed-Lag Here

Use fixed-lag smoothing in this repo when:

- you want a more practical smoother than full RTS
- you want to compare delay-vs-accuracy tradeoffs
- you care about smoother estimates but do not want full-sequence dependence
