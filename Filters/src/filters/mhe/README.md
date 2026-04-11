# MHE

## Theory

Moving Horizon Estimation solves a sliding-window optimization problem rather than performing a purely recursive Bayesian update. Instead of keeping only the latest state, it optimizes over a recent state trajectory subject to:

- process-model residuals
- measurement residuals
- arrival-cost terms that summarize information before the window
- optional state or input constraints

MHE is attractive when:

- nonlinearities are strong
- constraints matter
- robust cost functions are desired
- smoothing over a recent window is acceptable

## This Implementation

Files:

- `mhe.h`
- `mhe.cpp`

Current status in this repo:

- This is a scaffold, not a full nonlinear MHE solver.
- The class maintains:
  - a measurement window
  - arrival-cost placeholders
  - per-node warm-start state/covariance
- An embedded EKF currently supplies those warm starts.
- `Solve()` currently returns the latest EKF warm-start result instead of solving an optimization problem.

What is already in place:

- horizon management
- reset and arrival-cost APIs
- integration into the highway harness and RMSE pipeline

What is missing for true MHE:

- explicit decision variables for the window state sequence
- process and measurement residual construction
- nonlinear least-squares backend
- convergence and constraint handling

Practical note:

- This module is useful today for integration and interface work, but not yet for meaningful MHE performance comparisons.
