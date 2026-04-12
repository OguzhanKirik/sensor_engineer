# MHE

## What The Algorithm Is

Moving Horizon Estimation is a sliding-window optimization method. Instead of updating only the latest state recursively, it solves for the best recent trajectory over a finite horizon.

A generic MHE problem minimizes something like:

- arrival cost from information before the horizon
- process residual cost over the horizon
- measurement residual cost over the horizon
- optional robust penalties or hard constraints

So unlike EKF/UKF, MHE is not primarily a closed-form recursive estimator. It is an optimization problem solved repeatedly.

## Why It Is Interesting

MHE is attractive because it can naturally handle:

- nonlinear process models
- nonlinear measurement models
- explicit state constraints
- actuator constraints
- robust losses for outliers
- smoothing over a recent window

In many practical systems, MHE is conceptually closer to online optimization than to classic Kalman filtering.

## Typical Formulation

Over a horizon of states `x_0 ... x_N`, MHE usually solves for the sequence that minimizes:

1. deviation from an arrival prior at the start of the window
2. process-model residuals between successive states
3. measurement residuals at each node

The latest state in that optimized window becomes the current estimate.

That means MHE is:

- not fully offline like RTS over a full sequence
- not purely causal like EKF
- a bounded-horizon optimization-based estimator

## This Repo's Current Status

Files:

- `mhe.h`
- `mhe.cpp`

This repo does not yet contain a true solver-backed MHE.

What exists:

- horizon management with a deque of `MHENode`
- per-node measurement storage
- arrival-cost placeholders
- `SetWindowSize()`
- `SetArrivalCost()`
- `Reset()`
- integration into the highway harness and RMSE logic

How the current estimate is produced:

- the class embeds an EKF as a warm-start generator
- every incoming measurement is first processed by that EKF
- the warm-start state/covariance is stored with the node
- `Solve()` currently returns the latest warm-start result

So this module is best understood as:

- interface scaffold
- data-structure scaffold
- integration scaffold

not yet as a finished MHE algorithm

## What A Full Version Would Need

To become a real MHE, this module would need:

- explicit window decision variables
- process residual construction for CTRV dynamics
- lidar and radar residual terms
- arrival cost weighting
- a nonlinear least-squares backend
- solver iteration and convergence control
- optional robust loss or constraints

Likely solver choices would be:

- Ceres
- CasADi
- IPOPT

depending on how much symbolic modeling or constraint support is desired.

## Why Keep It Anyway

Even as a scaffold, it is useful because it already defines:

- the estimator interface
- the window bookkeeping
- where a future solver would plug in
- how MHE outputs would be evaluated in the highway harness

That lowers the cost of implementing the true method later.
