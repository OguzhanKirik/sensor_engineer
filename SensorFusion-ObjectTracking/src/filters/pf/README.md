# Particle Filter

## What The Algorithm Is

A Particle Filter represents the posterior distribution with a set of weighted samples:

- each particle is one hypothetical state
- its weight says how plausible that state is under the latest data

Unlike EKF, UKF, and CKF, a particle filter does not require the posterior to remain Gaussian. That makes it attractive for:

- nonlinear systems
- non-Gaussian noise
- ambiguous or multimodal belief states

## Core Cycle

The bootstrap particle filter used here follows this pattern:

1. Initialize particles around the first observation.
2. Propagate each particle through the motion model with sampled noise.
3. Evaluate measurement likelihood for each particle.
4. Multiply old weights by the new likelihoods.
5. Normalize weights.
6. Resample if particle degeneracy becomes severe.
7. Extract a state estimate from the weighted particle cloud.

This is fundamentally different from Kalman-style filters:

- no single mean/covariance is propagated as the internal belief
- the distribution is represented by the cloud itself

## Why It Matters In This Repo

The highway scenario is still simple enough that Gaussian filters work reasonably well, but PF is useful as a conceptually different estimator:

- it does not linearize
- it does not assume a Gaussian posterior
- it can tolerate more nonlinear behavior if enough particles are used

## This Repo's Setup

State:

`[px, py, v, yaw, yaw_rate]`

Process model:

- CTRV-like propagation with sampled process noise

Measurements:

- lidar likelihood in Cartesian space
- radar likelihood in polar space

## This Implementation

Files:

- `pf.h`
- `pf.cpp`

Implementation details:

- `n_particles_ = 1200`
- particles are initialized around the first measurement with broad random spread
- propagation samples:
  - longitudinal acceleration noise
  - yaw-acceleration noise
- lidar update multiplies each weight by a 2D Gaussian likelihood in `(px, py)`
- radar update multiplies each weight by a Gaussian likelihood in `(rho, phi, rho_dot)`
- `EffectiveSampleSize()` monitors degeneracy
- `ResampleSystematic()` performs low-variance systematic resampling
- a small roughening step is added after resampling
- `ComputeStateMean()` exports the weighted mean, with yaw averaged on the circle

This is a practical bootstrap PF, not a highly specialized proposal-distribution design.

## Strengths

- handles non-Gaussian belief states better than Kalman-family methods
- no Jacobians required
- conceptually robust to strong nonlinearities
- good educational contrast against EKF/UKF/CKF

## Weaknesses

- more expensive than Gaussian filters
- quality depends strongly on particle count
- can suffer sample impoverishment
- resampling design matters
- this repo’s PF is still fairly simple and not tuned for extreme ambiguity

## When To Prefer PF Here

Use PF in this repo when:

- you want to compare Gaussian and non-Gaussian estimators
- you expect nonlinear behavior that may not stay well approximated by one Gaussian
- you accept higher compute cost in exchange for distributional flexibility
