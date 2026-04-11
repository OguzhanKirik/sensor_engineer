# Particle Filter

## Theory

A Particle Filter represents the posterior distribution with a weighted set of random samples instead of a single Gaussian. This is useful when the posterior can become non-Gaussian or multimodal.

Typical particle-filter cycle:

1. Sample particles from a proposal distribution.
2. Propagate particles through the process model.
3. Score each particle under the measurement likelihood.
4. Normalize weights.
5. Resample when degeneracy becomes severe.
6. Estimate the output state from the weighted particle set.

## This Implementation

Files:

- `pf.h`
- `pf.cpp`

Implementation choices in this repo:

- State is `[px, py, v, yaw, yaw_rate]`.
- Propagation uses a CTRV motion prior with sampled acceleration and yaw-acceleration noise.
- Lidar and radar updates are implemented as direct likelihood multipliers on each particle.
- Degeneracy is monitored via effective sample size.
- Systematic resampling is used when the particle set collapses.
- A small roughening step is applied after resampling to reduce sample impoverishment.
- The exported estimate `x_` is the weighted particle mean, with yaw averaged on the unit circle.

Practical note:

- This is a bootstrap-style particle filter, not a Rao-Blackwellized or map-based variant.
