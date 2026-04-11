# Filters Directory

This directory contains multiple Kalman filter implementations for object tracking, plus an offline RTS smoother built on top of the EKF forward pass.

## Directory Structure

```
filters/
├── ekf/                    - Extended Kalman Filter
│   ├── ekf.hpp
│   └── ekf.cpp
├── iekf/                   - Iterated Extended Kalman Filter
│   ├── iekf.hpp
│   └── iekf.cpp
├── ukf/                    - Unscented Kalman Filter
│   ├── ukf.h
│   └── ukf.cpp
├── ckf/                    - Cubature Kalman Filter
│   ├── ckf.h
│   ├── ckf.cpp
│   └── CKF_README.md
├── pf/                     - Particle Filter
│   ├── pf.h
│   └── pf.cpp
├── mhe/                    - Moving Horizon Estimation scaffold
│   ├── mhe.h
│   └── mhe.cpp
├── rts/                    - Rauch-Tung-Striebel smoother
│   ├── ekf_rts_smoother.h
│   └── ekf_rts_smoother.cpp
└── imm/                    - Interacting Multiple Model Filter
   ├── IMM.hpp
    ├── IMM.cpp
    └── IMM_README.md
```

## Filter Types

### 1. EKF (Extended Kalman Filter)
**Location**: `ekf/`

- **Type**: Linear approximation-based filter
- **State**: [px, py, v, yaw, yaw_rate]
- **Motion Model**: CTRV (Constant Turn Rate and Velocity)
- **Best For**: Simple scenarios with moderate nonlinearity
- **Complexity**: Low
- **Performance**: Good for basic tracking

**Use Case**: When computational efficiency is critical and moderate accuracy is acceptable.

### 2. IEKF (Iterated Extended Kalman Filter)
**Location**: `iekf/`

- **Type**: Iterative refinement-based linearization filter
- **State**: [px, py, v, yaw, yaw_rate]
- **Motion Model**: CTRV (Constant Turn Rate and Velocity)
- **Best For**: Improving EKF accuracy for radar measurements
- **Complexity**: Low-Medium (slightly higher than EKF)
- **Performance**: Better than EKF for highly nonlinear radar measurements

**Key Feature**: Uses Gauss-Newton iterative refinement (max 3 iterations) to recompute the Jacobian at the current estimate rather than the prior. This improves linearization accuracy for radar updates.

**Use Case**: When you need better accuracy than EKF but want to stay with linearization-based methods. Good middle ground between EKF and UKF.

### 3. UKF (Unscented Kalman Filter)
**Location**: `ukf/`

- **Type**: Sigma points-based filter
- **State**: [px, py, v, yaw, yaw_rate]
- **Motion Model**: CTRV (Constant Turn Rate and Velocity)
- **Best For**: Nonlinear systems with good accuracy requirements
- **Complexity**: Medium
- **Performance**: Better than EKF for nonlinear motion

**Use Case**: Standard choice for most tracking applications with radar/lidar fusion.

### 4. CKF (Cubature Kalman Filter)
**Location**: `ckf/`

- **Type**: Spherical-radial cubature-based filter
- **State**: [px, py, v, yaw, yaw_rate]
- **Motion Model**: CTRV (Constant Turn Rate and Velocity)
- **Best For**: Nonlinear systems **without tuning parameters**
- **Complexity**: Medium
- **Performance**: Similar to UKF, parameter-free

**Use Case**: When you want UKF-level accuracy without the hassle of tuning α, β, κ parameters.

See [ckf/CKF_README.md](ckf/CKF_README.md) for detailed CKF documentation.

### 5. EKF + RTS Smoother
**Location**: `rts/`

- **Type**: Two-pass smoother on top of EKF
- **Forward Model**: EKF with CTRV motion model
- **Best For**: Offline trajectory refinement after a full measurement sequence
- **Complexity**: Medium
- **Performance**: Usually improves trajectory consistency and post-run RMSE relative to EKF alone

**Important**: RTS is not a real-time causal estimator. It runs EKF forward first, then performs a backward correction pass using future measurements to refine earlier states.

### 6. MHE (Moving Horizon Estimation)
**Location**: `mhe/`

- **Type**: Sliding-window optimization-based estimator
- **State**: [px, py, v, yaw, yaw_rate]
- **Best For**: Constrained nonlinear estimation and future solver-based work
- **Complexity**: High
- **Performance**: Potentially very strong, but depends heavily on solver quality and tuning

**Current Status**: The class is created as a scaffold. It maintains a moving window, arrival-cost placeholders, and an EKF warm-start path, but it does not yet solve the nonlinear MHE optimization problem.

### 7. IMM (Interacting Multiple Model Filter)
**Location**: `imm/`

- **Type**: Multi-model adaptive filter
- **Models**: CV, CA, CTRV, CTRA (4 models)
- **State**: [px, py, v, yaw, yaw_rate] (fused output)
- **Best For**: Complex scenarios with varying motion patterns
- **Complexity**: High
- **Performance**: Best overall, especially in mixed scenarios

**Use Case**: When maximum accuracy is needed and computational resources allow. Automatically adapts to different driving behaviors.

See [imm/IMM_README.md](imm/IMM_README.md) for detailed IMM documentation.

## Performance Comparison

| Filter | RMSE | Computation | Scenarios | Adaptability | Tuning Needed |
|--------|------|-------------|-----------|--------------|---------------|
| **EKF** | Higher | Lowest | Simple | None | Low |
| **EKF + RTS** | Lower than EKF | Low forward + offline backward pass | Offline evaluation | None | Low |
| **MHE** | Placeholder until solver is added | Moderate | Sliding-window estimation | Constraints possible | High |
| **IEKF** | Medium | Low | Simple-Medium | None | Low |
| **UKF** | Medium | Medium | Standard | None | Medium (α,β,κ) |
| **CKF** | Medium | Medium | Standard | None | **None** |
| **IMM** | Lowest | Highest | Complex | Automatic | High |

## Usage in Code

### Highway Simulation (main.cpp → highway.h)

The filter choice is controlled in `highway.h`:

```cpp
// Use EKF instead of UKF (false = UKF, true = EKF)
bool use_ekf = true;  // Set to false for UKF

// Run offline RTS smoothing after the EKF simulation ends
bool use_ekf_rts = false; // Enabled by ekf_rts_highway

// Use MHE (Moving Horizon Estimation scaffold)
bool use_mhe = false; // Enabled by mhe_highway

// Use IEKF (Iterated Extended Kalman Filter)
bool use_iekf = false; // Set to true for IEKF

// Use CKF (Cubature Kalman Filter)
bool use_ckf = false; // Set to true for CKF
```

**Note**: RTS smoothing is only enabled in the dedicated `ekf_rts_highway` executable. It is an offline backward pass after the forward EKF run completes.

### Building

```bash
# Build all versions
cd build
cmake ..
make

# Run standard filter (EKF/UKF - controlled by use_ekf flag in highway.h)
./filters_highway

# Run Extended Kalman Filter
./ekf_highway

# Run EKF + RTS smoother
./ekf_rts_highway

# Run MHE scaffold
./mhe_highway

# Run Iterated Extended Kalman Filter
./iekf_highway

# Run Unscented Kalman Filter
./ukf_highway

# Run Cubature Kalman Filter
./ckf_highway

# Run Particle Filter
./pf_highway

# Run IMM (4-model adaptive filter)
./imm_highway
```

## When to Use Which Filter

### Use EKF when:
- ✅ Computational resources are very limited
- ✅ Motion is relatively smooth and predictable
- ✅ Quick prototyping is needed

### Use EKF + RTS when:
- ✅ You can process the full measurement sequence offline
- ✅ You want backward correction of earlier EKF estimates
- ✅ You care about post-run trajectory quality and RMSE
- ✅ Real-time output is not required

### Use MHE when:
- ✅ You want the codebase scaffold for future constrained estimation work
- ✅ You plan to add a solver-backed sliding-window estimator
- ✅ You want to test the integration path in the highway harness now
- ❌ You do not yet need true MHE performance, because the nonlinear solve is not implemented

### Use IEKF when:
- ✅ You want better accuracy than EKF without jumping to UKF
- ✅ Radar measurements are highly nonlinear
- ✅ You prefer linearization methods over sigma points
- ✅ Acceptable to run 2-3 iterations per radar measurement
- ✅ Good middle ground between EKF simplicity and UKF accuracy

### Use UKF when:
- ✅ Standard tracking with good accuracy is needed
- ✅ Dealing with nonlinear sensor models (radar)
- ✅ Computational resources are moderate
- ✅ You can tune parameters (α, β, κ) for optimal performance

### Use CKF when:
- ✅ You want UKF-level accuracy **without tuning parameters**
- ✅ Dealing with nonlinear systems (same as UKF)
- ✅ Prefer mathematical rigor (spherical-radial cubature theory)
- ✅ Simpler setup than UKF (no α, β, κ to tune)
- ✅ This is the best choice for "set it and forget it" nonlinear tracking

### Use IMM when:
- ✅ Maximum tracking accuracy is required
- ✅ Vehicles exhibit mixed behaviors (cruising, turning, accelerating)
- ✅ Computational resources are sufficient
- ✅ Urban or complex highway scenarios with frequent maneuvers
- ✅ RMSE must be minimized

## Implementation Details

All filters share:
- Common measurement package format
- Support for radar and lidar sensors
- RMSE evaluation via tools.cpp
- 3D visualization via render/

Each filter directory contains:
- Header file (.h or .hpp)
- Implementation file (.cpp)
- Filter-specific logic and parameters

## Related Directories

- **`../motion_models/`**: Motion model implementations used by IMM
- **`../render/`**: Visualization code (includes filter headers)
- **`../sensors/`**: Sensor data and lidar processing
- **`../tools.cpp`**: RMSE calculation and utilities

## Development

To add a new filter:
1. Create new directory under `filters/`
2. Implement with common interface (ProcessMeasurement, etc.)
3. Update CMakeLists.txt
4. Update render/render.h if needed
5. Add documentation

## RTS Smoother

### Rauch-Tung-Striebel (RTS) Smoother

The **RTS smoother** is fundamentally different from filters. It is a **backward smoothing algorithm** that processes data in two passes.

**Key Differences from Filters:**
- **Filters**: Process data **forward in time** (causal), providing estimates up to the current time
- **Smoothers**: Process data **backward in time** (non-causal), providing estimates for all past states using all available data

**How the current EKF RTS smoother works:**

1. **Forward Pass**:
   - Run EKF forward through all measurements
   - Store filtered states `x_filtered[k]`, filtered covariances `P_filtered[k]`
   - Store one-step predictions `x_pred[k+1]`, `P_pred[k+1]`
   - Store the EKF process Jacobian `F[k]`

2. **Backward Pass** (Smoothing):
   - Start from the last timestep and work backward
   - For each timestep k, compute smoother gain:
     ```
     C[k] = P_filtered[k] * F[k]^T * P_pred[k+1]^{-1}
     ```
   - Update smoothed estimate:
     ```
     x_smooth[k] = x_filtered[k] + C[k] * (x_smooth[k+1] - x_pred[k+1])
     P_smooth[k] = P_filtered[k] + C[k] * (P_smooth[k+1] - P_pred[k+1]) * C[k]^T
     ```

**Use Cases:**
- ✅ **Offline processing**: When you have all data and can process backward
- ✅ **Trajectory reconstruction**: Post-processing recorded sensor data
- ✅ **Maximum accuracy**: Uses future measurements to improve past estimates
- ✅ **Animation/visualization**: Smoothing noisy recorded trajectories
- ✅ **SLAM**: Simultaneous localization and mapping (loop closure)

**Limitations:**
- ❌ **Not real-time**: Requires all measurements before starting backward pass
- ❌ **Memory intensive**: Must store all forward pass results
- ❌ **Latency**: Cannot provide estimates until entire sequence is processed

**Current Integration:**
- `filters/rts/ekf_rts_smoother.*` implements the backward pass
- `ekf_rts_highway` enables EKF forward tracking plus post-run RTS smoothing
- `highway.h` prints per-car filtered RMSE and smoothed RMSE after the run completes

**Current Limitation:**
- the PCL viewer still shows the forward EKF during the live run
- the smoothed trajectory is currently reported after the sequence ends

**Likely Next Steps:**
1. Replay the smoothed trajectory in the viewer after the forward run
2. Add a fixed-lag smoother for delayed online visualization
3. Add UKF-based RTS using sigma-point cross-covariance
4. Add CKF-based smoothing

## References

- EKF Implementation: `ekf/ekf.hpp`, `ekf/ekf.cpp`
- UKF Implementation: `ukf/ukf.h`, `ukf/ukf.cpp`
- IMM Implementation: `imm/IMM.hpp`, `imm/IMM.cpp`
- Motion Models: `../motion_models/README.md`
