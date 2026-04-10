# IMM Filter Implementation with 4 Motion Models

This directory contains the Interacting Multiple Model (IMM) filter implementation with **four motion models** for comprehensive object tracking.

## Files

### IMM Core
- **IMM.hpp / IMM.cpp**: Main IMM filter implementation
- **cv_ekf.h / cv_ekf.cpp**: Constant Velocity (CV) model using Extended Kalman Filter
- **ca_ekf.h / ca_ekf.cpp**: Constant Acceleration (CA) model using Extended Kalman Filter
- **ctrv_ukf.h / ctrv_ukf.cpp**: Constant Turn Rate and Velocity (CTRV) model using Unscented Kalman Filter
- **ctra_ukf.h / ctra_ukf.cpp**: Constant Turn Rate and Acceleration (CTRA) model using Unscented Kalman Filter

## Motion Models

### 1. **CV (Constant Velocity)** 
- **State**: [px, py, vx, vy]
- **Best For**: Straight-line motion, cruising at constant speed
- **Filter Type**: EKF (Extended Kalman Filter)

### 2. **CA (Constant Acceleration)**
- **State**: [px, py, vx, vy, ax, ay]
- **Best For**: Accelerating or decelerating in straight line
- **Filter Type**: EKF (Extended Kalman Filter)

### 3. **CTRV (Constant Turn Rate and Velocity)**
- **State**: [px, py, v, yaw, yaw_rate]
- **Best For**: Turning at steady speed (lane changes)
- **Filter Type**: UKF (Unscented Kalman Filter)

### 4. **CTRA (Constant Turn Rate and Acceleration)**
- **State**: [px, py, v, yaw, yaw_rate, a]
- **Best For**: Turning while accelerating/decelerating
- **Filter Type**: UKF (Unscented Kalman Filter)

## How IMM Works

The IMM filter combines all four motion models to handle diverse vehicle behaviors:

```
┌─────────┐  ┌─────────┐  ┌──────────┐  ┌──────────┐
│   CV    │  │   CA    │  │   CTRV   │  │   CTRA   │
│ Straight│  │Accel/Dec│  │  Turning │  │Turn+Accel│
└────┬────┘  └────┬────┘  └────┬─────┘  └────┬─────┘
     │            │             │             │
     └────────────┴─────────────┴─────────────┘
                      │
               Weighted Fusion
       (μ₁×CV + μ₂×CA + μ₃×CTRV + μ₄×CTRA)
                      │
              Final Estimate
```

### Algorithm Steps

```
For each measurement:
1. Mix States: Blend previous estimates based on mode transition probabilities
2. Predict: Each of 4 models predicts independently
3. Update: Each model updates with measurement
4. Calculate Likelihoods: Determine which model fits best
5. Update Probabilities: Calculate weight for each model
6. Fuse: Combine estimates weighted by model probabilities
```

### Mode Transition Matrix (4x4)

```cpp
    //      To: CV    CA    CTRV  CTRA
PI_ << 0.70, 0.15, 0.10, 0.05,  // From CV
       0.15, 0.70, 0.05, 0.10,  // From CA
       0.10, 0.05, 0.70, 0.15,  // From CTRV
       0.05, 0.10, 0.15, 0.70;  // From CTRA
```

**Structure**:
- Diagonal (0.70): High probability of staying in current model
- Adjacent similar models (0.15): CV↔CA, CTRV↔CTRA  
- Opposite models (0.05-0.10): Lower transition probability

## Build Instructions

```bash
cd build
cmake ..
make

# Run IMM filter with 4 models
./imm_highway
```

## Key Advantages

✅ **Highly Adaptive**: Handles all common driving scenarios  
✅ **Robust**: Automatically switches between models  
✅ **Best Performance**: Lowest RMSE in complex scenarios  
✅ **Comprehensive**: Covers straight, accelerating, turning, and combined motions

## Model Selection Examples

| Scenario | Dominant Model | Why |
|----------|---------------|-----|
| Highway cruising | **CV** | Constant velocity, straight |
| Approaching traffic | **CA** | Decelerating |
| Lane change | **CTRV** | Turning at steady speed |
| Merging onto highway | **CTRA** | Turning + accelerating |
| Stopping at light | **CA** | Strong deceleration |

## Example Output

```
IMM Probs: CV=0.85 CA=0.08 CTRV=0.05 CTRA=0.02   // Cruising straight
IMM Probs: CV=0.10 CA=0.70 CTRV=0.10 CTRA=0.10   // Braking
IMM Probs: CV=0.05 CA=0.05 CTRV=0.80 CTRA=0.10   // Lane change
IMM Probs: CV=0.03 CA=0.05 CTRV=0.15 CTRA=0.77   // Turning + accelerating
```

## Tuning Parameters

### Mode Transition Matrix
- **Diagonal (0.60-0.80)**: Stay in current model
- **Similar models (0.10-0.20)**: CV↔CA, CTRV↔CTRA
- **Different models (0.03-0.10)**: Cross transitions

### Initial Probabilities
```cpp
mu_ << 0.25, 0.25, 0.25, 0.25;  // Equal start
// Or bias towards expected behavior:
mu_ << 0.40, 0.20, 0.30, 0.10;  // Favor CV and CTRV for highway
```

### Process Noise by Model
- **CV**: Low (std_a = 1.0)
- **CA**: Very low (std_a = 0.5)  
- **CTRV**: Medium (std_a = 2.0, std_yawdd = 0.6)
- **CTRA**: Medium-High (std_a = 1.5, std_yawdd = 0.6)

## Performance Comparison

| Filter | RMSE | Scenarios | Complexity |
|--------|------|-----------|------------|
| EKF | High | Simple | Low |
| UKF | Medium | Moderate | Medium |
| IMM-2 | Low | Mixed | Medium-High |
| **IMM-4** | **Lowest** | **All** | **High** |

## Implementation Details

### State Conversions
All models internally convert to/from common 5D state:
```
[px, py, v, yaw, yaw_rate]
```

This allows seamless mixing and fusion.

### Likelihood Calculation
Each model calculates how well it explains the measurement:
```cpp
L = exp(-0.5 * y^T * S^-1 * y) / sqrt(det(2πS))
```

Higher likelihood → Higher model probability

## When to Use

- ✅ **Complex scenarios** with mixed driving behaviors
- ✅ **Urban environments** with frequent maneuvers
- ✅ **Highway** with lane changes and speed variations  
- ✅ **Performance critical** applications (RMSE matters)

Use simpler filter if:
- ❌ Computational resources very limited
- ❌ Motion is truly constant (CV or CTRV only)
- ❌ Real-time constraints prevent 4-model execution
