# Filters Directory

This directory contains three different Kalman filter implementations for object tracking.

## Directory Structure

```
filters/
├── ekf/                    - Extended Kalman Filter
│   ├── ekf.hpp
│   └── ekf.cpp
├── ukf/                    - Unscented Kalman Filter
│   ├── ukf.h
│   └── ukf.cpp
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

### 2. UKF (Unscented Kalman Filter)
**Location**: `ukf/`

- **Type**: Sigma points-based filter
- **State**: [px, py, v, yaw, yaw_rate]
- **Motion Model**: CTRV (Constant Turn Rate and Velocity)
- **Best For**: Nonlinear systems with good accuracy requirements
- **Complexity**: Medium
- **Performance**: Better than EKF for nonlinear motion

**Use Case**: Standard choice for most tracking applications with radar/lidar fusion.

### 3. IMM (Interacting Multiple Model Filter)
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

| Filter | RMSE | Computation | Scenarios | Adaptability |
|--------|------|-------------|-----------|--------------|
| **EKF** | Higher | Lowest | Simple | None |
| **UKF** | Medium | Medium | Standard | None |
| **IMM** | Lowest | Highest | Complex | Automatic |

## Usage in Code

### Highway Simulation (main.cpp → highway.h)

The filter choice is controlled in `highway.h`:

```cpp
// Use EKF instead of UKF (false = UKF, true = EKF)
bool use_ekf = true;  // Set to false for UKF
```

### Building

```bash
# Build EKF/UKF version
cd build
cmake ..
make
./filters_highway

# Build IMM version
./imm_highway
```

## When to Use Which Filter

### Use EKF when:
- ✅ Computational resources are very limited
- ✅ Motion is relatively smooth and predictable
- ✅ Quick prototyping is needed

### Use UKF when:
- ✅ Standard tracking with good accuracy is needed
- ✅ Dealing with nonlinear sensor models (radar)
- ✅ Computational resources are moderate
- ✅ This is the default choice for most applications

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

## References

- EKF Implementation: `ekf/ekf.hpp`, `ekf/ekf.cpp`
- UKF Implementation: `ukf/ukf.h`, `ukf/ukf.cpp`
- IMM Implementation: `imm/IMM.hpp`, `imm/IMM.cpp`
- Motion Models: `../motion_models/README.md`
