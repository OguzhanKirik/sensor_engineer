# Motion Models

This directory contains the four motion model implementations used by the IMM (Interacting Multiple Model) filter.

## Models

### 1. CV (Constant Velocity) - `cv_ekf.*`
- **Filter Type**: Extended Kalman Filter (EKF)
- **State Dimension**: 4 [px, py, vx, vy]
- **Use Case**: Straight-line motion at constant speed
- **Best For**: Highway cruising, steady-state driving

### 2. CA (Constant Acceleration) - `ca_ekf.*`
- **Filter Type**: Extended Kalman Filter (EKF)
- **State Dimension**: 6 [px, py, vx, vy, ax, ay]
- **Use Case**: Accelerating or decelerating motion
- **Best For**: Braking, accelerating, approaching intersections

### 3. CTRV (Constant Turn Rate and Velocity) - `ctrv_ukf.*`
- **Filter Type**: Unscented Kalman Filter (UKF)
- **State Dimension**: 5 [px, py, v, yaw, yaw_rate]
- **Use Case**: Turning at constant speed
- **Best For**: Lane changes, steady cornering

### 4. CTRA (Constant Turn Rate and Acceleration) - `ctra_ukf.*`
- **Filter Type**: Unscented Kalman Filter (UKF)
- **State Dimension**: 6 [px, py, v, yaw, yaw_rate, a]
- **Use Case**: Turning while accelerating/decelerating
- **Best For**: Highway merging, complex maneuvers

## File Structure

```
motion_models/
├── cv_ekf.h       - CV model header
├── cv_ekf.cpp     - CV model implementation
├── ca_ekf.h       - CA model header
├── ca_ekf.cpp     - CA model implementation
├── ctrv_ukf.h     - CTRV model header
├── ctrv_ukf.cpp   - CTRV model implementation
├── ctra_ukf.h     - CTRA model header
└── ctra_ukf.cpp   - CTRA model implementation
```

## Common Interface

All models implement a common interface:

```cpp
void Initialize(const VectorXd& x_in, const MatrixXd& P_in);
void Predict(double dt);
void Update(const MeasurementPackage& meas_package);
double CalculateLikelihood(const MeasurementPackage& meas_package);
VectorXd GetState5D() const;
void SetStateFrom5D(const VectorXd& x_5d);
```

This allows the IMM filter to treat all models uniformly.

## State Conversion

Each model internally manages its own state representation but provides conversion to/from a common 5D state format: `[px, py, v, yaw, yaw_rate]`. This enables seamless state mixing and fusion in the IMM algorithm.

## Usage

These models are not meant to be used directly. They are instantiated and managed by the IMM filter in `../IMM.cpp`.

For direct usage examples, see `../IMM_README.md`.
