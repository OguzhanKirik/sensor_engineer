# CKF (Cubature Kalman Filter) Implementation

## Overview

The Cubature Kalman Filter (CKF) is a nonlinear Gaussian filter that uses the **spherical-radial cubature rule** for numerical integration. It's similar in spirit to the Unscented Kalman Filter (UKF) but with a more principled mathematical foundation based on cubature theory.

## Key Differences: CKF vs UKF

| Aspect | UKF | CKF |
|--------|-----|-----|
| **Sigma Points** | 2n+1 points | 2n points |
| **Weights** | Tunable (α, β, κ) | Equal weights (1/2n) |
| **Tuning** | Requires parameter tuning | Parameter-free |
| **Theory** | Unscented transform | Spherical-radial cubature |
| **Accuracy** | 3rd order for Gaussian | 3rd order for Gaussian |
| **Complexity** | Slightly higher | Slightly lower |

**Key Advantage**: CKF has **no tuning parameters**, making it more robust and easier to use than UKF.

---

## Algorithm

### Cubature Point Generation

For an n-dimensional state with mean **x** and covariance **P**:

```
1. Compute matrix square root: A = chol(P)
2. Generate 2n cubature points:
   ξᵢ = x + √n * Aᵢ     for i = 1, ..., n
   ξᵢ = x - √n * Aᵢ₋ₙ   for i = n+1, ..., 2n
3. All weights are equal: wᵢ = 1/(2n)
```

Where:
- **n** = state dimension (5 for CTRV model: px, py, v, yaw, yaw_rate)
- **2n** = 10 cubature points total
- **√n** = scaling factor (√5 ≈ 2.236)
- **Aᵢ** = i-th column of Cholesky decomposition of P

### Prediction Step

```cpp
For each cubature point ξᵢ:
    1. Propagate through process model: χᵢ = f(ξᵢ, Δt)

Compute predicted mean:
    x̄ = (1/2n) ∑ χᵢ

Compute predicted covariance:
   P̄ = (1/2n) ∑ (χᵢ - x̄)(χᵢ - x̄)ᵀ + Q

Where Q is process noise covariance
```

### Update Step

```cpp
For each predicted cubature point χᵢ:
    1. Transform to measurement space: Zᵢ = h(χᵢ)

Compute predicted measurement:
    ẑ = (1/2n) ∑ Zᵢ

Compute innovation covariance:
    S = (1/2n) ∑ (Zᵢ - ẑ)(Zᵢ - ẑ)ᵀ + R

Compute cross-covariance:
    Pxz = (1/2n) ∑ (χᵢ - x̄)(Zᵢ - ẑ)ᵀ

Compute Kalman gain:
    K = Pxz * S⁻¹

Update state and covariance:
    x = x̄ + K(z - ẑ)
    P = P̄ - K * S * Kᵀ
```

---

## Implementation Details

### State Vector

CTRV motion model (Constant Turn Rate and Velocity):

```
x = [px, py, v, yaw, yaw_rate]ᵀ
```

- **px, py**: Position in meters
- **v**: Velocity magnitude in m/s
- **yaw**: Heading angle in radians
- **yaw_rate**: Turn rate in rad/s

### Process Model (CTRV)

```cpp
if (|yaw_rate| > 0.001):
    px' = px + (v / yaw_rate) * [sin(yaw + yaw_rate*dt) - sin(yaw)]
    py' = py + (v / yaw_rate) * [cos(yaw) - cos(yaw + yaw_rate*dt)]
else:  // Straight motion
    px' = px + v * cos(yaw) * dt
    py' = py + v * sin(yaw) * dt

v' = v
yaw' = yaw + yaw_rate * dt
yaw_rate' = yaw_rate
```

### Measurement Models

**Lidar** (2D position):
```
z = [px, py]ᵀ
```

**Radar** (range, bearing, radial velocity):
```
ρ = √(px² + py²)
φ = atan2(py, px)
ρ̇ = (px*vx + py*vy) / ρ
where vx = v*cos(yaw), vy = v*sin(yaw)
```

---

## Parameter Tuning

### Process Noise

Located in `ckf.cpp` constructor:

```cpp
// Longitudinal acceleration noise (m/s²)
std_a_ = 1.5;

// Yaw acceleration noise (rad/s²)
std_yawdd_ = 0.57;
```

**Tuning Guidelines**:
- **Increase** if filter is too sluggish (RMSE high, slow adaptation)
- **Decrease** if filter is too noisy (oscillations, instability)

| Scenario | std_a_ | std_yawdd_ |
|----------|--------|------------|
| **Smooth highway** | 0.5-1.0 | 0.3-0.5 |
| **Normal driving** | 1.0-2.0 | 0.5-0.8 |
| **Aggressive** | 2.0-3.0 | 0.8-1.2 |

### Measurement Noise

```cpp
// Lidar noise (meters)
std_laspx_ = 0.15;
std_laspy_ = 0.15;

// Radar noise
std_radr_ = 0.3;       // Range (m)
std_radphi_ = 0.03;    // Bearing (rad)
std_radrd_ = 0.3;      // Radial velocity (m/s)
```

These are typically **fixed** based on sensor specifications.

---

## Performance Characteristics

### Computational Complexity

- **Cubature points**: 2n = 10 (vs UKF's 2n+1 = 11)
- **Matrix operations**: Similar to UKF
- **Cholesky decomposition**: O(n³) per prediction/update
- **Overall**: Comparable to UKF, slightly faster

### Accuracy

- **Gaussian approximation**: 3rd orderfor Gaussian distributions
- **Nonlinear systems**: Excellent performance
- **Compared to UKF**: Similar accuracy, more consistent (no parameter tuning needed)
- **Compared to EKF**: Much better for highly nonlinear systems

### When to Use CKF

✅ **Use CKF when**:
- You want UKF-like performance without parameter tuning
- Nonlinear process/measurement models (turning, accelerating vehicles)
- Prefer mathematical rigor (cubature theory)
- Want simpler implementation than UKF

❌ **Don't use CKF when**:
- System is nearly linear (use EKF for efficiency)
- You need the absolute best performance and can tune UKF parameters
- Computational resources are extremely limited (use EKF)

---

## Code Structure

### Files

- **`ckf.h`**: CKF class declaration
- **`ckf.cpp`**: CKF implementation

### Key Methods

```cpp
// Initialization
void Initialize(const VectorXd& x, const MatrixXd& P);

// Main processing
void ProcessMeasurement(const MeasurementPackage& meas_package);

// Core algorithm
void Predict(double delta_t);
void UpdateLidar(const MeasurementPackage& meas_package);
void UpdateRadar(const MeasurementPackage& meas_package);

// Internal helpers
MatrixXd GenerateCubaturePoints(const VectorXd& x, const MatrixXd& P);
void PredictCubaturePoints(const MatrixXd& Xsig, double delta_t);
VectorXd ProcessModel(const VectorXd& x, double dt, double nu_a, double nu_yawdd);
```

---

## Mathematical Foundation

### Spherical-Radial Cubature Rule

The CKF is based on the third-degree spherical-radial cubature rule for computing integrals of the form:

```
I(f) = ∫ f(x) N(x; μ, Σ) dx
```

Where N(x; μ, Σ) is a Gaussian density. The cubature rule approximates this as:

```
I(f) ≈ ∑ᵢ wᵢ f(ξᵢ)
```

With cubature points ξᵢ and equal weights wᵢ = 1/(2n).

### Properties

1. **Exactness**: Exact for polynomials up to degree 3
2. **Symmetry**: Points are symmetrically distributed
3. **Equal weights**: All cubature points equally weighted
4. **Minimal point set**: Uses 2n points (optimal for 3rd degree)

---

## Comparison with Other Filters

| Filter | RMSE (Position) | RMSE (Velocity) | Tuning Needed | Complexity |
|--------|----------------|-----------------|---------------|------------|
| **EKF** | 0.08-0.12 m | 0.50-0.70 m/s | Low | Low |
| **UKF** | 0.06-0.09 m | 0.40-0.55 m/s | Medium | Medium |
| **CKF** | 0.06-0.09 m | 0.40-0.55 m/s | **None** | Medium |
| **IMM** | 0.05-0.08 m | 0.35-0.50 m/s | High | High |

**CKF Sweet Spot**: Similar accuracy to UKF without the tuning headache!

---

## References

1. **Original Paper**: Arasaratnam, I., & Haykin, S. (2009). "Cubature Kalman Filters". IEEE Transactions on Automatic Control.

2. **Cubature Theory**: Jia, B., Xin, M., & Cheng, Y. (2013). "High-degree cubature Kalman filter".

3. **Comparison Study**: Wu, Y., Hu, D., Wu, M., & Hu, X. (2006). "A numerical-integration perspective on Gaussian filters".

---

## Example Usage

```cpp
#include "ckf.h"

// Create filter
CKF ckf;

// Initialize with first measurement
VectorXd x_init(5);
x_init << 1.0, 2.0, 5.0, 0.0, 0.0;  // px, py, v, yaw, yaw_rate
MatrixXd P_init = MatrixXd::Identity(5, 5);
ckf.Initialize(x_init, P_init);

// Process measurements
for (auto& meas : measurements) {
    ckf.ProcessMeasurement(meas);
    
    // Get state estimate
    VectorXd x = ckf.x_;
    MatrixXd P = ckf.P_;
}
```

---

## Troubleshooting

### High RMSE Values

**Problem**: Position RMSE > 0.5m

**Solutions**:
1. Check process noise: Try std_a_ = 1.5-2.5
2. Check initialization: Ensure P_init is reasonable
3. Verify measurement noise matches sensor specs

### Filter Divergence

**Problem**: Covariance P grows unbounded

**Solutions**:
1. Increase process noise (more conservative)
2. Check for numerical issues in Cholesky decomposition
3. Add small regularization to P (e.g., P += 1e-6 * I)

### Noisy Estimates

**Problem**: State jumps between measurements

**Solutions**:
1. Decrease process noise
2. Check if measurement noise is too low
3. Verify sensor data quality

---

## Future Improvements

1. **Square-Root CKF**: Improved numerical stability
2. **Adaptive noise**: Automatically tune process noise
3. **Higher-degree cubature**: 5th or 7th degree rules
4. **Constrained CKF**: Handle state constraints (e.g., positive velocity)

