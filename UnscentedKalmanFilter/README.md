# Unscented_Kalman_Filter
Sensor Fusion UKF Highway Project Starter Code

<img src="media/ukf_highway_tracked.gif" width="700" height="400" />


# 🚗 Unscented Kalman Filters — Nonlinear Object Tracking

This project implements an **Unscented Kalman Filter (UKF)** to track multiple vehicles in a simulated traffic environment using fused **radar and lidar measurements**. The vehicles follow a **CTRV (Constant Turn Rate and Velocity)** motion model, and the filter is robust to noisy measurements and nonlinear motion.

---

## 🎯 Project Overview

- **Nonlinear object tracking** using the UKF  
- **Fusion of noisy radar and lidar sensor data**  
- **Tracking multiple moving objects** on a simulated highway  
- **Handles nonlinear motion and measurement spaces**  
- **Real-time RMSE evaluation** for accuracy assessment  

In the simulation, each vehicle’s ground truth is compared against the estimated state.  
The **RMSE (Root Mean Square Error)** is computed across all vehicles, affecting global accuracy if one object performs poorly.  
The displayed RMSE values correspond to:

1. **X position**
2. **Y position**
3. **Vx velocity**
4. **Vy velocity**

---

## 📦 Dependencies

Required versions:

- **cmake ≥ 3.5**
  - All OS: https://cmake.org/install/
- **make ≥ 4.1 (Linux/Mac) OR 3.81 (Windows)**
  - Linux: usually preinstalled
  - Mac: install Xcode Command Line Tools  
    https://developer.apple.com/xcode/features/
  - Windows: http://gnuwin32.sourceforge.net/packages/make.htm
- **gcc/g++ ≥ 5.4**
  - Linux: typically preinstalled
  - Mac: via Xcode Command Line Tools
  - Windows: use MinGW: http://www.mingw.org/
- **PCL ≥ 1.2**

---

## 🧭 How to Build & Run

From the project root:

```bash
mkdir build
cd build
cmake ..
make
./ukf_highway





