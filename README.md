# Sensor Fusion, Camera, Radar, and Lidar — Complete Course Summary

This repository consolidates key concepts, lessons, and project summaries from a multi-module program covering **Lidar**, **Radar**, **Computer Vision**, **Kalman Filters**, and **Sensor Fusion**.  
Each section provides foundational theory paired with practical implementations in C++, Python, and MATLAB.

---

# ** Overview**

## 1. **Lidar Obstacle Detection **

### **L1: Introduction to Lidar & Point Clouds**
- Learn the basics of lidar technology and point cloud data.
- Use a simulated highway environment to generate and visualize point clouds.

### **L2: Point Cloud Segmentation**
- Apply RANSAC plane fitting to segment road points from obstacles.

### **L3: Clustering Obstacles**
- Use KD-Trees and Euclidean clustering for fast obstacle grouping.

### **L4: Working with Real PCD**
- Run your full pipeline on real-world point cloud data.

### **L5 Project: Lidar Obstacle Detection**
- Build a complete detection pipeline: segmentation → clustering → bounding boxes.

---

## 2. **Radar Signal Processing**

### **L1: Introduction**
- Overview of radar systems and their applications.

### **L2: Radar Principles Review**
- FMCW radar fundamentals  
- Hardware & schematics  
- Radar equation and signal power relationships  

### **L3: Range–Doppler Estimation**
- Estimate range and velocity using FFT and Doppler processing.

### **L4: Clutter, CFAR, AoA & Clustering**
- Learn clutter formation and removal using CFAR  
- Estimate Angle of Arrival (AoA)  
- Perform clustering on radar detections  

---

## 3. **Computer Vision & TTC Estimation**

### **L1: Autonomous Vehicles & Computer Vision**
- Levels of autonomy  
- Typical sensor suites  
- Camera fundamentals  
- Intro to OpenCV  

### **L2: Engineering a Collision Detection System**
- Compute **Time-to-Collision (TTC)** using lidar and camera.

### **L3: Tracking Image Features**
- Learn image gradients, filtering, corner detection, feature tracking.

### **L4 Project: 2D Feature Tracking**
- Build a complete 2D feature-tracking pipeline in OpenCV.

---

## 4. **Camera + Lidar Sensor Fusion**

### **L5: Combining Camera and Lidar**
- Fuse 2D image data with 3D lidar points for improved robustness.

### **L6 Project: Track an Object in 3D Space**
- Build a multi-sensor fusion tracker to estimate TTC and motion in 3D.

---

## 5. **Kalman Filters & Real-Time Tracking**

### **L1: Kalman Filters (Python)**
- Learn from Sebastian Thrun  
- Understand prediction & update cycles  
- Implement a Kalman Filter in Python  

### **L2: Lidar & Radar Fusion with Kalman Filters (C++)**
- Implement a high-performance **Extended Kalman Filter (EKF)** in C++  
- Fuse radar and lidar measurements for real-time tracking  

---

## 6. **Unscented Kalman Filter (UKF)**

### **L3: Unscented Kalman Filters**
- Overcome EKF limitations with nonlinear motion  
- Use sigma points & the Unscented Transform  
- Apply UKF to lidar/radar fusion  

### **L4 Project: UKF Highway Object Tracking**
- Build a complete UKF tracking pipeline for multiple vehicles on a highway.

---

# 📡 7. **1D & 2D CFAR Detection (Radar Filtering)**

### **1D CFAR**
Includes MATLAB tools for:
- FMCW radar generation  
- Range/Doppler detection  
- Frequency estimation  
- Maximum range computation  

### **2D CFAR Algorithm Summary**
1. Select training + guard cells  
2. Convert training cells from dB → linear  
3. Exclude guard cells & CUT  
4. Compute average noise, convert back to dB, add offset  
5. Compare CUT to threshold  
6. Mark detections  

---

# 🧰 **Technologies & Tools Used**
- **C++17**, **Python**, **MATLAB**
- **Point Cloud Library (PCL)**  
- **OpenCV**  
- **Udacity Lidar & Radar simulators**  
- **FMCW Radar models**  
- **Kalman Filters (KF, EKF, UKF)**  
- **KD-Tree data structures**  

---
> **Note:** This project uses the official **Udacity Fixed-Wing Simulator** and includes **partial control code** intended for educational and practice purposes.
 

---
