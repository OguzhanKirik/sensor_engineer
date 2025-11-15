# 1D CFAR Practice

This repository contains MATLAB scripts designed for practicing and understanding radar signal processing concepts, including target generation, detection, Doppler estimation, and range calculations. The scripts focus on building a foundation for FMCW radar analysis and implementing detection pipelines commonly used in automotive and remote-sensing radar systems.

---

## 📁 Files Included

### **1. `Radar_Target_Generation_and_Detection.m`**
Generates FMCW radar signals, simulates target motion, and performs target detection using range and Doppler FFT processing.  
Includes 1D CFAR thresholding for peak detection.

### **2. `doppler_estimation.m`**
Implements methods for estimating Doppler shift in received radar signals.  
Used to determine **target radial velocity** based on frequency changes.

### **3. `estimate_frequency_shifts.m`**
Calculates frequency shifts present in the radar return signal.  
Useful for analyzing **beat frequencies**, Doppler effects, and frequency-modulated signals.

### **4. `estimate_max_range.m`**
Computes the maximum detectable range of an FMCW radar system using system parameters such as sweep bandwidth, chirp duration, and sampling rate.

---

## 🚀 Overview

This repository provides simple, modular MATLAB scripts to help users:

- Understand FMCW radar signal generation  
- Perform FFT-based range and velocity estimation  
- Implement 1D CFAR detection  
- Analyze radar performance limits using theoretical models  

These scripts are suitable for self-study, radar coursework, or early-stage radar algorithm prototyping.

---

## 📌 Requirements

- MATLAB R2018a or later  
- Signal Processing Toolbox (recommended)


