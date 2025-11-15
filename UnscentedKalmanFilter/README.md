# Unscented_Kalman_Filter
Sensor Fusion UKF Highway Project Starter Code

<img src="media/ukf_highway_tracked.gif" width="700" height="400" />


## Unscented Kalman Filters — Nonlinear Object Tracking

### **1: Unscented Kalman Filters (UKF)**
While Extended Kalman Filters work well for systems with mostly linear motion, real-world objects frequently move in nonlinear patterns.  
The **Unscented Kalman Filter (UKF)** uses sigma-point sampling to accurately track motion in highly nonlinear systems.  
In this lesson, you will learn:
- Why EKFs struggle with nonlinear dynamics  
- How the Unscented Transform works  
- How to generate and propagate sigma points  
- How to apply UKF for sensor fusion with radar and lidar measurements  
UKF provides significantly improved accuracy in complex motion scenarios.

---

### **L2 Project: Unscented Kalman Filter Highway Project**
In this capstone project, students implement a full UKF pipeline capable of:
- Processing noisy radar and lidar data  
- Tracking multiple objects on a simulated highway  
- Handling nonlinear motion and measurement models  


