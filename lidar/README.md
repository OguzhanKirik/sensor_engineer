# Lidar Obstacle Detection

This repository covers the full pipeline for lidar-based obstacle detection, including point cloud generation, segmentation, clustering, and real-world data processing. The lessons and project are based on implementing core 3D perception techniques used in autonomous driving.

---


### **L1: Introduction to Lidar and Point Clouds**

### **L2: Point Cloud Segmentation**
Apply **RANSAC plane fitting** to classify points as either:
- Road surface  
- Obstacles  
This lesson covers robust model fitting and separating inliers from outliers in noisy data.

---

### **L3: Clustering Obstacles**
Perform **Euclidean clustering** to group obstacle points into individual objects.  
Learn to build and use **KD-Trees** for accelerated nearest-neighbor search to make clustering efficient and scalable.

---

### **L4: Working with Real PCD**
Apply your full pipeline to real lidar point cloud data (.pcd files).  
You will visualize real point clouds and run segmentation + clustering on real sensor data.

---

### **L5: Project — Lidar Obstacle Detection**
Combine everything learned to build a complete 3D obstacle detection pipeline:
- Load real or simulated point clouds  
- Segment the road using RANSAC  
- Cluster obstacles using KD-Tree–based Euclidean clustering  
- Visualize bounding boxes around detected objects  



## 🧰 Requirements
- C++17  
- PCL (Point Cloud Library)  
- Open3D or visualization library (if desired)  
- CMake build system  


