# Sensor Fusion Workspace

This workspace contains several C++ projects related to sensor fusion, perception, tracking, and control. The folders are not one combined build; each project should be treated as its own codebase with its own dependencies, build flow, and goals.

## Workspace Structure

```text
.
├── Camera/
├── FCND-Controls-CPP/
├── ObstacleDetection-Lidar/
├── QuadrotorStateEstimation/
├── Radar/
├── SensorFusion-CameraLidar/
├── SensorFusion-RadarLidar/
└── README.md
```

## Main Projects

### QuadrotorStateEstimation

`QuadrotorStateEstimation` is a simulator-based state estimation project for a quadrotor.

It focuses on:
- IMU-based attitude estimation
- EKF prediction for position, velocity, and yaw
- magnetometer fusion
- GPS position and velocity updates
- covariance tuning and estimator consistency checks

Project documentation:
- [QuadrotorStateEstimation/README.md](/Users/oguz/Desktop/workspace_cpp/Filters/QuadrotorStateEstimation/README.md)

### SensorFusion-CameraLidar

`SensorFusion-CameraLidar` is a camera-lidar fusion project for 3D object tracking and time-to-collision estimation.

It combines:
- YOLO-based 2D object detection
- lidar point projection into the image
- ROI-based lidar association
- keypoint matching across frames
- bounding-box tracking over time
- TTC estimation from lidar and camera

Core files:
- [SensorFusion-CameraLidar/src/cameraLidar.cpp](/Users/oguz/Desktop/workspace_cpp/Filters/SensorFusion-CameraLidar/src/cameraLidar.cpp)
- [SensorFusion-CameraLidar/src/cam_Fusion.cpp](/Users/oguz/Desktop/workspace_cpp/Filters/SensorFusion-CameraLidar/src/cam_Fusion.cpp)
- [SensorFusion-CameraLidar/README.md](/Users/oguz/Desktop/workspace_cpp/Filters/SensorFusion-CameraLidar/README.md)

### SensorFusion-RadarLidar

`SensorFusion-RadarLidar` is a radar-lidar fusion project built around tracked-object state estimation.

It is the right project if you want:
- multi-sensor tracking
- Kalman-filter-based fusion
- RMSE-style evaluation of tracked motion

### ObstacleDetection-Lidar

`ObstacleDetection-Lidar` focuses on lidar point-cloud perception.

Typical work here includes:
- point-cloud filtering
- segmentation
- clustering
- obstacle extraction
- geometric processing in 3D

### FCND-Controls-CPP

`FCND-Controls-CPP` is a controls-focused C++ project related to quadrotor control and flight dynamics.

This is the right area if your goal is:
- controller implementation
- trajectory tracking
- flight-control tuning

## Domain Folders

### Camera

`Camera` contains camera-based exercises and supporting perception projects.

Typical work in this area includes:
- feature detection and matching
- image-based tracking
- calibration exercises
- camera-lidar fusion coursework and reference implementations

### Radar

`Radar` contains radar-related signal-processing exercises and detection workflows.

Typical work in this area includes:
- radar target generation
- FFT and Doppler processing
- CFAR concepts
- radar-specific measurement interpretation

## How To Choose A Project

Use `QuadrotorStateEstimation` if you want:
- simulator-based EKF work
- IMU/GPS/magnetometer fusion
- estimator tuning with known ground truth

Use `SensorFusion-CameraLidar` if you want:
- camera-lidar perception fusion
- 3D object association in image space
- TTC estimation from two sensing modalities

Use `SensorFusion-RadarLidar` if you want:
- radar-lidar target tracking
- Kalman-filter-style multi-sensor fusion
- tracked-object evaluation workflows

Use `ObstacleDetection-Lidar` if you want:
- lidar-only perception
- point-cloud segmentation and clustering

Use `FCND-Controls-CPP` if you want:
- control-system implementation
- flight dynamics and controller tuning

Use `Camera` or `Radar` if you want:
- modality-specific exercises
- supporting material and smaller focused projects

## Build Notes

There is no single workspace-wide build. Build each project inside its own directory.

Typical pattern:

```bash
mkdir -p build
cd build
cmake ..
cmake --build . -j2
```

Project-specific dependencies vary, but common requirements across the workspace include:
- CMake
- a modern C++ compiler
- Eigen

Additional dependencies depend on the project:
- `QuadrotorStateEstimation`: Qt5, OpenGL, GLUT
- `SensorFusion-CameraLidar`: OpenCV, Git LFS for model assets
- `SensorFusion-RadarLidar`: Eigen and project-specific fusion dependencies
- `ObstacleDetection-Lidar`: typically PCL
