# Sensor Fusion Workspace

This workspace contains two main C++ estimation projects:

- `QuadrotorStateEstimation`
- `SensorFusion-ObjectTracking`

It also includes domain folders for related sensor-fusion work:

- `Camera`
- `Lidar`
- `Radar`

These parts of the workspace cover different sensor modalities and project types, so they should be read as separate areas rather than one single build target.

## Workspace Structure

```text
.
├── Camera/
├── Lidar/
├── QuadrotorStateEstimation/
├── Radar/
├── SensorFusion-ObjectTracking/
└── README.md
```

Local-only note:
- `QuadrotorStateEstimation.git-backup/` is not part of the workspace structure. It is only a local backup of the old nested `.git` directory and should be removed when no longer needed.

## QuadrotorStateEstimation

`QuadrotorStateEstimation` is a quadrotor state estimation simulator project based on a partially implemented EKF pipeline.

It focuses on:
- IMU-based attitude estimation
- EKF prediction for position, velocity, and yaw
- magnetometer yaw correction
- GPS position and velocity correction
- covariance tuning and estimator consistency checks in simulation

The simulator provides ground truth, which makes it useful for:
- validating estimator accuracy
- checking sigma consistency against true error
- tuning process and measurement noise
- understanding the full predict/update flow of an EKF in a controlled environment

The project is organized around simulation scenarios, especially:
- `06_SensorNoise`
- `07_AttitudeEstimation`
- `08_PredictState`
- `09_PredictCovariance`
- `10_MagUpdate`
- `11_GPSUpdate`

Build pattern:

```bash
cd QuadrotorStateEstimation
mkdir -p build
cd build
cmake ..
cmake --build . -j2
./CPPEstSim
```

Notes:
- This project uses Qt5, OpenGL, and GLUT.
- On macOS, you may need to pass `CMAKE_PREFIX_PATH` so CMake can find `qt@5`.

Project documentation:
- [QuadrotorStateEstimation/README.md](/Users/oguz/Desktop/workspace_cpp/Filters/QuadrotorStateEstimation/README.md)

## SensorFusion-ObjectTracking

`SensorFusion-ObjectTracking` is a lidar/radar highway object-tracking project for benchmarking classical filters and smoothers.

It includes:
- EKF
- IEKF
- UKF
- CKF
- PF
- IMM
- RTS smoothing
- fixed-lag smoothing
- an MHE scaffold integrated into the same highway test harness

The focus here is:
- object tracking rather than robot localization
- comparing estimators under the same simulated highway scenario
- RMSE-based evaluation for tracked vehicle states

Build pattern:

```bash
cd SensorFusion-ObjectTracking/build
cmake ..
cmake --build . -j2
./ukf_highway
```

Project documentation:
- [SensorFusion-ObjectTracking/README.md](/Users/oguz/Desktop/workspace_cpp/Filters/SensorFusion-ObjectTracking/README.md)

## Camera

`Camera` contains camera-based perception and computer-vision projects.

Typical work in this area includes:
- feature detection and matching
- image-based tracking
- 3D object tracking with camera data
- calibration and perception exercises

This folder is the right place to look if you want vision-focused pipelines rather than state-estimation-only code.

## Lidar

`Lidar` contains lidar-focused perception work.

Typical work in this area includes:
- point cloud filtering
- segmentation
- clustering
- obstacle detection
- geometric processing on 3D lidar data

This folder is primarily about perception from point clouds rather than recursive state estimation.

## Radar

`Radar` contains radar-related signal processing and detection work.

Typical work in this area includes:
- radar measurement interpretation
- target detection concepts
- radar-specific processing pipelines

This folder is the radar-specific side of the workspace, separate from the lidar/radar object-tracking benchmark in `SensorFusion-ObjectTracking`.

## Choosing The Right Project

Use `QuadrotorStateEstimation` if you want:
- IMU/GPS/magnetometer fusion
- a state-estimation workflow with known simulator ground truth
- EKF prediction/update debugging on a flying vehicle

Use `SensorFusion-ObjectTracking` if you want:
- lidar/radar target tracking
- multiple classical filters in one benchmark
- smoothing methods such as RTS and fixed-lag on tracked-object trajectories

Use `Camera`, `Lidar`, or `Radar` if you want:
- modality-specific perception exercises
- preprocessing and detection pipelines tied to one sensor type
- supporting perception projects outside the two main estimation benchmarks

## Dependencies

Common dependencies across the workspace:
- CMake
- a modern C++ compiler
- Eigen

Additional project-specific dependencies:
- `QuadrotorStateEstimation`: Qt5, OpenGL, GLUT
- `SensorFusion-ObjectTracking`: PCL
