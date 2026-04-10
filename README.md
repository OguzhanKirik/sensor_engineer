# Sensor Fusion Engineer Projects

A comprehensive collection of sensor fusion, computer vision, and object tracking implementations for autonomous systems.

## 📁 Repository Structure

### 🎥 Camera
Camera-based perception projects including feature detection, tracking, and 3D object tracking.

- **Camera-Exercises**: Fundamental camera calibration and computer vision exercises
- **SFND_2D_Feature_Matching**: 2D feature detection and matching implementation
- **SFND_3D_Object_Tracking**: 3D object tracking using camera data and lidar fusion

### 📊 Kalman Filters

#### KalmanFillter
Basic Kalman filter implementations and exercises for state estimation.

#### Filters (UKF/EKF Highway Project)
Advanced Kalman filter implementations for highway vehicle tracking:
- **Extended Kalman Filter (EKF)**: Linear approximation-based filtering
- **Unscented Kalman Filter (UKF)**: Sigma point-based filtering for non-linear systems
- **Sensor Fusion**: Combines radar and lidar measurements
- **Real-time Tracking**: Multi-vehicle highway scenario with visualization

**Features:**
- CTRV (Constant Turn Rate and Velocity) motion model
- Radar and Lidar sensor fusion
- RMSE-based performance evaluation
- PCL-based 3D visualization

See [Filters/README.md](Filters/README.md) for detailed setup and build instructions.

### 📡 Radar
Radar signal processing and target detection implementations.

### 🔦 Lidar
Lidar point cloud processing, clustering, and object detection projects.

## 🛠️ Prerequisites

### General Dependencies
- **CMake** >= 3.5
- **Make** >= 4.1 (Linux, Mac), 3.81 (Windows)
- **GCC/G++** >= 5.4
- **C++11** or higher

### Library Dependencies
- **Eigen3**: Linear algebra library
- **PCL** (Point Cloud Library): For lidar processing and 3D visualization
- **OpenCV**: For camera-based projects

### Installation (macOS)
```bash
# Install build tools
xcode-select --install

# Install dependencies via Homebrew
brew install cmake
brew install eigen
brew install pcl
brew install opencv
```

### Installation (Linux - Ubuntu/Debian)
```bash
sudo apt-get update
sudo apt-get install cmake build-essential
sudo apt-get install libeigen3-dev
sudo apt-get install libpcl-dev
sudo apt-get install libopencv-dev
```

## 🚀 Quick Start

Each project contains its own build instructions. General build pattern:

```bash
# Navigate to project directory
cd <project-name>

# Create build directory
mkdir build && cd build

# Configure and build
cmake ..
make

# Run executable
./<executable-name>
```

## 📚 Project Details

### Filters (UKF/EKF)
Multi-object tracking on a highway using sensor fusion:
```bash
cd Filters/build
./ukf_highway    # UKF implementation
./filters_highway # EKF implementation
```

### Camera Projects
Feature detection, matching, and 3D object tracking:
```bash
cd Camera/SFND_2D_Feature_Matching/build
make
./2D_feature_tracking
```

## 📖 Learning Resources

This repository contains implementations from:
- Sensor Fusion Nanodegree projects
- Computer Vision fundamentals
- Kalman Filtering techniques
- Point Cloud Processing

## 🎯 Key Concepts Covered

- **State Estimation**: EKF, UKF, Particle Filters
- **Sensor Fusion**: Combining lidar, radar, and camera data
- **Object Tracking**: Multi-object tracking and data association
- **Feature Detection**: SIFT, SURF, ORB, FAST, BRIEF
- **Point Cloud Processing**: Segmentation, clustering, filtering
- **Motion Models**: CTRV, CV, CA models

## 📝 Code Style

Projects follow [Google's C++ Style Guide](https://google.github.io/styleguide/cppguide.html).

## 🔗 Original Sources

Some projects are based on Udacity's Sensor Fusion Nanodegree:
- [SFND_Unscented_Kalman_Filter](https://github.com/udacity/SFND_Unscented_Kalman_Filter)

## 📄 License

Individual projects may have their own licenses. Please check each project directory.

## 👤 Author

Oguzhan Kirik - [GitHub](https://github.com/OguzhanKirik)

---

**Note**: For specific project setup and detailed instructions, refer to the README.md file in each project directory.
# SFND_Unscented_Kalman_Filter
Sensor Fusion UKF Highway Project Starter Code

<img src="media/ukf_highway_tracked.gif" width="700" height="400" />

In this project you will implement an Unscented Kalman Filter to estimate the state of multiple cars on a highway using noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower that the tolerance outlined in the project rubric.

## Prerequisites

**Eigen Library and Sensor Data:**  
This project requires the Eigen library and sensor data. Please refer to the original Udacity repository for setup instructions and data files:  
**[https://github.com/udacity/SFND_Unscented_Kalman_Filter](https://github.com/udacity/SFND_Unscented_Kalman_Filter)** 

The main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ukf_highway

Note that the programs that need to be written to accomplish the project are src/ukf.cpp, and src/ukf.h

The program main.cpp has already been filled out, but feel free to modify it.

<img src="media/ukf_highway.png" width="700" height="400" />

`main.cpp` is using `highway.h` to create a straight 3 lane highway environment with 3 traffic cars and the main ego car at the center. 
The viewer scene is centered around the ego car and the coordinate system is relative to the ego car as well. The ego car is green while the 
other traffic cars are blue. The traffic cars will be accelerating and altering their steering to change lanes. Each of the traffic car's has
it's own UKF object generated for it, and will update each indidual one during every time step. 

The red spheres above cars represent the (x,y) lidar detection and the purple lines show the radar measurements with the velocity magnitude along the detected angle. The Z axis is not taken into account for tracking, so you are only tracking along the X/Y axis.

---

## Other Important Dependencies
* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
 * PCL 1.2

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./ukf_highway`

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html) as much as possible.

## Generating Additional Data

This is optional!

If you'd like to generate your own radar and lidar modify the code in `highway.h` to alter the cars. Also check out `tools.cpp` to
change how measurements are taken, for instance lidar markers could be the (x,y) center of bounding boxes by scanning the PCD environment
and performing clustering. This is similar to what was done in Sensor Fusion Lidar Obstacle Detection.

## Project Instructions and Rubric

This information is only accessible by people who are already enrolled in Sensor Fusion. 
If you are enrolled, see the project page in the classroom
for instructions and the project rubric.
