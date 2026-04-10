cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(playback)

# Point to our custom Eigen config that wraps system Eigen as version 3.3.0
set(Eigen3_DIR "${CMAKE_BINARY_DIR}")
list(INSERT CMAKE_PREFIX_PATH 0 "${CMAKE_BINARY_DIR}")

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${CMAKE_SOURCE_DIR}/src)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")


add_executable (filters_highway 
    src/main.cpp 
    src/filters/ukf/ukf.cpp 
    src/filters/ekf/ekf.cpp 
    src/tools.cpp 
    src/render/render.cpp
)
target_link_libraries (filters_highway ${PCL_LIBRARIES})

# IMM Filter executable with 4 models
add_executable (imm_highway 
    src/main.cpp 
    src/filters/ukf/ukf.cpp 
    src/filters/ekf/ekf.cpp 
    src/motion_models/cv_ekf.cpp
    src/motion_models/ca_ekf.cpp
    src/motion_models/ctrv_ukf.cpp 
    src/motion_models/ctra_ukf.cpp
    src/filters/imm/IMM.cpp 
    src/tools.cpp 
    src/render/render.cpp
)
target_link_libraries (imm_highway ${PCL_LIBRARIES})
target_compile_definitions(imm_highway PRIVATE USE_IMM)
