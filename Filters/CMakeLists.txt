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
    src/filters/mhe/mhe.cpp
    src/filters/rts/ekf_rts_smoother.cpp
    src/filters/lag_smoother/ukf_fixed_lag_smoother.cpp
    src/filters/rts/ukf_rts_smoother.cpp
    src/filters/iekf/iekf.cpp
    src/filters/ckf/ckf.cpp
    src/filters/pf/pf.cpp
    src/tools.cpp 
    src/render/render.cpp
)
target_link_libraries (filters_highway ${PCL_LIBRARIES})

# CKF Filter executable (Cubature Kalman Filter)
add_executable (ckf_highway 
    src/main.cpp 
    src/filters/ckf/ckf.cpp 
    src/filters/ukf/ukf.cpp 
    src/filters/ekf/ekf.cpp 
    src/filters/mhe/mhe.cpp
    src/filters/rts/ekf_rts_smoother.cpp
    src/filters/lag_smoother/ukf_fixed_lag_smoother.cpp
    src/filters/rts/ukf_rts_smoother.cpp
    src/filters/iekf/iekf.cpp
    src/filters/pf/pf.cpp
    src/tools.cpp 
    src/render/render.cpp
)
target_link_libraries (ckf_highway ${PCL_LIBRARIES})
target_compile_definitions(ckf_highway PRIVATE USE_CKF)

# EKF Filter executable (Extended Kalman Filter)
add_executable (ekf_highway 
    src/main.cpp 
    src/filters/ekf/ekf.cpp 
    src/filters/mhe/mhe.cpp
    src/filters/rts/ekf_rts_smoother.cpp
    src/filters/lag_smoother/ukf_fixed_lag_smoother.cpp
    src/filters/rts/ukf_rts_smoother.cpp
    src/filters/ukf/ukf.cpp 
    src/filters/ckf/ckf.cpp
    src/filters/iekf/iekf.cpp
    src/filters/pf/pf.cpp
    src/tools.cpp 
    src/render/render.cpp
)
target_link_libraries (ekf_highway ${PCL_LIBRARIES})
target_compile_definitions(ekf_highway PRIVATE USE_EKF)

# EKF + RTS smoothing executable
add_executable (ekf_rts_highway
    src/main.cpp
    src/filters/ekf/ekf.cpp
    src/filters/mhe/mhe.cpp
    src/filters/rts/ekf_rts_smoother.cpp
    src/filters/lag_smoother/ukf_fixed_lag_smoother.cpp
    src/filters/rts/ukf_rts_smoother.cpp
    src/filters/ukf/ukf.cpp
    src/filters/ckf/ckf.cpp
    src/filters/iekf/iekf.cpp
    src/filters/pf/pf.cpp
    src/tools.cpp
    src/render/render.cpp
)
target_link_libraries (ekf_rts_highway ${PCL_LIBRARIES})
target_compile_definitions(ekf_rts_highway PRIVATE USE_EKF_RTS)

# MHE executable
add_executable (mhe_highway
    src/main.cpp
    src/filters/mhe/mhe.cpp
    src/filters/ekf/ekf.cpp
    src/filters/rts/ekf_rts_smoother.cpp
    src/filters/rts/ukf_rts_smoother.cpp
    src/filters/ukf/ukf.cpp
    src/filters/ckf/ckf.cpp
    src/filters/iekf/iekf.cpp
    src/filters/pf/pf.cpp
    src/tools.cpp
    src/render/render.cpp
)
target_link_libraries (mhe_highway ${PCL_LIBRARIES})
target_compile_definitions(mhe_highway PRIVATE USE_MHE)

# UKF + RTS smoothing executable
add_executable (ukf_rts_highway
    src/main.cpp
    src/filters/ukf/ukf.cpp
    src/filters/ekf/ekf.cpp
    src/filters/mhe/mhe.cpp
    src/filters/rts/ekf_rts_smoother.cpp
    src/filters/lag_smoother/ukf_fixed_lag_smoother.cpp
    src/filters/rts/ukf_rts_smoother.cpp
    src/filters/ckf/ckf.cpp
    src/filters/iekf/iekf.cpp
    src/filters/pf/pf.cpp
    src/tools.cpp
    src/render/render.cpp
)
target_link_libraries (ukf_rts_highway ${PCL_LIBRARIES})
target_compile_definitions(ukf_rts_highway PRIVATE USE_UKF_RTS)

# UKF fixed-lag smoothing executable
add_executable (ukf_fixed_lag_highway
    src/main.cpp
    src/filters/ukf/ukf.cpp
    src/filters/ekf/ekf.cpp
    src/filters/mhe/mhe.cpp
    src/filters/rts/ekf_rts_smoother.cpp
    src/filters/lag_smoother/ukf_fixed_lag_smoother.cpp
    src/filters/rts/ukf_rts_smoother.cpp
    src/filters/ckf/ckf.cpp
    src/filters/iekf/iekf.cpp
    src/filters/pf/pf.cpp
    src/tools.cpp
    src/render/render.cpp
)
target_link_libraries (ukf_fixed_lag_highway ${PCL_LIBRARIES})
target_compile_definitions(ukf_fixed_lag_highway PRIVATE USE_UKF_FIXED_LAG)

# UKF Filter executable (Unscented Kalman Filter)
add_executable (ukf_highway 
    src/main.cpp 
    src/filters/ukf/ukf.cpp 
    src/filters/ekf/ekf.cpp 
    src/filters/mhe/mhe.cpp
    src/filters/rts/ekf_rts_smoother.cpp
    src/filters/lag_smoother/ukf_fixed_lag_smoother.cpp
    src/filters/rts/ukf_rts_smoother.cpp
    src/filters/ckf/ckf.cpp
    src/filters/iekf/iekf.cpp
    src/filters/pf/pf.cpp
    src/tools.cpp 
    src/render/render.cpp
)
target_link_libraries (ukf_highway ${PCL_LIBRARIES})
target_compile_definitions(ukf_highway PRIVATE USE_UKF)

# PF Filter executable (Particle Filter)
add_executable (pf_highway 
    src/main.cpp 
    src/filters/pf/pf.cpp
    src/filters/ukf/ukf.cpp 
    src/filters/ekf/ekf.cpp 
    src/filters/mhe/mhe.cpp
    src/filters/rts/ekf_rts_smoother.cpp
    src/filters/lag_smoother/ukf_fixed_lag_smoother.cpp
    src/filters/rts/ukf_rts_smoother.cpp
    src/filters/iekf/iekf.cpp
    src/filters/ckf/ckf.cpp
    src/tools.cpp 
    src/render/render.cpp
)
target_link_libraries (pf_highway ${PCL_LIBRARIES})
target_compile_definitions(pf_highway PRIVATE USE_PF)

# IMM Filter executable with 4 models
add_executable (imm_highway 
    src/main.cpp 
    src/filters/ukf/ukf.cpp 
    src/filters/ekf/ekf.cpp 
    src/filters/mhe/mhe.cpp
    src/filters/rts/ekf_rts_smoother.cpp
    src/filters/lag_smoother/ukf_fixed_lag_smoother.cpp
    src/filters/rts/ukf_rts_smoother.cpp
    src/filters/iekf/iekf.cpp
    src/filters/ckf/ckf.cpp
    src/filters/pf/pf.cpp
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

# IEKF Filter executable (Iterated Extended Kalman Filter)
add_executable (iekf_highway 
    src/main.cpp 
    src/filters/iekf/iekf.cpp 
    src/filters/ukf/ukf.cpp 
    src/filters/ekf/ekf.cpp 
    src/filters/mhe/mhe.cpp
    src/filters/rts/ekf_rts_smoother.cpp
    src/filters/lag_smoother/ukf_fixed_lag_smoother.cpp
    src/filters/rts/ukf_rts_smoother.cpp
    src/filters/ckf/ckf.cpp
    src/filters/pf/pf.cpp
    src/tools.cpp 
    src/render/render.cpp
)
target_link_libraries (iekf_highway ${PCL_LIBRARIES})
target_compile_definitions(iekf_highway PRIVATE USE_IEKF)
