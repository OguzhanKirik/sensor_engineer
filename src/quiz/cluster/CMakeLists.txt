cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(playback)

if(EXISTS "/opt/homebrew/share/eigen3/cmake/Eigen3Config.cmake")
  set(Eigen3_DIR "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Eigen3Compat")
endif()

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")


add_executable (quizCluster cluster.cpp ../../render/render.cpp)
target_link_libraries (quizCluster ${PCL_LIBRARIES})



