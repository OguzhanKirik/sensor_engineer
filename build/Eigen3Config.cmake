# Eigen3 Config wrapper - uses system Eigen but reports version 3.3.0 for PCL compatibility
set(EIGEN3_FOUND TRUE)
set(Eigen3_FOUND TRUE)
set(EIGEN3_INCLUDE_DIR "/opt/homebrew/include/eigen3")
set(EIGEN3_INCLUDE_DIRS "/opt/homebrew/include/eigen3")
set(Eigen3_INCLUDE_DIRS "/opt/homebrew/include/eigen3")
set(EIGEN3_VERSION "3.3.0")
set(Eigen3_VERSION "3.3.0")
set(EIGEN3_VERSION_STRING "3.3.0")

# Provide Eigen3::Eigen target if needed
if(NOT TARGET Eigen3::Eigen)
  add_library(Eigen3::Eigen INTERFACE IMPORTED)
  set_target_properties(Eigen3::Eigen PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${EIGEN3_INCLUDE_DIR}"
  )
endif()
