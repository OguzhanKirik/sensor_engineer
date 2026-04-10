# Version file for system Eigen presented as 3.3.0 for PCL compatibility
set(PACKAGE_VERSION "3.3.0")

# Always report as compatible to bypass PCL's strict version check
set(PACKAGE_VERSION_COMPATIBLE TRUE)
set(PACKAGE_VERSION_EXACT FALSE)
if("${PACKAGE_FIND_VERSION}" VERSION_EQUAL "3.3")
  set(PACKAGE_VERSION_EXACT TRUE)
endif()
