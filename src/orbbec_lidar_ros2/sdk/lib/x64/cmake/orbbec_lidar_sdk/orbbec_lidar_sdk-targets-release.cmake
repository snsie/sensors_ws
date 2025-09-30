#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "orbbec_lidar_sdk" for configuration "Release"
set_property(TARGET orbbec_lidar_sdk APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(orbbec_lidar_sdk PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/liborbbec_lidar_sdk.so.1.0.4"
  IMPORTED_SONAME_RELEASE "liborbbec_lidar_sdk.so.1"
  )

list(APPEND _cmake_import_check_targets orbbec_lidar_sdk )
list(APPEND _cmake_import_check_files_for_orbbec_lidar_sdk "${_IMPORT_PREFIX}/lib/liborbbec_lidar_sdk.so.1.0.4" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
