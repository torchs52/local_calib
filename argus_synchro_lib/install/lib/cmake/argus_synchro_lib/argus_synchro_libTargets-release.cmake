#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "argus_synchro_lib::argus_synchro_lib" for configuration "Release"
set_property(TARGET argus_synchro_lib::argus_synchro_lib APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(argus_synchro_lib::argus_synchro_lib PROPERTIES
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "Open3D::Open3D"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libargus_synchro_lib.so"
  IMPORTED_SONAME_RELEASE "libargus_synchro_lib.so"
  )

list(APPEND _cmake_import_check_targets argus_synchro_lib::argus_synchro_lib )
list(APPEND _cmake_import_check_files_for_argus_synchro_lib::argus_synchro_lib "${_IMPORT_PREFIX}/lib/libargus_synchro_lib.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
