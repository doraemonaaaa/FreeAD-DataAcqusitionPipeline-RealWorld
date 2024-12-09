#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "diff_control::diff_control" for configuration ""
set_property(TARGET diff_control::diff_control APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(diff_control::diff_control PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libdiff_control.so"
  IMPORTED_SONAME_NOCONFIG "libdiff_control.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS diff_control::diff_control )
list(APPEND _IMPORT_CHECK_FILES_FOR_diff_control::diff_control "${_IMPORT_PREFIX}/lib/libdiff_control.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
