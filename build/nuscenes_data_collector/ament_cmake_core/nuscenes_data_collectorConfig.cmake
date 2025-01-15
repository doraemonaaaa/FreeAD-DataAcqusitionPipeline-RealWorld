# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_nuscenes_data_collector_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED nuscenes_data_collector_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(nuscenes_data_collector_FOUND FALSE)
  elseif(NOT nuscenes_data_collector_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(nuscenes_data_collector_FOUND FALSE)
  endif()
  return()
endif()
set(_nuscenes_data_collector_CONFIG_INCLUDED TRUE)

# output package information
if(NOT nuscenes_data_collector_FIND_QUIETLY)
  message(STATUS "Found nuscenes_data_collector: 0.0.0 (${nuscenes_data_collector_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'nuscenes_data_collector' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${nuscenes_data_collector_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(nuscenes_data_collector_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${nuscenes_data_collector_DIR}/${_extra}")
endforeach()
