# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_vesc_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED vesc_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(vesc_FOUND FALSE)
  elseif(NOT vesc_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(vesc_FOUND FALSE)
  endif()
  return()
endif()
set(_vesc_CONFIG_INCLUDED TRUE)

# output package information
if(NOT vesc_FIND_QUIETLY)
  message(STATUS "Found vesc: 1.2.0 (${vesc_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'vesc' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${vesc_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(vesc_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${vesc_DIR}/${_extra}")
endforeach()
