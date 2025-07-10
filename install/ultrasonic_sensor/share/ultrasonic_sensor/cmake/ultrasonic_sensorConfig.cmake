# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_ultrasonic_sensor_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED ultrasonic_sensor_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(ultrasonic_sensor_FOUND FALSE)
  elseif(NOT ultrasonic_sensor_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(ultrasonic_sensor_FOUND FALSE)
  endif()
  return()
endif()
set(_ultrasonic_sensor_CONFIG_INCLUDED TRUE)

# output package information
if(NOT ultrasonic_sensor_FIND_QUIETLY)
  message(STATUS "Found ultrasonic_sensor: 0.0.0 (${ultrasonic_sensor_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'ultrasonic_sensor' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ultrasonic_sensor_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(ultrasonic_sensor_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${ultrasonic_sensor_DIR}/${_extra}")
endforeach()
