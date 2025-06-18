# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_ark_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED ark_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(ark_FOUND FALSE)
  elseif(NOT ark_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(ark_FOUND FALSE)
  endif()
  return()
endif()
set(_ark_CONFIG_INCLUDED TRUE)

# output package information
if(NOT ark_FIND_QUIETLY)
  message(STATUS "Found ark: 0.0.0 (${ark_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'ark' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${ark_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(ark_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${ark_DIR}/${_extra}")
endforeach()
