# -- Minimum required version
cmake_minimum_required (VERSION 3.8)

set(PROJ sx1302_hal)

# -- Project name
project (${PROJ})

# -- Various includes
include (CMakePackageConfigHelpers)
include (GNUInstallDirs)

# -- Set c99 standard default
set(CMAKE_C_STANDARD 99)

# -- options for shared lib (defaults off)
option(${PROJ}_build_shared_libs "build as a shared library" OFF)
set(BUILD_SHARED_LIBS ${${PROJ}_build_shared_libs})

# -- Required to build
set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)
find_package(Threads REQUIRED)

# -- Versioning with git tag
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/.git)
  execute_process(
    COMMAND git describe --tags --always
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    OUTPUT_VARIABLE "${PROJ}_VERSION"
    ERROR_QUIET
    OUTPUT_STRIP_TRAILING_WHITESPACE)
  if(${PROJ}_VERSION STREQUAL "")
    set(${PROJ}_VERSION 0)
  endif(${PROJ}_VERSION STREQUAL "")
  message( STATUS "Git full version: ${${PROJ}_VERSION}" )
  execute_process(
    COMMAND /bin/bash -c "git describe --tags --abbrev=0 | cut --delimiter='v' --fields=2"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    OUTPUT_VARIABLE "${PROJ}_VERSION_SHORT"
    ERROR_QUIET
    OUTPUT_STRIP_TRAILING_WHITESPACE)
  if(${PROJ}_VERSION_SHORT STREQUAL "")
    set(${PROJ}_VERSION_SHORT 0)
  endif(${PROJ}_VERSION_SHORT STREQUAL "")
  message( STATUS "Git version: ${${PROJ}_VERSION_SHORT}" )
else(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/.git)
  set(${PROJ}_VERSION_SHORT 0)
  set(${PROJ}_VERSION 0)
endif(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/.git)

# when building, don't use the install RPATH already
# (but later on when installing)
SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
if (NOT (${CMAKE_INSTALL_PREFIX} STREQUAL "/usr" ) )
   SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}")
endif()

# -- add tools library
add_subdirectory(libtools)

# -- add the core library
add_subdirectory(libloragw)

# -- add packet forwarder
add_subdirectory(packet_forwarder)

# --add util_boot
add_subdirectory(util_boot)

# --add util_chip_id
add_subdirectory(util_chip_id)

# --add util_net_downlink
add_subdirectory(util_net_downlink)

# --add util_spectral_scan
add_subdirectory(util_spectral_scan)