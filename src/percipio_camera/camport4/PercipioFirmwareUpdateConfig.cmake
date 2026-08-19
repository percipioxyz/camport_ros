# ===================================================================================
#  The PercipioFirmwareUpdate Library CMake configuration file
#
#             ** File generated automatically, do not modify **
#
#  Usage from an external project:
#    In your CMakeLists.txt, add these lines:
#
#    find_package(PercipioFirmwareUpdate REQUIRED)
#    include_directories(${PercipioFirmwareUpdate_INCLUDE_DIRS})
#    target_link_libraries(MY_TARGET_NAME ${PercipioFirmwareUpdate_LIB})
#
#    This file will define the following variables:
#      - PercipioFirmwareUpdate_LIB            : The list of all imported targets for PercipioFirmwareUpdate
#      - PercipioFirmwareUpdate_INCLUDE_DIRS   : The PercipioFirmwareUpdate include directories.
#      - PercipioFirmwareUpdate_VERSION        : The version of this PercipioFirmwareUpdate build: "1.1.0"
#      - PercipioFirmwareUpdate_VERSION_MAJOR  : Major version part of PercipioFirmwareUpdate_VERSION: "1"
#      - PercipioFirmwareUpdate_VERSION_MINOR  : Minor version part of PercipioFirmwareUpdate_VERSION: "1"
#      - PercipioFirmwareUpdate_VERSION_PATCH  : Patch version part of PercipioFirmwareUpdate_VERSION: "0"
#
# ===================================================================================

# ======================================================
#  Version variables:
# ======================================================
SET(PercipioFirmwareUpdate_VERSION_MAJOR  1)
SET(PercipioFirmwareUpdate_VERSION_MINOR  0)
SET(PercipioFirmwareUpdate_VERSION_PATCH  33)
SET(PercipioFirmwareUpdate_VERSION 1.0.33)

include(FindPackageHandleStandardArgs)

message("PercipioFirmwareUpdate_VERSION: ${PercipioFirmwareUpdate_VERSION}")
# Extract directory name from full path of the file currently being processed.
# Note that CMake 2.8.3 introduced CMAKE_CURRENT_LIST_DIR. We reimplement it
# for older versions of CMake to support these as well.
if(CMAKE_VERSION VERSION_LESS "2.8.3")
  get_filename_component(CMAKE_CURRENT_LIST_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
endif()

# Extract the directory where *this* file has been installed (determined at cmake run-time)
# Get the absolute path with no ../.. relative marks, to eliminate implicit linker warnings
get_filename_component(FIRMWAREUPDATE_LIB_PATH "${CMAKE_CURRENT_LIST_DIR}" REALPATH)

message("CMAKE_CURRENT_LIST_DIR: ${CMAKE_CURRENT_LIST_DIR}")
message("FIRMWAREUPDATE_LIB_PATH: ${FIRMWAREUPDATE_LIB_PATH}")

set(PercipioFirmwareUpdate_INCLUDE_DIRS ${PercipioFirmwareUpdate_INCLUDE_DIRS} "${FIRMWAREUPDATE_LIB_PATH}/include")

set(ABSOLUTE_FIRMWAREUPDATE_LIB PercipioFirmwareUpdate)
add_library(${ABSOLUTE_FIRMWAREUPDATE_LIB} SHARED IMPORTED)
if (MSVC)#for windows
    set (LIB_ROOT_PATH ${FIRMWAREUPDATE_LIB_PATH}/lib/win/)
    if(CMAKE_CL_64) #x64
        set_property(TARGET ${ABSOLUTE_FIRMWAREUPDATE_LIB} PROPERTY IMPORTED_LOCATION ${LIB_ROOT_PATH}/x64/PercipioFirmwareUpdate.dll)
        set_property(TARGET ${ABSOLUTE_FIRMWAREUPDATE_LIB} PROPERTY IMPORTED_IMPLIB  ${LIB_ROOT_PATH}/x64/PercipioFirmwareUpdate.lib)
    else()
        set_property(TARGET ${ABSOLUTE_FIRMWAREUPDATE_LIB} PROPERTY IMPORTED_LOCATION ${LIB_ROOT_PATH}/x86/PercipioFirmwareUpdate.dll)
        set_property(TARGET ${ABSOLUTE_FIRMWAREUPDATE_LIB} PROPERTY IMPORTED_IMPLIB  ${LIB_ROOT_PATH}/x86/PercipioFirmwareUpdate.lib)
    endif()
else()
    if(ARCH)
        set_property(TARGET ${ABSOLUTE_FIRMWAREUPDATE_LIB} PROPERTY IMPORTED_LOCATION ${FIRMWAREUPDATE_LIB_PATH}/lib/linux/lib_${ARCH}/libPercipioFirmwareUpdate.so)
    else()
        set(ABSOLUTE_FIRMWAREUPDATE_LIB -lPercipioFirmwareUpdate)
    endif()
endif()

set(PercipioFirmwareUpdate_LIB ${PercipioFirmwareUpdate_LIB} "${ABSOLUTE_FIRMWAREUPDATE_LIB}")
