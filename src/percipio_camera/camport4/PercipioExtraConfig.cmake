# ===================================================================================
#  The PercipioExtra Library CMake configuration file
#
#             ** File generated automatically, do not modify **
#
#  Usage from an external project:
#    In your CMakeLists.txt, add these lines:
#
#    find_package(PercipioExtra REQUIRED)
#    include_directories(${PercipioExtra_INCLUDE_DIRS})
#    target_link_libraries(MY_TARGET_NAME ${PercipioExtra_LIB})
#
#    This file will define the following variables:
#      - PercipioExtra_LIB            : The list of all imported targets for PercipioExtra
#      - PercipioExtra_INCLUDE_DIRS   : The PercipioExtra include directories.
#      - PercipioExtra_VERSION        : The version of this PercipioExtra build: "1.1.0"
#      - PercipioExtra_VERSION_MAJOR  : Major version part of PercipioExtra_VERSION: "1"
#      - PercipioExtra_VERSION_MINOR  : Minor version part of PercipioExtra_VERSION: "1"
#      - PercipioExtra_VERSION_PATCH  : Patch version part of PercipioExtra_VERSION: "0"
#
# ===================================================================================

# ======================================================
#  Version variables:
# ======================================================
SET(PercipioExtra_VERSION_MAJOR  1)
SET(PercipioExtra_VERSION_MINOR  0)
SET(PercipioExtra_VERSION_PATCH  33)
SET(PercipioExtra_VERSION 1.0.33)

include(FindPackageHandleStandardArgs)

message("PercipioExtra_VERSION: ${PercipioExtra_VERSION}")
# Extract directory name from full path of the file currently being processed.
# Note that CMake 2.8.3 introduced CMAKE_CURRENT_LIST_DIR. We reimplement it
# for older versions of CMake to support these as well.
if(CMAKE_VERSION VERSION_LESS "2.8.3")
  get_filename_component(CMAKE_CURRENT_LIST_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
endif()

# Extract the directory where *this* file has been installed (determined at cmake run-time)
# Get the absolute path with no ../.. relative marks, to eliminate implicit linker warnings
get_filename_component(PERCIPIO_EXTRA_LIB_PATH "${CMAKE_CURRENT_LIST_DIR}" REALPATH)

message("CMAKE_CURRENT_LIST_DIR: ${CMAKE_CURRENT_LIST_DIR}")
message("PERCIPIO_EXTRA_LIB_PATH: ${PERCIPIO_EXTRA_LIB_PATH}")

set(PercipioExtra_INCLUDE_DIRS ${PercipioExtra_INCLUDE_DIRS} "${PERCIPIO_EXTRA_LIB_PATH}/include")

set(ABSOLUTE_PERCIPIO_EXTRA_LIB PercipioExtra)
add_library(${ABSOLUTE_PERCIPIO_EXTRA_LIB} SHARED IMPORTED)
if (MSVC)#for windows
    set (LIB_ROOT_PATH ${PERCIPIO_EXTRA_LIB_PATH}/lib/win/)
    if(CMAKE_CL_64) #x64
        set_property(TARGET ${ABSOLUTE_PERCIPIO_EXTRA_LIB} PROPERTY IMPORTED_LOCATION ${LIB_ROOT_PATH}/x64/PercipioExtra.dll)
        set_property(TARGET ${ABSOLUTE_PERCIPIO_EXTRA_LIB} PROPERTY IMPORTED_IMPLIB  ${LIB_ROOT_PATH}/x64/PercipioExtra.lib)
    else()
        set_property(TARGET ${ABSOLUTE_PERCIPIO_EXTRA_LIB} PROPERTY IMPORTED_LOCATION ${LIB_ROOT_PATH}/x86/PercipioExtra.dll)
        set_property(TARGET ${ABSOLUTE_PERCIPIO_EXTRA_LIB} PROPERTY IMPORTED_IMPLIB  ${LIB_ROOT_PATH}/x86/PercipioExtra.lib)
    endif()
else()
    if(ARCH)
        set_property(TARGET ${ABSOLUTE_PERCIPIO_EXTRA_LIB} PROPERTY IMPORTED_LOCATION ${PERCIPIO_EXTRA_LIB_PATH}/lib/linux/lib_${ARCH}/libPercipioExtra.so)
    else()
        set(ABSOLUTE_PERCIPIO_EXTRA_LIB -lPercipioExtra)
    endif()
endif()

set(PercipioExtra_LIB ${PercipioExtra_LIB} "${ABSOLUTE_PERCIPIO_EXTRA_LIB}")
