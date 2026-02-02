# ===================================================================================
#  The TYCam Library CMake configuration file
#
#             ** File generated automatically, do not modify **
#
#  Usage from an external project:
#    In your CMakeLists.txt, add these lines:
#
#    find_package(TYCam REQUIRED)
#    include_directories(${TYCam_INCLUDE_DIRS})
#    target_link_libraries(MY_TARGET_NAME ${TYCam_LIBS})
#
#    This file will define the following variables:
#      - TYCam_LIBS                      : The list of all imported targets for TYCam
#      - TYCam_INCLUDE_DIRS              : The TYCam include directories.
#      - TYCam_VERSION                  : The version of this TYCam build: "4.2.0"
#      - TYCam_VERSION_MAJOR            : Major version part of TYCam_VERSION: "4"
#      - TYCam_VERSION_MINOR            : Minor version part of TYCam_VERSION: "2"
#      - TYCam_VERSION_PATCH            : Patch version part of TYCam_VERSION: "0"
#
# ===================================================================================

# ======================================================
#  Version variables:
# ======================================================
SET(TYCam_VERSION_MAJOR  4)
SET(TYCam_VERSION_MINOR  2)
SET(TYCam_VERSION_PATCH  10)
SET(TYCam_VERSION 4.2.10)

include(FindPackageHandleStandardArgs)

message("TYCam_VERSION: ${TYCam_VERSION}")
# Extract directory name from full path of the file currently being processed.
# Note that CMake 2.8.3 introduced CMAKE_CURRENT_LIST_DIR. We reimplement it
# for older versions of CMake to support these as well.
if(CMAKE_VERSION VERSION_LESS "2.8.3")
  get_filename_component(CMAKE_CURRENT_LIST_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
endif()

# Extract the directory where *this* file has been installed (determined at cmake run-time)
# Get the absolute path with no ../.. relative marks, to eliminate implicit linker warnings
get_filename_component(TYCAM_LIB_PATH "${CMAKE_CURRENT_LIST_DIR}" REALPATH)

message("CMAKE_CURRENT_LIST_DIR: ${CMAKE_CURRENT_LIST_DIR}")
message("TYCAM_LIB_PATH: ${TYCAM_LIB_PATH}")

set(TYCam_INCLUDE_DIRS ${TYCam_INCLUDE_DIRS} "${TYCAM_LIB_PATH}/include")

set(ABSOLUTE_TYCAM_LIB tycam)
add_library(${ABSOLUTE_TYCAM_LIB} SHARED IMPORTED)
if (MSVC)#for windows
    set (LIB_ROOT_PATH ${TYCAM_LIB_PATH}/lib/win/hostapp/)
    if(CMAKE_CL_64) #x64
        set_property(TARGET ${ABSOLUTE_TYCAM_LIB} PROPERTY IMPORTED_LOCATION ${LIB_ROOT_PATH}/x64/tycam.dll)
        set_property(TARGET ${ABSOLUTE_TYCAM_LIB} PROPERTY IMPORTED_IMPLIB  ${LIB_ROOT_PATH}/x64/tycam.lib)
    else()
        set_property(TARGET ${ABSOLUTE_TYCAM_LIB} PROPERTY IMPORTED_LOCATION ${LIB_ROOT_PATH}/x86/tycam.dll)
        set_property(TARGET ${ABSOLUTE_TYCAM_LIB} PROPERTY IMPORTED_IMPLIB ${LIB_ROOT_PATH}/x86/tycam.lib)
    endif()
else()
  if(ARCH)
      set_property(TARGET ${ABSOLUTE_TYCAM_LIB} PROPERTY IMPORTED_LOCATION ${TYCAM_LIB_PATH}/lib/linux/lib_${ARCH}/libtycam.so)
  else()
      set(ABSOLUTE_TYCAM_LIB -ltycam)
  endif()
endif()

set(ABSOLUTE_TYIMGPROC_LIB tyimgproc)
add_library(${ABSOLUTE_TYIMGPROC_LIB} SHARED IMPORTED)
if (MSVC)#for windows
    if(CMAKE_CL_64) #x64
        set_property(TARGET ${ABSOLUTE_TYIMGPROC_LIB} PROPERTY IMPORTED_LOCATION ${LIB_ROOT_PATH}/x64/tyimgproc.dll)
        set_property(TARGET ${ABSOLUTE_TYIMGPROC_LIB} PROPERTY IMPORTED_IMPLIB  ${LIB_ROOT_PATH}/x64/tyimgproc.lib)
    else()
        set_property(TARGET ${ABSOLUTE_TYIMGPROC_LIB} PROPERTY IMPORTED_LOCATION ${LIB_ROOT_PATH}/x86/tyimgproc.dll)
        set_property(TARGET ${ABSOLUTE_TYIMGPROC_LIB} PROPERTY IMPORTED_IMPLIB ${LIB_ROOT_PATH}/x86/tyimgproc.lib)
    endif()
else()
    if(ARCH)
        set_property(TARGET ${ABSOLUTE_TYIMGPROC_LIB} PROPERTY IMPORTED_LOCATION ${TYCAM_LIB_PATH}/lib/linux/lib_${ARCH}/libtyimgproc.so)
    else()
        set(ABSOLUTE_TYIMGPROC_LIB -ltyimgproc)
    endif()
endif()

set(TYCam_LIBS ${TYCam_LIBS} ${ABSOLUTE_TYCAM_LIB} ${ABSOLUTE_TYIMGPROC_LIB})
