cmake_minimum_required(VERSION 2.8.3)
project(percipio_camera)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_C_STANDARD 11)

find_package(catkin REQUIRED camera_info_manager std_msgs tf2 tf2_eigen tf2_msgs tf2_ros cv_bridge pcl_conversions pcl_ros dynamic_reconfigure image_transport  nodelet sensor_msgs roscpp message_generation)

find_package(Boost REQUIRED COMPONENTS system thread)

find_package(PkgConfig)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fpic")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpic")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Werror")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror")

if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "i686")
  set(ARCH i686)
elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
  set(ARCH x64)
elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
  set(ARCH Aarch64)
elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "armv7")
  set(ARCH armv7hf)
else()
  message(STATUS "host processor architecture is unknown")
endif()

find_package(TYCam REQUIRED PATHS ${CMAKE_CURRENT_SOURCE_DIR}/camport4 NO_DEFAULT_PATH)

generate_dynamic_reconfigure_options(cfg/Percipio.cfg)
add_service_files(FILES
  GetSerial.srv)
generate_messages()

catkin_package(
  INCLUDE_DIRS include ${TYCam_INCLUDE_DIRS}
  LIBRARIES percipio_wrapper
  CATKIN_DEPENDS camera_info_manager dynamic_reconfigure image_transport nodelet sensor_msgs roscpp message_runtime
  DEPENDS TYCam
)

include_directories(include 
                    ${catkin_INCLUDE_DIRS} 
                    ${Boost_INCLUDE_DIRS}
                    ${TYCam_INCLUDE_DIRS}
                    )

#link_directories(${PC_PERCIPIO_LIBRARY_DIRS})

add_library(percipio_wrapper
   src/TYThread.cpp
   src/percipio_interface.cpp
   src/percipio_convert.cpp
   src/percipio_device.cpp
   src/gige_2_0.cpp
   src/gige_2_1.cpp
   src/percipio_device_info.cpp 
   src/percipio_timer_filter.cpp 
   src/percipio_frame_listener.cpp
   src/percipio_device_manager.cpp
   src/percipio_exception.cpp
   src/percipio_video_mode.cpp
   src/percipio_depth_algorithm.cpp
   src/percipio_log_server.cpp
   src/huffman.cpp
   src/percipio_xml.cpp
   src/ParametersParse.cpp
)

target_link_libraries(percipio_wrapper ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TYCam_LIBS})

#add_executable(test_wrapper test/test_wrapper.cpp )
#target_link_libraries(test_wrapper percipio_wrapper ${Boost_LIBRARIES} ${TYCAM})

add_library(percipio_driver_lib
   src/percipio_driver.cpp
)

find_package(PCL REQUIRED)
if (NOT PCL_FOUND)
    message(FATAL_ERROR "PCL library not found")
else()
    include_directories(${PCL_INCLUDE_DIRS})
    target_link_libraries(percipio_wrapper ${PCL_LIBRARIES})
    target_link_libraries(percipio_driver_lib ${PCL_LIBRARIES})
endif()

target_link_libraries(percipio_driver_lib percipio_wrapper ${catkin_LIBRARIES} ${Boost_LIBRARIES} )
add_dependencies(percipio_driver_lib ${PROJECT_NAME}_gencfg)

add_library(percipio_camera_nodelet
   ros/percipio_camera_nodelet.cpp
)
target_link_libraries(percipio_camera_nodelet percipio_driver_lib ${catkin_LIBRARIES} ${Boost_LIBRARIES} )
add_dependencies(percipio_camera_nodelet ${PROJECT_NAME}_gencfg)

add_executable(percipio_camera_node
   ros/percipio_camera_node.cpp
)
target_link_libraries(percipio_camera_node percipio_driver_lib ${catkin_LIBRARIES} ${Boost_LIBRARIES} )
add_dependencies(percipio_camera_node ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages_cpp)

add_executable(list_devices
   src/list_devices.cpp
)

add_executable(network_ip_config
   src/network_ip_config.cpp
)

target_link_libraries(list_devices percipio_wrapper)
target_link_libraries(network_ip_config percipio_wrapper)

install(TARGETS percipio_wrapper percipio_camera_nodelet percipio_camera_node list_devices network_ip_config percipio_driver_lib ${ADDITIONAL_EXECUTABLES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# add xml file
install(FILES percipio_nodelets.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(ARCH)
  if(MSVC)
    if(CMAKE_CL_64)
      set(LIB_SUBDIR "x64")
    else()
      set(LIB_SUBDIR "x86")
    endif()
    
    set(TYCAM_DLL "${TYCAM_LIB_PATH}/lib/win/hostapp/${LIB_SUBDIR}/tycam.dll")
    set(TYIMGPROC_DLL "${TYCAM_LIB_PATH}/lib/win/hostapp/${LIB_SUBDIR}/tyimgproc.dll")
    
    if(EXISTS ${TYCAM_DLL} AND EXISTS ${TYIMGPROC_DLL})
      install(FILES ${TYCAM_DLL} ${TYIMGPROC_DLL}
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
      )
    else()
      message(WARNING "TYCam DLL files not found: ${TYCAM_DLL}, ${TYIMGPROC_DLL}")
    endif()
    
  else()
    set(TYCAM_LIB_DIR "${TYCAM_LIB_PATH}/lib/linux/lib_${ARCH}")
    
    if(EXISTS ${TYCAM_LIB_DIR})
      install(DIRECTORY "${TYCAM_LIB_DIR}/"
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        FILES_MATCHING
        PATTERN "libtycam.so*"
        PATTERN "libtyimgproc.so*"
      )
      
      set(CMAKE_INSTALL_RPATH "$ORIGIN/../lib")
      set_target_properties(percipio_camera_node percipio_camera_nodelet list_devices network_ip_config
        PROPERTIES
          INSTALL_RPATH ${CMAKE_INSTALL_RPATH}
          BUILD_WITH_INSTALL_RPATH TRUE
      )
    else()
      message(WARNING "TYCam library directory not found: ${TYCAM_LIB_DIR}")
    endif()
  endif()
endif()