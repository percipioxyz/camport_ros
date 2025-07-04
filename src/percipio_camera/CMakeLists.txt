cmake_minimum_required(VERSION 2.8.3)
project(percipio_camera)

find_package(catkin REQUIRED camera_info_manager std_msgs tf2 tf2_eigen tf2_msgs tf2_ros cv_bridge pcl_conversions pcl_ros dynamic_reconfigure image_transport  nodelet sensor_msgs roscpp message_generation)

find_package(Boost REQUIRED COMPONENTS system thread)

find_package(PkgConfig)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fpic")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpic")

generate_dynamic_reconfigure_options(cfg/Percipio.cfg)
add_service_files(FILES
  GetSerial.srv)
generate_messages()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES percipio_wrapper
  CATKIN_DEPENDS camera_info_manager dynamic_reconfigure image_transport nodelet sensor_msgs roscpp message_runtime
  DEPENDS libtycam
)

include_directories(include 
                    ${catkin_INCLUDE_DIRS} 
                    ${Boost_INCLUDE_DIRS}
                    )

#link_directories(${PC_PERCIPIO_LIBRARY_DIRS})

add_library(percipio_wrapper
   src/TYThread.cpp
   src/percipio_interface.cpp
   src/percipio_convert.cpp
   src/percipio_device.cpp
   src/gige_2_0.cpp
   src/gige_2_1.cpp
   src/DepthStreamProc.cpp
   src/percipio_device_info.cpp 
   src/percipio_timer_filter.cpp 
   src/percipio_frame_listener.cpp
   src/percipio_device_manager.cpp
   src/percipio_exception.cpp
   src/percipio_video_mode.cpp
   src/percipio_depth_algorithm.cpp
   src/huffman.cpp
)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/camport3/include)

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

set(TYCAM ${CMAKE_CURRENT_SOURCE_DIR}/camport3/lib/linux/lib_${ARCH}/libtycam.so)

target_link_libraries(percipio_wrapper ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} ${TYCAM})

#add_executable(test_wrapper test/test_wrapper.cpp )
#target_link_libraries(test_wrapper percipio_wrapper ${Boost_LIBRARIES} ${TYCAM})

add_library(percipio_driver_lib
   src/percipio_driver.cpp
)

find_package(OpenCV REQUIRED PATHS ${catkin_INCLUDE_DIRS})
if (NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV library not found")
else()
    message(STATUS "OpenCV version: ${OpenCV_VERSION}")
    include_directories(${OpenCV_INCLUDE_DIRS})
    target_link_libraries(percipio_wrapper ${OpenCV_LIBRARIES})
    target_link_libraries(percipio_driver_lib ${OpenCV_LIBRARIES})
endif()

find_package(PCL REQUIRED)
if (NOT PCL_FOUND)
    message(FATAL_ERROR "PCL library not found")
else()
    include_directories(${PCL_INCLUDE_DIRS})
    target_link_libraries(percipio_wrapper ${PCL_LIBRARIES})
    target_link_libraries(percipio_driver_lib ${PCL_LIBRARIES})
endif()

target_link_libraries(percipio_driver_lib percipio_wrapper ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} )
add_dependencies(percipio_driver_lib ${PROJECT_NAME}_gencfg)

add_library(percipio_camera_nodelet
   ros/percipio_camera_nodelet.cpp
)
target_link_libraries(percipio_camera_nodelet percipio_driver_lib ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} )
add_dependencies(percipio_camera_nodelet ${PROJECT_NAME}_gencfg)

add_executable(percipio_camera_node
   ros/percipio_camera_node.cpp
)
target_link_libraries(percipio_camera_node percipio_driver_lib ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} )
add_dependencies(percipio_camera_node ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages_cpp)

add_executable(list_devices
   src/list_devices.cpp
)
target_link_libraries(list_devices percipio_wrapper)

install(TARGETS percipio_wrapper percipio_camera_nodelet percipio_camera_node list_devices percipio_driver_lib ${ADDITIONAL_EXECUTABLES}
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



