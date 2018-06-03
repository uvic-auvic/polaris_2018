#####################################################################################
# CMake                                                                             #
#####################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(monitor)
set(CMAKE_CXX_FLAGS "-std=c++11")
set(Boost_USE_MULTITHREADED OFF)
#####################################################################################
# Catkin                                                                            #
#####################################################################################

## What packages to we need to compile with our package ?
find_package(catkin REQUIRED
  roscpp
  rospy
  std_msgs
  message_generation
  serial
)

find_package(Boost REQUIRED)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   GetSerialDevice.srv
   GetSerialDevices.srv
)

add_message_files(
  FILES
  SerialDevice.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing 
)

catkin_package(
   CATKIN_DEPENDS message_runtime std_msgs
)

include_directories(
${catkin_INCLUDE_DIRS})

#####################################################################################
# Binaries                                                                          #
#####################################################################################

## Required packages for linking
find_package(serial REQUIRED)

## Declare a C++ executable
add_executable(serial_manager src/serial_manager.cpp src/devices.hpp)
add_dependencies(serial_manager monitor_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_include_directories(serial_manager PUBLIC ${Boost_INCLUDE_DIRS})
target_link_libraries(serial_manager ${catkin_LIBRARIES} ${serial_LIBRARIES})
