cmake_minimum_required(VERSION 2.8.3)
project(navigation)
set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
  peripherals
  dsp
  geometry_msgs
)

find_package(
  peripherals
)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   nav.msg
   joystick.msg
   keyboard.msg
   nav_request.msg
   thrusts.msg
   depth_info.msg
 )

add_service_files(
    FILES
    control_en.srv
    rpm_control_en.srv
)

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   peripherals
 )

catkin_package(
  #LIBRARIES rov
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)


include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${dsp_INCLUDE_DIRS}
)

add_executable(rov src/rov.cpp)
target_link_libraries(rov ${catkin_LIBRARIES})
add_dependencies(rov navigation_generate_messages_cpp peripherals_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

add_executable(thrustmap src/thrustmap.cpp)
target_link_libraries(thrustmap ${catkin_LIBRARIES})
add_dependencies(thrustmap navigation_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

add_executable(control_system src/control_system.cpp src/controllers.cpp src/controllers.hpp src/pid.cpp src/pid.h)
target_link_libraries(control_system ${catkin_LIBRARIES} ${dsp_LIBRARIES})
add_dependencies(control_system navigation_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

add_executable(rpm_controller src/rpm_controller.cpp src/controllers.cpp src/controllers.hpp src/pid.cpp src/pid.h)
target_link_libraries(rpm_controller ${catkin_LIBRARIES})
add_dependencies(rpm_controller navigation_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
