cmake_minimum_required(VERSION 2.8.3)
project(navigation)
set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
  peripherals
  geometry_msgs
)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   rpy.msg
   nav.msg
   joystick.msg
   keyboard.msg
 )


## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
 )

catkin_package(
  #LIBRARIES rov
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(rov src/rov.cpp)
target_link_libraries(rov ${catkin_LIBRARIES})
add_dependencies(rov navigation_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

add_executable(thrustmap src/thrustmap.cpp)
target_link_libraries(thrustmap ${catkin_LIBRARIES})
add_dependencies(thrustmap navigation_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
