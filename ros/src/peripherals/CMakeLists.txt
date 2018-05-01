cmake_minimum_required(VERSION 2.8.3)
project(peripherals)
set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  #message_generation
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
  DIRECTORY msg FILES depth_msg.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
  #LIBRARIES peripherals
  CATKIN_DEPENDS message_generation roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(depth_sensor_node src/depth_sensor_node.cpp)
target_link_libraries(depth_sensor_node ${catkin_LIBRARIES})
add_dependencies(depth_sensor_node peripherals_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
