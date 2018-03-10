cmake_minimum_required(VERSION 2.8.3)
project(depth_sensor)
set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
)

add_message_files(
  FILES
  depth_msg.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES depth_sensor
  CATKIN_DEPENDS message_generation roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(depth_sensor_node src/depth_sensor_node.cpp)
target_link_libraries(depth_sensor_node ${catkin_LIBRARIES})
add_dependencies(depth_sensor_node ${depth_sensor_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(depth_test_node src/depth_test_node.cpp)
target_link_libraries(depth_test_node ${catkin_LIBRARIES})
add_dependencies(depth_sensor_node ${depth_sensor_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})