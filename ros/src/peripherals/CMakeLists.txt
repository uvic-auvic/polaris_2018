cmake_minimum_required(VERSION 2.8.3)
project(peripherals)
set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  #message_generation
  roscpp
  rospy
  std_msgs
  message_generation
  serial
  monitor
  geometry_msgs
)

add_service_files(
   FILES
   motor.srv
   motors.srv
)

add_message_files(
  FILES
  depth.msg
  powerboard.msg
  imu.msg
  orientation.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
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

find_package(serial REQUIRED)

add_executable(depth_sensor src/depth_sensor_node.cpp)
target_link_libraries(depth_sensor ${catkin_LIBRARIES})
add_dependencies(depth_sensor peripherals_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

add_executable(motor_controller src/motor_controller.cpp)
target_link_libraries(motor_controller ${catkin_LIBRARIES} ${serial_LIBRARIES})
add_dependencies(motor_controller monitor_generate_messages_cpp peripherals_generate_messages_cpp ${catkin_EXPORTED_TARGETS})


add_executable(imu src/imu.cpp)
target_link_libraries(imu ${catkin_LIBRARIES} ${serial_LIBRARIES})
add_dependencies(imu monitor_generate_messages_cpp peripherals_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
