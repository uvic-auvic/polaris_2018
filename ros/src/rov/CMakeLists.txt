cmake_minimum_required(VERSION 2.8.3)
project(rov)
set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
)

## Generate messages in the 'msg' folder
 add_message_files(
   DIRECTORY msg FILES joystick_message.msg
 )


## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
  #LIBRARIES rov
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(joystick_node src/joystick_node.cpp src/joystick.cpp)
target_link_libraries(joystick_node ${catkin_LIBRARIES})
add_dependencies(joystick_node rov_generate_messages_cpp ${catkin_EXPORTED_TARGETS})



