cmake_minimum_required(VERSION 2.8.3)
project(ai)
set(CMAKE_CXX_FLAGS "-std=c++11 -g")
set(Boost_USE_MULTITHREADED OFF)

find_package(catkin REQUIRED COMPONENTS
  #message_generation
  roscpp
  rospy
  std_msgs
  message_generation
  peripherals
  navigation
  vision
  geometry_msgs
)

add_service_files(
  FILES
  start.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  peripherals
  navigation
  vision
)

catkin_package(
  CATKIN_DEPENDS message_generation roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(master src/master.cpp)
target_link_libraries(master ${catkin_LIBRARIES})
add_dependencies(master 
  ai_generate_messages_cpp 
  peripherals_generate_messages_cpp 
  navigation_generate_messages_cpp 
  vision_generate_messages_cpp 
  ${catkin_EXPORTED_TARGETS}
)
target_include_directories(master PUBLIC ${Boost_INCLUDE_DIRS})
