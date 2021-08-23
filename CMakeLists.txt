cmake_minimum_required(VERSION 2.8.3)
project(porpdaq)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  std_msgs
  message_generation
  cv_bridge
  image_transport
)

find_package( OpenCV REQUIRED )

add_message_files(
  FILES
  fft.msg
)

#add_service_files(
 # FILES
  #DaqCommand.srv
#)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    serial
    std_msgs
    message_runtime
    cv_bridge
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(porpdaq_node src/fft_node.cpp)

add_dependencies(porpdaq_node porpdaq_generate_messages_cpp)

target_link_libraries(porpdaq_node
  ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)
