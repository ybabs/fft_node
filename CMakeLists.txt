cmake_minimum_required(VERSION 2.8.3)
project(serial_processing)

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
)

find_package( OpenCV REQUIRED )

add_message_files(
  FILES
  fft.msg
)

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
  ${catkin_INCLUDE_DIRS}
)

add_executable(serial_processing_node src/fft_node.cpp)

add_dependencies(serial_processing_node serial_processing_generate_messages_cpp)

target_link_libraries(serial_processing_node
  ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)
