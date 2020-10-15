cmake_minimum_required(VERSION 2.8.3)
project(motors)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  maccepa_msgs
)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS roscpp maccepa_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(dynamixel_control src/dynamixel_control.cpp)

target_link_libraries(dynamixel_control
  ${catkin_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/lib/libdxl_x64_cpp.so
)
