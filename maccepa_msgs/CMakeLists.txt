cmake_minimum_required(VERSION 2.8.3)
project(maccepa_msgs)

add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

add_message_files(
  FILES
  MotorsPose.msg
  MotorsVel.msg
  GoalState.msg
  EncRawData.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS 
  message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
