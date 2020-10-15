cmake_minimum_required(VERSION 2.8.3)
project(gazebo_joint_state_publisher_plugin)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
)

find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS} ${catkin_LIBRARY_DIRS})
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
	gazebo_joint_state_publisher_plugin
  DEPENDS
    roscpp
    gazebo_ros
)

add_library(gazebo_joint_state_publisher_plugin src/gazebo_joint_state_publisher_plugin.cpp)
target_link_libraries(gazebo_joint_state_publisher_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})


