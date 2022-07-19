cmake_minimum_required(VERSION 3.0.2)
project(optitrack_environment)

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation

  # - Messages
  std_msgs
  geometry_msgs
)


catkin_package(
    DEPENDS
    CATKIN_DEPENDS
    INCLUDE_DIRS
    LIBRARIES
)
