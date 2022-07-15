cmake_minimum_required(VERSION 3.0.2)
project(tello_driver)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  std_msgs
  sensor_msgs
  message_generation
)

catkin_package(
    DEPENDS
    CATKIN_DEPENDS
    INCLUDE_DIRS
    LIBRARIES
)
