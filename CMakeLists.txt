cmake_minimum_required(VERSION 3.0.2)
project(tello_ros_wrapper)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  std_msgs
  sensor_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  FlightData.msg
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
    DEPENDS
    CATKIN_DEPENDS message_runtime
    INCLUDE_DIRS
    LIBRARIES
)
