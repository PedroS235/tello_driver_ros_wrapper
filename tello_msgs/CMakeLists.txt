cmake_minimum_required(VERSION 3.0.2)
project(tello_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  rospy
  std_msgs
  sensor_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  FlightData.msg
  FlipControl.msg
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
    DEPENDS
    CATKIN_DEPENDS
)
