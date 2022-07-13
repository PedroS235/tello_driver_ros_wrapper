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

catkin_package(
  CATKIN_DEPENDS message_runtime
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs#   std_msgs
# )


## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  src/tello_driver_ros.py
  src/tello_controller.py
  src/test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

