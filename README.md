# Introduction

Tello_ros_wrapper is a ros metapackage that containst multiple ros packages to control DJI Tello drone.
In it's core lies the package **tello_ros_driver** which is responsible to send the commands to the drone.
In addition, a package **tello_ros_controller** is provided to be able to communicate with the drone via keyboard.
As for the rest of the packages, the purpose of them are to make the tello autonomous by integrating obstacle avoidance for instance. (To be done)

***

## ROS topics

### Command velocity topics

There are 2 types of **command velocity** topics. One with time stamp and one without.

* command velocity unstamped topic name: `tello/cmd_vel_unstamped`
* command velocity stamped topic name: `tello/cmd_vel_stamped`

### Topics for triggering the takeoff and landing of the Tello

In order to inform the drone to land or takeoff, there are 2 additional topics which operate with empty messages.

* takeoff topic name: `tello/takeoff`
* land topic name: `tello/land`

***

## ROS parameters

* `tello_takeoff_topic_name` (default: *tello/takeoff*)
* `tello_land_topic_name` (default: *tello/land*)
* `tello_image_topic_name` (default: *tello/camera/image_raw*)
* `tello_flight_data_topic_name` (default: *tello/flight_data*)
* `tello_vel_cmd_unstamped_topic_name` (default: *tello/cmd_vel_unstamped*)
* `tello_vel_cmd_stamped_topic_name` (default: *tello/cmd_vel_stamped*)
* `tello_frame_name` (default: *tello_base_link*)
* `tello_world_frame_name` (default: *world*)
* `tello_ssid` (default: *world*)
* `tello_password` (default: )

***

## Dependencies

In order to be able to use tello_ros_wrapper metapackage, some dependencies are needed.

1. python3: `sudo apt install python3`
2. python3 pip: `sudo apt install python3-pip`
3. ROS noetic: [Installation guide](http://wiki.ros.org/noetic/Installation/Ubuntu)
4. python libraries: `pip install requirements.txt`
