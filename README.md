# Introduction

Tello **driver_ros_wrapper** is a simple ROS metapackage that allows to control DJI Tello drone via ROS topics.

In order to communicate with the drone, the python library `tellopy` is being used which can be found [here](https://github.com/hanyazou/TelloPy).
At this moment, the pose of the drone is being published by the help of an optitrack system, where it is reading the information from the TF frame _world_ to _robot_base_link_.
In case there is not such transform available no drone pose will be published.

In this ROS metapackage, there lies 2 ROS packages, the **tello_driver** and the **tello_msgs**.
The **tello_driver** is the core, where the **tello_msgs** only contains custom messages specific to Tello, such as the flight data.

---

## The available ROS topics

- `tello_takeoff_topic_name`
  - **Default:** `/tello/takeoff`
  - **Purpose:** Trigger the drone to takeoff.
  - **Use:** Send an empty msg.
- `tello_land_topic_name`
  - **Default:** `/tello/lang`
  - **Purpose:** Trigger the drone to land.
  - **Use:** Send an empty msg.
- `tello_image_topic_name`
  - **Default:** `/tello/camera/image_raw`
  - **Purpose:** Publish the camera image to a ROS topic.
- `tello_flight_data_topic_name`
  - **Default:** `/tello/flight_data`
  - **Purpose:** Publish the drone flight data to a ROS topic, such as the battery percentage.
- `tello_vel_cmd_stamped_topic_name`
  - **Default:** `/tello/cmd_vel_stamped`
  - **Purpose:** Topic to control the drones velocity.
- `tello_collision_topic_name`
  - **Default:** `/tello/collision_detection`
  - **Purpose:** Topic that tells the tello driver if it is about to collide. If so controls will be blocked.
- `tello_flip_control_topic_name`
  - **Default:** `/tello/flip_control`
  - **Purpose:** Triggers the drone to perform a flip. Refer to `tello_msgs/msg/FlipControl` to see the type of the message to send.
- `tello_cmd_vel_topic_name`
  - **Default:** `/tello/cmd_vel`
  - **Purpose:** Publish the drone velocity.

---

## ROS parameters

- `tello_frame_name`
  - **Default:** `robot_base_link`
- `tello_world_frame_name`
  - **Default:** `world`
- `connect_to_tello_wifi_auto`
  - **Default:** `true`
- `tello_ssid`
  - Only required if _connect_to_tello_wifi_auto_ is set to true
- `tello_password`
  - **Default:** None

---

## How to run the Tello driver

**Note: you should place this repository inside the `catkin_ws/src` folder!**

1. go to your _catkin_ws_ and do `catkin_make` and source the `catkin_ws/devel` setup file (only needs to be done once)
2. To launch the tello driver and make it connect to the tello's wifi automatically, run this command: `roslaunch tello_driver tello_driver_ros.launch tello_ssid:=<the ssid of tello's wifi>`
3. To launch the tello driver and manually connect to the tello's wifi, run this command: `roslaunch tello_driver tello_driver_ros.launch connect_to_tello_wifi_auto:=false`

## Dependencies

In order to be able to use tello_ros_wrapper metapackage, some dependencies are needed.

1. python3: `sudo apt install python3`
2. python3 pip: `sudo apt install python3-pip`
3. ROS noetic: [Installation guide](http://wiki.ros.org/noetic/Installation/Ubuntu)
4. python libraries: `pip install requirements.txt`
