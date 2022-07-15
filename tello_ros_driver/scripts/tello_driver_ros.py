#!/usr/bin/env python3

import rospy
import numpy as np
from tellopy import Tello
import tf2_ros
import time
import cv2
from tello_driver import TelloDriver


# - ROS messages imports
from geometry_msgs.msg import Twist

from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Image


class TelloDriverRos:

    # - Publishers
    video_stream_pub = None
    tello_pose_pub = None
    tello_flight_data_pub = None

    robot_vel_world_pub = None
    robot_vel_robot_pub = None

    robot_acc_world_pub = None
    robot_acc_robot_pub = None

    # - Subscribers
    robot_collision_sub = None
    flag_sub_tello_vel_cmd_unstamped = True
    tello_vel_cmd_unstamped_sub = None
    tello_vel_cmd_stamped_sub = None

    # - Topics
    tello_vel_cmd_unstamped_topic_name = "/tello/cmd_vel_unstamped"
    tello_vel_cmd_stamped_topic_name = "/tello/cmd_vel_stamped"

    # - TF2
    tello_frame = "tello_base_link"
    world_frame = "world"
    tf2_broadcaster = None

    # - Flags
    flag_sub_tello_vel_cmd_unstamped = True

    # - Timers
    pub_step_timer = None
    pub_step_time_interval = 0.02

    def __init__(self):
        self.driver = TelloDriver("tello03")

    def begin(self):
        rospy.init_node("Tello_driver_ros")
        self.init_pub()
        self.init_sub()
        self.init_timers()
        self.driver.begin()

    def init_pub(self):
        # self.tello_flight_data_pub = rospy.Publisher(
        #     self.tello_flight_data_topic_name, FlightData
        # )
        pass

    def init_sub(self):
        if self.flag_sub_tello_vel_cmd_unstamped:
            self.tello_vel_cmd_unstamped_sub = rospy.Subscriber(
                self.tello_vel_cmd_unstamped_topic_name,
                Twist,
                self.tello_vel_cmd_unstamped_callback,
            )

        # self.tello_vel_cmd_stamped_sub = rospy.Subscriber(
        #     self.tello_vel_cmd_stamped_topic_name,
        #     Twist,
        #     self.tello_vel_cmd_stamped_callback,
        # )

    def init_timers(self):
        self.pub_step_timer = rospy.Timer(
            rospy.Duration(self.pub_step_time_interval), self.pub_step_timer_callback
        )

    # +-----------+
    # | Callbacks |
    # +-----------+

    def tello_vel_cmd_unstamped_callback(self, twist_msg):
        lin_vel_cmd = np.zeros((3,), dtype=float)
        lin_vel_cmd[0] = twist_msg.linear.x
        lin_vel_cmd[1] = twist_msg.linear.y
        lin_vel_cmd[2] = twist_msg.linear.z

        alg_vel_cmd = np.zeros((3,), dtype=float)
        alg_vel_cmd[0] = twist_msg.angular.x
        alg_vel_cmd[1] = twist_msg.angular.y
        alg_vel_cmd[2] = twist_msg.angular.z

        self.driver.set_cmd_vel(lin_vel_cmd, alg_vel_cmd)

    def tello_vel_cmd_stamped_callback(self, twist_msg):
        lin_vel_cmd = np.zeros((3,), dtype=float)
        lin_vel_cmd[0] = twist_msg.twist.linear.x
        lin_vel_cmd[1] = twist_msg.twist.linear.y
        lin_vel_cmd[2] = twist_msg.twist.linear.z

        alg_vel_cmd = np.zeros((3,), dtype=float)
        alg_vel_cmd[0] = twist_msg.twist.angular.x
        alg_vel_cmd[1] = twist_msg.twist.angular.y
        alg_vel_cmd[2] = twist_msg.twist.angular.z

    def pub_step_timer_callback(self, timer_msg):
        pass

    def set_tello_vel_cmd(self, lin_vel_cmd, ang_vel_cmd):
        self.tello.forward(lin_vel_cmd[0] * 100)
        self.tello.right(lin_vel_cmd[1] * 100)
        self.tello.up(lin_vel_cmd[2] * 100)

        self.tello.clockwise(ang_vel_cmd[2] * 100)

    def run(self):
        rospy.spin()
        self.tello.quit()


if __name__ == "__main__":
    tello_driver = TelloDriverRos()
    tello_driver.begin()
    tello_driver.run()
