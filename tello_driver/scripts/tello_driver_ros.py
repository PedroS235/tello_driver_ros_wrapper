#!/usr/bin/env python3

import rospy
import numpy as np
import tf2_ros
from tello_driver import TelloDriver


# - ROS messages imports
from geometry_msgs.msg import Twist, TwistStamped, PoseStamped, Header
from std_msgs.msg import Bool


class TelloDriverRos:

    # - Publishers
    tello_pose_pub = None

    robot_vel_world_pub = None
    robot_vel_robot_pub = None

    robot_acc_world_pub = None
    robot_acc_robot_pub = None

    # - Subscribers
    robot_collision_sub = None
    flag_sub_tello_vel_cmd_unstamped = True
    tello_vel_cmd_unstamped_sub = None
    tello_vel_cmd_stamped_sub = None
    robot_collision_sub = None

    # - Topics
    tello_vel_cmd_unstamped_topic_name = "/tello/cmd_vel_unstamped"
    tello_vel_cmd_stamped_topic_name = "/tello/cmd_vel_stamped"
    tello_collision_topic_name = "/tello/is_colliding"
    tello_pose_topic_name = "/tello/pose"

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
        self.driver = TelloDriver()

    def begin(self):
        rospy.init_node("Tello_driver_ros")
        self.init_pub()
        self.init_sub()
        self.init_timers()
        self.driver.begin()

    def init_pub(self):
        self.tello_pose_pub = rospy.Publisher(
            self.tello_pose_topic_name, PoseStamped, queue_size=1
        )

    def init_sub(self):
        if self.flag_sub_tello_vel_cmd_unstamped:
            self.tello_vel_cmd_unstamped_sub = rospy.Subscriber(
                self.tello_vel_cmd_unstamped_topic_name,
                Twist,
                self.tello_vel_cmd_unstamped_callback,
            )

        self.tello_vel_cmd_stamped_sub = rospy.Subscriber(
            self.tello_vel_cmd_stamped_topic_name,
            TwistStamped,
            self.tello_vel_cmd_stamped_callback,
        )

        self.robot_collision_sub = rospy.Subscriber(
            self.tello_collision_topic_name, Bool, self.tello_collision_callback
        )

    def init_timers(self):
        self.pub_step_timer = rospy.Timer(
            rospy.Duration(self.pub_step_time_interval), self.pub_step_timer_callback
        )

    def read_params(self):
        self.tello_vel_cmd_unstamped_topic_name = rospy.get_param(
            "/tello_driver_node/tello_vel_cmd_unstamped_topic_name",
            default=self.tello_vel_cmd_unstamped_topic_name,
        )
        self.tello_vel_cmd_stamped_topic_name = rospy.get_param(
            "/tello_driver_node/tello_vel_cmd_stamped_topic_name",
            default=self.tello_vel_cmd_stamped_topic_name,
        )

        self.flag_sub_tello_vel_cmd_unstamped = rospy.get_param(
            "/tello_driver_node/flag_sub_tello_vel_cmd_unstamped",
            default=self.flag_sub_tello_vel_cmd_unstamped,
        )

        self.tello_frame = rospy.get_param(
            "/tello_driver_node/tello_frame_name", self.tello_frame
        )
        self.world_frame = rospy.get_param(
            "/tello_driver_node/tello_world_frame_name", self.world_frame
        )
        self.tello_collision_topic_name = rospy.get_param(
            "/tello_driver_node/tello_collision_topic_name",
            self.tello_collision_topic_name,
        )

    # +--------------------+
    # | Start of Callbacks |
    # +--------------------+

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

        self.driver.set_cmd_vel(lin_vel_cmd, alg_vel_cmd)

    def pub_step_timer_callback(self, timer_msg):
        self.tello_pose_msg_creation()

    def tello_collision_callback(self, msg):
        self.driver.set_flag_collision_detected(msg.data)

    # +------------------+
    # | End of Callbacks |
    # +------------------+

    def tello_pose_msg_creation(self):
        curr_time_stamp = rospy.Time()
        try:
            pose = self.buffer.lookup_transform(
                self.world_frame, self.robot_frame, curr_time_stamp
            )

            #
            robot_pose_msg = PoseStamped()

            robot_pose_msg.header = Header()
            robot_pose_msg.header.stamp = curr_time_stamp
            robot_pose_msg.header.frame_id = self.world_frame

            robot_pose_msg.pose.position.x = pose.transform.translation.x
            robot_pose_msg.pose.position.y = pose.transform.translation.y
            robot_pose_msg.pose.position.z = pose.transform.translation.z

            robot_pose_msg.pose.orientation.w = pose.transform.rotation.w
            robot_pose_msg.pose.orientation.x = pose.transform.rotation.x
            robot_pose_msg.pose.orientation.y = pose.transform.rotation.y
            robot_pose_msg.pose.orientation.z = pose.transform.rotation.z

            self.tello_pose_pub.publish(robot_pose_msg)
        except:
            pass

    def run(self):
        rospy.spin()
        self.tello.quit()


if __name__ == "__main__":
    tello_driver = TelloDriverRos()
    tello_driver.begin()
    tello_driver.run()
