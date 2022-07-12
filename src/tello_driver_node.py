#!/usr/bin/env python3

import rospy
from tellopy import Tello

# - ROS messages imports
from geometry_msgs.msg import Twist


class TelloDriver:
    def __init__(self, topic_name='/cmd_vel'):
        self.subscriber = rospy.Subscriber(
            topic_name, Twist, self.command_velocity_callback
        )
        self.tello = Tello()

    def begin(self):
        rospy.init_node("Tello_driver")
        # self.tello.connect()
        # self.tello.wait_for_connection()

    def print_command_controller(self, linear_vector, angular_vector):
        rospy.loginfo(f"linear x: {linear_vector[0]}")
        rospy.loginfo(f"linear y: {linear_vector[1]}")
        rospy.loginfo(f"linear z: {linear_vector[2]}")
        rospy.loginfo(f"angular x: {angular_vector[0]}")
        rospy.loginfo(f"angular y: {angular_vector[1]}")
        rospy.loginfo(f"angular z: {angular_vector[2]}")

    def command_velocity_callback(self, msg):
        linear_vector = [msg.linear.x, msg.linear.y, msg.linear.z]
        angular_vector = [msg.angular.x, msg.angular.y, msg.angular.z]

        self.print_command_controller(linear_vector, angular_vector)

        # self.drone_controller(linear_vector, angular_vector)

    def drone_controller(self, linear_vector, angular_vector):
        linear_vector
        angular_vector

        self.tello.forward(linear_vector[0] * 100)
        self.tello.backward(linear_vector[1])
        self.tello.right(linear_vector[2])
        self.tello.clockwise()
        self.tello.up()

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    tello_driver = TelloDriver()
    tello_driver.begin()
    tello_driver.run()
