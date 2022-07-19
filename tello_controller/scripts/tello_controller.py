#!/usr/bin/env python3
import rospy

from pynput import keyboard
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty


class TelloController:

    # - Topics
    tello_vel_cmd_unstamped_topic_name = "/tello/cmd_vel_unstamped"
    tello_takeoff_topic_name = "/tello/takeoff"
    tello_land_topic_name = "/tello/land"

    def __init__(self):
        self.key_pressed = {
            "th": 0,
            "right": 0,
            "forward": 0,
            "cw": 0,
        }
        self.speed = 0.5  # from 0 - 1
        self._keyboard_listener = keyboard.Listener(
            on_press=self.on_press, on_release=self.on_release
        )

    def begin(self):
        rospy.init_node("Tello Controller", anonymous=True)
        self.read_params()

        self.init_pub()

        self.rate = rospy.Rate(25)

        self._keyboard_listener.start()

    def init_pub(self):
        self.cmd_vel_pub = rospy.Publisher(
            self.tello_vel_cmd_unstamped_topic_name, Twist, queue_size=10
        )

        self._takeoff_pub = rospy.Publisher(
            self.tello_takeoff_topic_name, Empty, queue_size=1
        )

        self._land_pub = rospy.Publisher(
            self.tello_land_topic_name, Empty, queue_size=1
        )

    def read_params(self):
        self.tello_takeoff_topic_name = rospy.get_param(
            "/tello_driver_node/tello_takeoff_topic_name",
            default=self.tello_takeoff_topic_name,
        )
        self.tello_land_topic_name = rospy.get_param(
            "/tello_driver_node/tello_land_topic_name",
            default=self.tello_land_topic_name,
        )
        self.tello_vel_cmd_unstamped_topic_name = rospy.get_param(
            "/tello_driver_node/tello_vel_cmd_unstamped_topic_name",
            default=self.tello_vel_cmd_unstamped_topic_name,
        )

    def on_press(self, key):
        try:
            if key.char == "w":
                self.key_pressed["forward"] = self.speed
            if key.char == "s":
                self.key_pressed["forward"] = -self.speed
            if key.char == "d":
                self.key_pressed["right"] = self.speed
            if key.char == "a":
                self.key_pressed["right"] = -self.speed
            if key.char == "t":
                self._takeoff_pub.publish(Empty())
            if key.char == "l":
                self._land_pub.publish(Empty())
        except AttributeError:
            pass

        try:
            if key == key.up:
                self.key_pressed["th"] = self.speed
            if key == key.down:
                self.key_pressed["th"] = -self.speed
            if key == key.left:
                self.key_pressed["cw"] = -self.speed
            if key == key.right:
                self.key_pressed["cw"] = self.speed
        except AttributeError:
            pass

    def on_release(self, key):
        if key == keyboard.Key.esc:
            return False
        try:
            if key.char == "w":
                self.key_pressed["forward"] = 0
            if key.char == "s":
                self.key_pressed["forward"] = 0
            if key.char == "d":
                self.key_pressed["right"] = 0
            if key.char == "a":
                self.key_pressed["right"] = 0
        except AttributeError:
            pass

        try:
            if key == key.up:
                self.key_pressed["th"] = 0
            if key == key.down:
                self.key_pressed["th"] = 0
            if key == key.left:
                self.key_pressed["cw"] = 0
            if key == key.right:
                self.key_pressed["cw"] = 0
        except AttributeError:
            pass

    def run(self):
        while not rospy.is_shutdown():
            twist = Twist()

            twist.linear.x = self.key_pressed["forward"]
            twist.linear.y = self.key_pressed["right"]
            twist.linear.z = self.key_pressed["th"]

            twist.angular.z = self.key_pressed["cw"]
            self.cmd_vel_pub.publish(twist)
            self.rate.sleep()


if __name__ == "__main__":
    tello = TelloController()
    tello.begin()

    tello.run()
