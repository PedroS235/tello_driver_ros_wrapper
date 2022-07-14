#!/usr/bin/env python3
import rospy
import threading

from pynput import keyboard
from geometry_msgs.msg import Twist


class TelloController:
    def __init__(self, topic_name="/tello_cmd_vel"):
        self.key_pressed = {
            "th": 0,
            "right": 0,
            "forward": 0,
            "cw": 0,
            "takeoff": 0,
            "land": 0,
        }
        self.speed = 0.5
        keyboard_thread = threading.Thread(target=self.keyboard_listener)
        keyboard_thread.start()

    def begin(self):
        rospy.init_node("Controller", anonymous=True)
        self.cmd_vel_pub = rospy.Publisher("/tello_cmd_vel", Twist, queue_size=10)
        self.rate = rospy.Rate(10)

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
                self.key_pressed["t"] = self.speed
            if key.char == "l":
                self.key_pressed["l"] = self.speed
        except AttributeError:
            pass

        try:
            if key == key.up:
                self.key_pressed["th"] = self.speed
            if key == key.down:
                self.key_pressed["th"] = -self.speed
            if key == key.left:
                self.key_pressed["cw"] = self.speed
            if key == key.right:
                self.key_pressed["cw"] = -self.speed
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
            if key.char == "t":
                self.key_pressed["t"] = 0
            if key.char == "l":
                self.key_pressed["l"] = 0
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

    def keyboard_listener(self):
        with keyboard.Listener(
            on_press=self.on_press, on_release=self.on_release
        ) as listener:
            listener.join()

        listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        listener.start()

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

    try:
        tello.run()
    except rospy.ROSInterruptException:
        self.keyboard_thread.join()
