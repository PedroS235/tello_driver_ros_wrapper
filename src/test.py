#!/usr/bin/env python3
import rospy

from pynput import keyboard
from geometry_msgs.msg import Twist


class TelloController:
    def __init__(self, topic_name='/cmd_vel'):
        rospy.init_node("Controller", anonymous=True)
        self.cmd_vel_pub = rospy.Publisher(topic_name, Twist, queue_size=10)
        self.key_pressed = {
            "th": 0,
            "right": 0,
            "forward": 0,
            "cw": 0,
            "takeoff": 0,
            "land": 0,
        }
        self.speed = 0.5
        self.keyboard_listener()

    def on_press(self, key):
        print("key pressed")
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
            if key == key.char == 'w':
                self.key_pressed["forward"] = self.speed
            if key == key.char == 's':
                self.key_pressed["forward"] = -self.speed
            if key == key.char == 'd':
                self.key_pressed["right"] = self.speed
            if key == key.char == 'a':
                self.key_pressed["right"] = -self.speed
            if key == key.char == 't':
                self.key_pressed["t"] = self.speed
            if key == key.char == 'l':
                self.key_pressed["l"] = self.speed

    def on_release(self, key):
        if key == keyboard.Key.esc:
            return False
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
            if key == key.char == 'w':
                self.key_pressed["forward"] = 0
            if key == key.char == 's':
                self.key_pressed["forward"] = 0
            if key == key.char == 'd':
                self.key_pressed["right"] = 0
            if key == key.char == 'a':
                self.key_pressed["right"] = 0
            if key == key.char == 't':
                self.key_pressed["t"] = 0
            if key == key.char == 'l':
                self.key_pressed["l"] = 0

    def keyboard_listener(self):
        with keyboard.Listener(
            on_press=self.on_press, on_release=self.on_release
        ) as listener:
            listener.join()

    def run(self):
        while not rospy.is_shutdown():
            twist = Twist()

            twist.linear.x = self.key_pressed['forward']
            twist.linear.y = self.key_pressed['right']
            twist.linear.z = self.key_pressed['th']

            twist.angular.z = self.key_pressed['cw']

            self.cmd_vel_pub.publish(twist)


if __name__ == "__main__":
    tello = TelloController()
    tello.run()
