#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from pynput import keyboard


class Keyboard_listener(object):
    def __init__(self):
        # initialize ROS
        rospy.init_node("keyboard_listener", anonymous=False)

        # ROS publishers
        self._user_input_pub = rospy.Publisher("keyboard_input", String, queue_size=1)

        self._listener = keyboard.Listener(
            on_press=self.on_press_callback, on_release=self.on_release_callback
        )

        self._listener.start()

    def on_press_callback(self, key):
        try:
            if key.char in ("t", "l", "k", "s"):
                self._user_input_pub.publish(str(key).strip("'"))
        except AttributeError as ex:
                self._user_input_pub.publish("^[")

    def on_release_callback(self, key):
        try:
            if key == keyboard.Key.esc:
                rospy.signal_shutdown("keyboard listener shutdown")
                self._listener.stop()
                # stop listener
                return False
        except:
            pass


if __name__ == "__main__":
    keyboard_listener = Keyboard_listener()

    # spin until interrupted
    rospy.spin()
