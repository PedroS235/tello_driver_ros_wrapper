import sys
import rospkg

import rospy
from std_msgs.msg import Empty, String
import smach
import time

import threading


keyboard_control_path = rospkg.RosPack().get_path("tello_controller") + "/scripts"
sys.path.append(keyboard_control_path)

from tello_controller import TelloController


class Keyboard_control(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["land", "hover", "done", "quit"])

        # ROS subscriptions
        rospy.Subscriber(
            "keyboard_input", String, self.get_keyboard_input_callback, queue_size=1
        )

        # ROS publishers
        self._land_pub = rospy.Publisher("tello/land", Empty, queue_size=1)

        self._tello = TelloController()

    def get_keyboard_input_callback(self, msg):
        self._keyboard_input = msg

    def run_keyboard_controller(self):
        self._tello.run()

    def execute(self, userdata):
        self._keyboard_input = None

        t = threading.Thread(self.run)
        t.start()

        self._tello.begin()

        rospy.loginfo("keyboard control enabled")

        waiting = True

        while waiting:
            if self._keyboard_input == String("l"):
                empty_message = Empty()

                time.sleep(2)

                self._land_pub.publish(empty_message)

                waiting = False

                return "land"

            elif self._keyboard_input == String("s"):
                waiting = False
                print("pressing s")

                return "hover"

            elif self._keyboard_input == String("^["):
                empty_message = Empty()

                time.sleep(2)

                self._land_pub.publish(empty_message)

                waiting = False

                return "quit"
