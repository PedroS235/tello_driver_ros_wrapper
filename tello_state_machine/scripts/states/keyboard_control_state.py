import rospy
from std_msgs.msg import Empty, String
import smach
import time


class Keyboard_control(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["land", "hover", "done"])
        # ROS subscriptions
        rospy.Subscriber(
            "keyboard_input", String, self.get_keyboard_input_callback, queue_size=1
        )

        # ROS publishers
        self._land_pub = rospy.Publisher("land", Empty, queue_size=1)

        # launch the keyboard controller

    def get_keyboard_input_callback(self, msg):
        self._keyboard_input = msg

    def execute(self, userdata):
        self._keyboard_input = None

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
