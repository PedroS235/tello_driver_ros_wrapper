import rospy

from tello_msgs.msg import FlipControl, PressedKeys
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist


class ControllerRos:
    # - Topics
    tello_vel_cmd_unstamped_topic_name = "/tello/cmd_vel_unstamped"
    tello_takeoff_topic_name = "/tello/takeoff"
    tello_land_topic_name = "/tello/land"
    tello_flip_control_topic_name = "/tello/flip_control"
    key_presses_topic_name = "/keyboard_listener/key_presses"

    def __init__(self):
        pass

    def begin(self):
        self._init_pub()
        self._init_sub()

    def _init_pub(self):
        self.cmd_vel_pub = rospy.Publisher(
            self.tello_vel_cmd_unstamped_topic_name, Twist, queue_size=1
        )

        self._takeoff_pub = rospy.Publisher(
            self.tello_takeoff_topic_name, Empty, queue_size=1
        )

        self._land_pub = rospy.Publisher(
            self.tello_land_topic_name, Empty, queue_size=1
        )

        self._flip_control_pub = rospy.Publisher(
            self.tello_flip_control_topic_name, FlipControl, queue_size=1
        )

    def _init_sub(self):
        self.key_presses_pub = rospy.Subscriber(
            self.key_presses_topic_name, PressedKeys, self.key_presses_callback
        )

    def key_presses_callback(sefl):
        pass

    def run(self):
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            return
