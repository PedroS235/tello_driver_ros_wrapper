#!/usr/bin/env python3
import rospy

from std_msgs.msg import String


class Subscriber:
    def __init__(self):
        self.subscriber = rospy.Subscriber("Tello_Pub", String, self.msg_callback)

    def begin(self):
        rospy.init_node("Subscriber_node")

    def msg_callback(self, data):
        rospy.loginfo(
            f"[{rospy.get_caller_id()} - Received the following data: {data.data}"
        )

    def listener(self):
        rospy.spin()


if __name__ == "__main__":
    sub = Subscriber()

    sub.begin()

    sub.listener()
