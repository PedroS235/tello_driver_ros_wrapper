#!/usr/bin/env python3
import rospy
import rospkg
import sys

from tello_driver_ros import TelloDriverRos


def main():
    tello_driver_ros = TelloDriverRos()
    tello_driver_ros.begin()

    try:
        tello_driver_ros.run()
    except rospy.ROSInterruptException:
        pass

    return 0


if __name__ == "__main__":
    main()
