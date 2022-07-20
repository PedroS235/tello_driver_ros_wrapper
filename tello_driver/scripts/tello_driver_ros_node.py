#!/usr/bin/env python3
import rospy
import rospkg
import sys


path = rospkg.RosPack().get_path("tello_controller") + "/scripts"
sys.path.append(path)
from tello_driver_ros import TelloDriverRos
import tello_controller


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
