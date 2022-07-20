#!/usr/bin/env python3

import rospy
import smach
from states.landed_state import Landed
from states.hovering_state import Hovering
from states.keyboard_control_state import Keyboard_control


class Sate_Machine(object):
    def __init__(self):
        # initialize ROS
        rospy.init_node("state_machine", anonymous=False)

        # create a SMACH state machine
        self._state_machine = smach.StateMachine(outcomes=["done"])

        # open the container
        with self._state_machine:
            # add states to the container
            smach.StateMachine.add(
                "LANDED", Landed(), transitions={"takeoff": "HOVERING", "quit": "done"}
            )
            smach.StateMachine.add(
                "HOVERING",
                Hovering(),
                transitions={"land": "LANDED", "keyboard": "KEYBOARD_CONTROL", "quit": "done"},
            )
            smach.StateMachine.add(
                "KEYBOARD_CONTROL",
                Keyboard_control(),
                transitions={"land": "LANDED", "hover": "HOVERING", "quit": "done"},
            )

        outcome = self._state_machine.execute()


if __name__ == "__main__":
    drone_intelligence = Sate_Machine()
