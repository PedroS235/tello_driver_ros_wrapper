import rospy
from std_msgs.msg import Empty, String
import smach
import time

class Landed(smach.State):

  def __init__(self):
    smach.State.__init__(self, outcomes=['takeoff', 'quit'])

    #ROS subscriptions
    rospy.Subscriber('keyboard_input', String, self.get_keyboard_input_callback, queue_size = 1)

    #ROS publishers
    self._takeoff_pub = rospy.Publisher('takeoff', Empty, queue_size = 1)

  def get_keyboard_input_callback(self, msg):
    self._keyboard_input = msg

  def execute(self, userdata):
    self._keyboard_input = None

    rospy.loginfo('drone is landed')

    waiting = True

    while waiting:
      if self._keyboard_input == String("t"):
        empty_message = Empty()

        time.sleep(1)

        self._takeoff_pub.publish(empty_message)

        waiting = False

        return 'takeoff'

      elif self._keyboard_input == String("q"):
        waiting = False

        return 'quit'