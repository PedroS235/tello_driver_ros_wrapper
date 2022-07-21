import rospy
from pynput import keyboard
from tello_msgs.msg import PressedKeys

"""
    You can refer to https://pynput.readthedocs.io/en/latest/keyboard.html
    in order to see how pynput works.
    In case you want to use another python keyboard listener feel free to do it 
    however you will need to change the current code eventually.
"""


class KeyboardListenerRos:
    # - Topics
    key_presses_topic_name = "/keyboard_listener/key_presses"

    # - Timers
    timer_pub = None
    timer_pub_rate = 25  # Hz

    # - Publishers
    key_presses_pub = None

    # - Keyboard
    keyboard_listener = None

    # - Message
    key_presses_message = None

    def __init__(self):
        rospy.init_node("Keyboard_listener")

    def begin(self):
        self.key_presses_message = PressedKeys()

        self._init_pub()
        self._init_timers()

        self.keyboard_listener = keyboard.Listener(
            on_press=self._on_key_press, on_release=self._on_key_release
        )

    def _init_pub(self):
        """
        Initilizes all ros publishers.
        """
        self.key_presses_pub = rospy.Publisher(
            self.key_presses_topic_name, PressedKeys, queue_size=1
        )

    def _init_timers(self):
        """
        Initilizes all ros timers.
        """
        self.timer_pub = rospy.Timer(
            1 / rospy.Duration(self.timer_pub_rate), self._timer_pub_callback()
        )

    def _timer_pub_callback(self, time_stamp):
        """
        This function will be called every few seconds (controlled by timer_pub_rate).

        Here, add the code to publish the <key_presses_message> using the <key_presses_pub>.
        The @time_stamp from the parameter is not required!
        """
        # -----------------------------------

        # - Write you code here

        # -----------------------------------

    def _on_key_press(self, key):
        """
        Here you can check for the keys that are being pressed and act upon it.
        For instance, if you press w, the message.w should be updated to true
        """
        # -----------------------------------

        # - Write you code here

        # -----------------------------------

    def _on_key_release(self, key):
        """
        Here you can check for the keys that are being released and act upon it.
        In this fuction should set all the keys to false.
        """
        # - Whenever esc is pressed, the keyboard listerner node will be shutdown
        if key == keyboard.Key.esc:
            rospy.signal_shutdown("Stopping keyboard listener")
            return False

        # -----------------------------------

        # - Write you code here

        # -----------------------------------

    def run(self):
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            return
