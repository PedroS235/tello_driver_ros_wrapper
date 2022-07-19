#! /usr/bin/env python3

# - Start of imports

import threading
import time
import av
import cv2
import numpy as np

from tellopy import Tello
from cv_bridge import CvBridge

# - ROS Imports
import rospy

from sensor_msgs.msg import Image
from std_msgs.msg import Empty
from tello_msgs.msg import FlightData
from utils import connect_wifi_device as cwd

# - End of imports


class TelloDriver(object):
    _frame_skip = 300  # - skip the first 300 frames to avoid inital lag

    # Tello WiFi credentials
    tello_ssid = None
    tello_pw = None

    # - Subscribers
    tello_takeoff_sub = None
    tello_land_sub = None

    # - Topics
    tello_takeoff_topic_name = "/tello/takeoff"
    tello_land_topic_name = "/tello/land"
    tello_image_topic_name = "/tello/camera/image_raw"
    tello_flight_data_topic_name = "/tello_flight_data"

    # - Booleans
    _connect_to_tello_wifi_auto = True
    flag_collision_detected = False

    curr_time = time.time()

    def __init__(self):

        # ROS OpenCV bridge
        self._cv_bridge = CvBridge()

        # connect to the drone
        self._tello = Tello()
        self._tello.set_loglevel(self._tello.LOG_WARN)

        # - Setting shutdown routine
        rospy.on_shutdown(self._shutdown_routine)

    def begin(self):
        print("[info] [Tello_driver] - Initiating Tello driver")
        self.read_params()

        self._init_pub()
        self._init_sub()

        self._connect_to_tello_network()

        self._start_video_threads()

    def _init_pub(self):
        print(
            f"[info] [Tello_driver] - Publishing drone image to <{self.tello_image_topic_name}>"
        )
        self._image_pub = rospy.Publisher(
            self.tello_image_topic_name, Image, queue_size=1
        )
        print(
            f"[info] [Tello_driver] - Publishing drone flight data to <{self.tello_flight_data_topic_name}>"
        )
        self._flight_data_pub = rospy.Publisher(
            self.tello_flight_data_topic_name, FlightData, queue_size=1
        )

    def _init_sub(self):
        print(
            f"[info] [Tello_driver] - Subscribing to <{self.tello_takeoff_topic_name} topic>"
        )
        rospy.Subscriber(
            self.tello_takeoff_topic_name, Empty, self._takeoff_callback, queue_size=1
        )
        print(
            f"[info] [Tello_driver] - Subscribing to <{self.tello_land_topic_name} topic>"
        )
        rospy.Subscriber(
            self.tello_land_topic_name, Empty, self._land_callback, queue_size=1
        )
        self._tello.subscribe(self._tello.EVENT_FLIGHT_DATA, self._flight_data_handler)

    def read_params(self):
        print("[info] - Reading parameters")
        self.tello_takeoff_topic_name = rospy.get_param(
            "/tello_driver_node/tello_takeoff_topic_name",
            default=self.tello_takeoff_topic_name,
        )
        self.tello_land_topic_name = rospy.get_param(
            "/tello_driver_node/tello_land_topic_name",
            default=self.tello_land_topic_name,
        )
        self.tello_image_topic_name = rospy.get_param(
            "/tello_driver_node/tello_image_topic_name",
            default=self.tello_image_topic_name,
        )
        self.tello_flight_data_topic_name = rospy.get_param(
            "/tello_driver_node/tello_flight_data_topic_name",
            default=self.tello_flight_data_topic_name,
        )

        self.tello_ssid = rospy.get_param(
            "/tello_driver_node/tello_ssid", default=self.tello_ssid
        )

        self.tello_pw = rospy.get_param(
            "/tello_driver_node/tello_pw", default=self.tello_pw
        )
        self._connect_to_tello_wifi_auto = rospy.get_param(
            "/tello_driver_node/connect_to_tello_wifi_auto",
            default=self._connect_to_tello_wifi_auto,
        )

    def set_cmd_vel(self, lin_cmd_vel, ang_cmd_vel):
        # TODO: in case a collision is detected only allow to move away from the obstacle
        if self.flag_collision_detected:
            # - Linear cmd_vel
            self._tello.set_pitch(0)  # linear X value
            self._tello.set_roll(0)  # linear Y value
            self._tello.set_throttle(0)  # linear Z value

            # - Angular cmd_vel
            self._tello.set_yaw(0)  # angualr Z value
        else:
            # - Linear cmd_vel
            self._tello.set_pitch(lin_cmd_vel[0])  # linear X value
            self._tello.set_roll(lin_cmd_vel[1])  # linear Y value
            self._tello.set_throttle(lin_cmd_vel[2])  # linear Z value

            # - Angular cmd_vel
            self._tello.set_yaw(ang_cmd_vel[2])  # angualr Z value

    def set_flag_collision_detected(self, flag):
        self.flag_collision_detected = flag
        if self.flag_collision_detected:
            print("[info] [Tello_driver] - Coollision detected")

    # +--------------------+
    # | Start of Callbacks |
    # +--------------------+

    def _takeoff_callback(self, msg):
        msg  # - just for not having linting errors
        print("[info] [Tello_driver] - Taking off")
        self._tello.takeoff()

    def _land_callback(self, msg):
        msg  # - just for not having linting errors
        print("[info] [Tello_driver] - Landing")
        self._tello.land()

    def _flight_data_handler(self, event, sender, data):
        flight_data = FlightData()

        # - Battery data
        flight_data.battery_percentage = data.battery_percentage
        flight_data.battery_low = data.battery_low
        flight_data.battery_time_left = data.drone_battery_left

        # - IMU Data
        flight_data.height = data.height
        flight_data.north_speed = data.north_speed
        flight_data.east_speed = data.east_speed
        flight_data.ground_speed = data.ground_speed

        # - Drone States
        flight_data.camera_state = data.camera_state
        flight_data.imu_calibration_state = data.imu_calibration_state
        flight_data.imu_state = data.imu_state
        flight_data.battery_state = data.battery_state
        flight_data.power_state = data.power_state

        # - Drone Stats
        flight_data.fly_mode = data.fly_mode

        # - WiFi Data
        flight_data.wifi_strength = data.wifi_strength
        flight_data.wifi_disturb = data.wifi_disturb

        if time.time() - self.curr_time >= 5:
            print(
                f"[info] [Tello_driver] - Drone's battery percentage is {flight_data.battery_percentage}%"
            )
            self.curr_time = time.time()

        # - Publish Flight data
        self._flight_data_pub.publish(flight_data)

    # +------------------+
    # | End of Callbacks |
    # +------------------+

    def _start_video_threads(self):
        # start video thread
        self._stop_request = threading.Event()
        self._video_thread = threading.Thread(target=self._video_worker_loop)
        self._video_thread.start()

    def _video_worker_loop(self):
        # get video stream, open with PyAV
        video_stream = self._tello.get_video_stream()

        container = av.open(video_stream)

        print("[info] [Tello_driver] - video stream is starting")

        for frame in container.decode(video=0):
            if self._frame_skip > 0:
                self._frame_skip = self._frame_skip - 1

                continue

            # convert PyAV frame => PIL image => OpenCV image
            image = np.array(frame.to_image())

            # VIDEO RESOLUTION
            # original 960x720
            # divided by 2 480x360
            # divided by 3 320x240
            # divided by 4 240x180

            image = cv2.resize(image, (480, 360), interpolation=cv2.INTER_LINEAR)

            # convert OpenCV image => ROS Image message
            image = self._cv_bridge.cv2_to_imgmsg(image, "rgb8")

            self._image_pub.publish(image)

            # check for normal shutdown
            if self._stop_request.isSet():
                return

    def _shutdown_routine(self):
        # force a landing
        self._tello.land()

        time.sleep(2)

        # shut down the drone
        self._tello.quit()

        # stop the video thread
        self._stop_request.set()
        self._video_thread.join(timeout=2)

    def _connect_to_tello_network(self):
        print("[info] [Tello_driver] - Connecting to drone")
        if self._connect_to_tello_wifi_auto:
            if not cwd.connect_device(self.tello_ssid, self.tello_pw, verbose=False):
                print("[error] [Tello_driver] - Connection to drone unsuccessful!")
                rospy.signal_shutdown(
                    "Not able to establish connection with Tello network"
                )
        self._tello.connect()
        self._tello.wait_for_connection(5)
        print("[info] [Tello_driver] - Connection to drone successfull")
