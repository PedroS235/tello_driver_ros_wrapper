#! /usr/bin/env python3
import threading
import subprocess
import time
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Empty
import av
import cv2
import numpy as np

from tellopy import Tello
from cv_bridge import CvBridge
from tello_ros_msgs.msg import FlightData
from utils import connect_wifi_device as cwd


class TelloDriver(object):
    CONNECTION_ATTEMPTS = 0
    MAX_NUMBER_OF_ATTEMPTS = 10
    _frame_skip = 300

    tello_ssid = None
    tello_pw = None

    def __init__(self, tello_ssid, tello_pw=None):
        # we need to skip first 300 frame to avoid initial stream lag
        self._frame_skip = 300

        self.tello_ssid = tello_ssid
        self.tello_pw = tello_pw

        # ROS OpenCV bridge
        self._cv_bridge = CvBridge()

        # connect to the drone
        self._drone = Tello()

        rospy.on_shutdown(self.shutdown_routine)

    def begin(self):
        # ROS subscriptions
        # rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_callback, queue_size=1)
        rospy.Subscriber("takeoff", Empty, self.takeoff_callback, queue_size=1)
        rospy.Subscriber("land", Empty, self.land_callback, queue_size=1)

        self._drone.subscribe(self._drone.EVENT_FLIGHT_DATA, self.flight_data_handler)

        # ROS publishers
        self._image_pub = rospy.Publisher("image/image_raw", Image, queue_size=1)
        self._flight_data_pub = rospy.Publisher(
            "/tello/flight_data", FlightData, queue_size=1
        )

        self._connect_to_tello_network()

        self.start_video_threads()

    def start_video_threads(self):
        # start video thread
        self._stop_request = threading.Event()
        self._video_thread = threading.Thread(target=self.video_worker_loop)
        self._video_thread.start()

    def _connect_to_tello_network(self):
        # if not cwd.connect_device(self.tello_ssid, self.tello_pw):
        #     rospy.signal_shutdown("Not able to establish connection with Tello network")
        self._drone.connect()
        self._drone.wait_for_connection(5)

    def set_cmd_vel(self, lin_cmd_vel, ang_cmd_vel):
        self._drone.set_yaw(ang_cmd_vel[2] * 2)  # yaw_velocity (inverse w.r.t ROS)
        self._drone.set_pitch(lin_cmd_vel[0] * 2)  # forward/back ward velocity
        self._drone.set_roll(lin_cmd_vel[1] * 2)  # right/left
        self._drone.set_throttle(lin_cmd_vel[2] * 2)  # up/down velocity

    def takeoff_callback(self, msg):
        msg  # - just for not having linting errors
        self._drone.takeoff()

    def flight_data_handler(self, event, sender, data):
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

        # - Publish Flight data
        self._flight_data_pub.publish(flight_data)

    def land_callback(self, msg):
        msg  # - just for not having linting errors
        self._drone.land()

    def video_worker_loop(self):
        # get video stream, open with PyAV
        video_stream = self._drone.get_video_stream()

        container = av.open(video_stream)

        rospy.loginfo("video stream is starting")

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

    def shutdown_routine(self):
        # force a landing
        self._drone.land()

        time.sleep(2)

        # shut down the drone
        self._drone.quit()

        # stop the video thread
        self._stop_request.set()
        self._video_thread.join(timeout=2)
