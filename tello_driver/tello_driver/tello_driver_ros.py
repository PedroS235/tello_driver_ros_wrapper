# - Begin of Imports

import cv_bridge
import rclpy
import numpy as np
import threading
import time
import av
import cv2

from tellopy import Tello
from cv_bridge import CvBridge
from utils.connect_to_wifi import connect_wifi_device

from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool, Header
from tello_msg.msg import FlightData, FlipControl

# - End Of Imports


class TelloDriverRos:

    # - Start Of Global Variables

    # - Publishers
    drone_vel_pub = None

    # - Subscribers
    drone_cmd_vel_sub = None
    drone_takeoff_sub = None
    drone_land_sub = None
    drone_flip_control_sub = None

    # - Topics Default Names
    drone_cmd_vel_topic_name = "/tello/cmd_vel"
    drone_takeoff_topic_name = "/tello/takeoff"
    drone_land_topic_name = "/tello/land"
    drone_image_topic_name = "/tello/camera_image_raw"
    drone_flight_data_topic_name = "/tello/fligh_data"

    # - Timers
    batt_percentage_timer = None
    batt_percentage_timer_freq = 0.2  # Hz
    general_purpose_timer = None
    general_purpose_timer_freq = 24  # Hz

    # - Drone WiFi Credentials
    wifi_ssid = None
    wifi_pw = None

    # - Flags
    auto_wifi_connection_flag = False

    # - Drone's battery percentage
    drone_curr_batt = 0

    # - Drone velocity
    drone_vel = {}

    # - End Of Global Variables

    def __init__(self):
        self._tello_driver = Tello()
        self._tello_driver.set_loglevel(self._tello_driver.LOG_WARN)

    def setup(self):
        pass

    def read_params(self):
        """
        ToDo:
            - figure out how to retrive the parameters
        """
        pass

    def run(self):
        rclpy.spin()

    # - Start Of Callbacks

    def takeoff_callback(self, msg):
        self._tello_driver.takeoff()

    def land_callback(self, msg):
        self._tello_driver.land()

    def batt_percentage_timer_callback(self, msg):
        print(f"[Info] - Battery percentage: {self.drone_curr_batt}%")

    def general_purpose_timer_callback(self, msg):
        self.build_drone_vel_msg()

    def flip_control_callback(self, msg):
        if msg.flip_forward:
            self._tello.flip_forward()
        elif msg.flip_backward:
            self._tello.flip_back()
        elif msg.flip_left:
            self._tello.flip_left()
        elif msg.flip_right:
            self._tello.flip_right()

    def drone_cmd_vel_stamped_callback(self, twist_msg):
        lin_cmd_vel = np.zeros((3,), dtype=float)
        lin_cmd_vel[0] = twist_msg.twist.linear.x
        lin_cmd_vel[1] = twist_msg.twist.linear.y
        lin_cmd_vel[2] = twist_msg.twist.linear.z

        ang_cmd_vel = np.zeros((3,), dtype=float)
        ang_cmd_vel[0] = twist_msg.twist.angular.x
        ang_cmd_vel[1] = twist_msg.twist.angular.y
        ang_cmd_vel[2] = twist_msg.twist.angular.z

        self.set_drone_cmd_vel(lin_cmd_vel, ang_cmd_vel)

    def fligh_data_callback(self, event, sender, data):
        flight_data = FlightData()

        # - Battery data
        flight_data.battery_low = data.battery_low
        flight_data.battery_lower = data.battery_lower
        flight_data.battery_percentage = data.battery_percentage
        flight_data.drone_battery_left = data.drone_battery_left
        # flight_data.drone_fly_time_left = data.drone_fly_time_left

        # =========================================================================

        # - States
        flight_data.battery_state = data.battery_state
        flight_data.camera_state = data.camera_state
        flight_data.electrical_machinery_state = data.electrical_machinery_state
        flight_data.down_visual_state = data.down_visual_state
        flight_data.gravity_state = data.gravity_state
        flight_data.imu_calibration_state = data.imu_calibration_state
        flight_data.imu_state = data.imu_state
        flight_data.power_state = data.power_state
        flight_data.pressure_state = data.pressure_state
        flight_data.wind_state = data.wind_state

        # =========================================================================

        # - Stats
        flight_data.drone_hover = data.drone_hover
        flight_data.em_open = data.em_open
        flight_data.em_sky = data.em_sky
        flight_data.em_ground = data.em_ground
        flight_data.factory_mode = data.factory_mode
        flight_data.fly_mode = data.fly_mode
        # flight_data.fly_time = data.fly_time
        flight_data.front_in = data.front_in
        flight_data.front_lsc = data.front_lsc
        flight_data.front_out = data.front_out

        # =========================================================================

        # - Sensors
        flight_data.fly_speed = data.fly_speed
        flight_data.east_speed = data.east_speed
        flight_data.ground_speed = data.ground_speed
        flight_data.height = data.height
        flight_data.light_strength = data.light_strength
        flight_data.north_speed = data.north_speed
        flight_data.temperature_height = data.temperature_height

        # =========================================================================

        # - Other
        flight_data.outage_recording = data.outage_recording
        flight_data.smart_video_exit_mode = data.smart_video_exit_mode
        # flight_data.throw_fly_timer = data.throw_fly_timer

        # =========================================================================

        # - WiFi
        flight_data.wifi_disturb = data.wifi_disturb
        flight_data.wifi_strength = data.wifi_strength

        # - Drone velocity
        self.drone_vel["x"] = flight_data.north_speed
        self.drone_vel["y"] = flight_data.east_speed
        self.drone_vel["z"] = flight_data.ground_speed

        self._current_battery_percentage = flight_data.battery_percentage

        # - Publish Flight data
        self._flight_data_pub.publish(flight_data)

    # - End Of Callbacks

    def build_drone_vel_msg(self):
        """
        ToDo: create a vel msg
        """
        pass

    def set_drone_cmd_vel(self, lin_cmd_vel, ang_cmd_vel):
        self.tello_driver.set_pitch(lin_cmd_vel[0])  # linear X value
        self.tello_driver.set_roll(-lin_cmd_vel[1])  # linear Y value
        self.tello_driver.set_throttle(lin_cmd_vel[2])  # linear Z value
        self.tello_driver.set_yaw(-ang_cmd_vel[2])  # angular Z value

    def start_video_threads(self):
        self._stop_request = threading.Event()
        self._video_thread = threading.Thread(target=self._video_worker_loop)
        self._video_thread.start()

    def drone_image_capture(self):
        video_stream = self.tello_driver.get_video_stream()

        container = av.open(video_stream)

        for frame in container.decode(video=0):
            if self._frame_skip > 0:
                self._frame_skip = self._frame_skip - 1

                continue

            # convert PyAV frame => PIL image => OpenCV image
            image = np.array(frame.to_image())

            image = cv2.resize(image, (480, 360), interpolation=cv2.INTER_LINEAR)

            # convert OpenCV image => ROS Image message
            image = CvBridge().cv2_to_imgmsg(image, "rgb8")

            self._image_pub.publish(image)

            # check for normal shutdown
            if self._stop_request.isSet():
                return

    def shutdown_routine(self):
        # force a landing
        self._tello.land()

        time.sleep(2)

        # shut down the drone
        self._tello.quit()

        # stop the video thread
        self._stop_request.set()
        self._video_thread.join(timeout=2)

    def connect_to_tello_wifi(self):
        if not connect_wifi_device(self.tello_ssid, self.tello_pw, verbose=False):
            print("[error] [Tello_driver] - Connection to drone unsuccessful!")
            rclpy.signal_shutdown("Not able to establish connection with Tello network")
        self._tello.connect()
        self._tello.wait_for_connection(5)
        print("[Info] - Connection to drone successfull")
