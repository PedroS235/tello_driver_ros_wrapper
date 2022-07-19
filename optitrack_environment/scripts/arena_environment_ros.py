import yaml
import rospy

import tf2_ros
import rospkg

from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header


class ArenaObstaclesROS:
    pck_path = rospkg.RosPack().get_path("optitrack_environment")

    obstacles_sizes = None
    obstacles_info_file_path = pck_path + "/config/arena_obstacles.yaml"
    obstacles_msg = MarkerArray()

    # TF
    tf2_listener = None
    tf2_buffer = None
    world_frame_name = "world"

    # Publishers
    obstacles_pub = None

    # Timers
    pub_timer_interval = 0.01
    pub_timer = None

    # Topics
    obstacles_info_topic_name = "/optitrack/obstacles"

    def __init__(self):
        rospy.init_node("Optitrack_obstacles_node")

    def begin(self):
        self.tf2_buffer = tf2_ros.Buffer()
        self.tf2_listener = tf2_ros.TransformListener(self.tf2_buffer)
        self.obstacles_sizes = self.get_obstacles_sizes(self.obstacles_info_file_path)
        self._init_pub()
        self._init_timers()

    def _init_pub(self):
        self.obstacles_pub = rospy.Publisher(
            self.obstacles_info_topic_name, MarkerArray, queue_size=1
        )

    def _init_timers(self):
        self.pub_timer = rospy.Timer(
            rospy.Duration(self.pub_timer_interval), self.timer_pub_callback
        )

    def run(self):
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            return False
        return True

    def timer_pub_callback(self, time_stamp):
        self.obstacles_msg.markers = []
        if self.obstacles_sizes:
            for i, sizes in enumerate(self.obstacles_sizes):
                obst_pos = self.read_tf_data("world", f"obst{i+1}", rospy.Time())
                if obst_pos is None:
                    continue

                obstacle_i = Marker()

                obstacle_i.header = Header()
                obstacle_i.header.stamp = rospy.Time()
                obstacle_i.header.frame_id = self.world_frame_name

                obstacle_i.ns = "Optitrack obstacle"
                obstacle_i.id = i

                obstacle_i.action = 0

                obstacle_i.type = 3  # circle shape

                obstacle_i.pose.position.x = obst_pos.translation.x
                obstacle_i.pose.position.y = obst_pos.translation.y
                obstacle_i.pose.position.z = obst_pos.translation.z

                obstacle_i.pose.orientation.w = 1.0
                obstacle_i.pose.orientation.x = 0.0
                obstacle_i.pose.orientation.y = 0.0
                obstacle_i.pose.orientation.z = 0.0

                obstacle_i.scale.x = sizes["x"]
                obstacle_i.scale.y = sizes["y"]
                obstacle_i.scale.z = sizes["z"]

                obstacle_i.color.r = 0.0
                obstacle_i.color.g = 1.0
                obstacle_i.color.b = 0.0
                obstacle_i.color.a = 0.3

                obstacle_i.lifetime = rospy.Duration(self.pub_timer_interval)

                self.obstacles_msg.markers.append(obstacle_i)

        self.obstacles_pub.publish(self.obstacles_msg)

    def read_tf_data(self, parent, child, time):
        try:
            return self.tf2_buffer.lookup_transform(
                self.world_frame_name, child, time
            ).transform
        except tf2_ros.LookupException:
            return None

    def get_obstacles_sizes(self, path):
        print("Reading obstacles sizes")
        with open(path) as file:
            content = yaml.full_load(file)
        sizes = []
        for i, obst in enumerate(content):
            size = obst[f"obst{i+1}"]["size"]
            dict = {}
            dict["x"] = size[0]
            dict["y"] = size[1]
            dict["z"] = size[2]
            sizes.append(dict)
        print("Finished reading")
        print(sizes)
        return sizes
