#!/usr/bin/env python

"""
import rospy
import numpy as np
from scipy.ndimage import distance_transform_edt
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
import tf

class CostmapGenerator:
    def __init__(self):
        rospy.init_node("costmap_generator", anonymous=True)

        # Subscribers
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.lidar_callback)

        # Publisher
        self.costmap_pub = rospy.Publisher("/inflated_costmap", OccupancyGrid, queue_size=10, latch=True)

        # Internal state
        self.map_data = None
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0.05
        self.map_origin = (0, 0)

        # Parameters
        self.inflation_radius = rospy.get_param("~inflation_radius", 0.3)
        self.lidar_decay_time = rospy.get_param("~lidar_decay_time", 1.0)  # Decay LiDAR obstacles
        
        # TF Listener
        self.tf_listener = tf.TransformListener()

        # Store LiDAR obstacles as dictionary (key: (gx, gy), value: timestamp)
        self.lidar_obstacles = {}

        # Timer for costmap updates
        self.update_timer = rospy.Timer(rospy.Duration(0.5), self.update_and_publish_costmap)

    def map_callback(self, msg):
        rospy.loginfo("Map received.")
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        
    def lidar_callback(self, msg):
        try:
            (trans, rot) = self.tf_listener.lookupTransform("/map", msg.header.frame_id, rospy.Time(0))
            lidar_x, lidar_y = trans[0], trans[1]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("TF transform failed for LiDAR")
            return

        angle = msg.angle_min
        current_time = rospy.Time.now().to_sec()
        
        for r in msg.ranges:
            if 0.1 < r < msg.range_max:
                wx = lidar_x + r * np.cos(angle)
                wy = lidar_y + r * np.sin(angle)
                gx, gy = self.world_to_grid(wx, wy)
                if 0 <= gx < self.map_width and 0 <= gy < self.map_height:
                    self.lidar_obstacles[(gx, gy)] = current_time
            angle += msg.angle_increment

    def world_to_grid(self, wx, wy):
        gx = int((wx - self.map_origin[0]) / self.map_resolution)
        gy = int((wy - self.map_origin[1]) / self.map_resolution)
        return gx, gy

    def generate_inflated_costmap(self):
        costmap = np.where(self.map_data == -1, -1, 0)
        costmap[self.map_data == 100] = 100  # Static obstacles
        
        # Remove outdated LiDAR obstacles
        current_time = rospy.Time.now().to_sec()
        self.lidar_obstacles = {k: v for k, v in self.lidar_obstacles.items() if current_time - v < self.lidar_decay_time}

        # Add LiDAR obstacles
        for (gx, gy), _ in self.lidar_obstacles.items():
            if 0 <= gx < self.map_width and 0 <= gy < self.map_height:
                costmap[gy, gx] = 100
        
        return self.apply_inflation(costmap)

    def apply_inflation(self, costmap):
        binary_map = np.zeros_like(costmap, dtype=np.uint8)
        binary_map[costmap == 100] = 1
        distances = distance_transform_edt(1 - binary_map) * self.map_resolution
        inflated_costmap = np.copy(costmap)
        inflated_costmap[(distances <= self.inflation_radius) & (costmap != -1)] = 100
        return inflated_costmap

    def update_and_publish_costmap(self, event):
        if self.map_data is None:
            return
        costmap = self.generate_inflated_costmap()
        self.publish_costmap(costmap)

    def publish_costmap(self, costmap):
        costmap_msg = OccupancyGrid()
        costmap_msg.header.stamp = rospy.Time.now()
        costmap_msg.header.frame_id = "map"
        costmap_msg.info.resolution = self.map_resolution
        costmap_msg.info.width = self.map_width
        costmap_msg.info.height = self.map_height
        costmap_msg.info.origin.position.x = self.map_origin[0]
        costmap_msg.info.origin.position.y = self.map_origin[1]
        costmap_msg.info.origin.orientation.w = 1.0
        costmap_msg.data = costmap.flatten().tolist()
        self.costmap_pub.publish(costmap_msg)

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    generator = CostmapGenerator()
    generator.run()


    







#!/usr/bin/env python
import rospy
import numpy as np
from scipy.ndimage import distance_transform_edt
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
import tf

class CostmapGenerator:
    def __init__(self):
        rospy.init_node("costmap_generator", anonymous=True)

        # Subscribers
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.lidar_callback)

        # Publisher
        self.costmap_pub = rospy.Publisher("/local_costmap", OccupancyGrid, queue_size=10, latch=True)

        # Internal state
        self.map_data = None
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0.05
        self.map_origin = (0, 0)

        # Parameters
        self.inflation_radius = rospy.get_param("~inflation_radius", 0.3)
        self.lidar_decay_time = rospy.get_param("~lidar_decay_time", 1.0)
        self.square_center = rospy.get_param("~square_center", [0, 0])
        self.square_size = rospy.get_param("~square_size", 5.0)
        
        # TF Listener
        self.tf_listener = tf.TransformListener()

        # Store LiDAR obstacles as dictionary (key: (gx, gy), value: timestamp)
        self.lidar_obstacles = {}

        # Timer for costmap updates
        self.update_timer = rospy.Timer(rospy.Duration(0.5), self.update_and_publish_costmap)

    def map_callback(self, msg):
        rospy.loginfo("Map received.")
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin = (msg.info.origin.position.x, msg.info.origin.position.y)

    def lidar_callback(self, msg):
        try:
            (trans, rot) = self.tf_listener.lookupTransform("/map", msg.header.frame_id, rospy.Time(0))
            lidar_x, lidar_y = trans[0], trans[1]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("TF transform failed for LiDAR")
            return

        angle = msg.angle_min
        current_time = rospy.Time.now().to_sec()
        
        for r in msg.ranges:
            if 0.1 < r < msg.range_max:
                wx = lidar_x + r * np.cos(angle)
                wy = lidar_y + r * np.sin(angle)
                gx, gy = self.world_to_grid(wx, wy)
                if self.is_within_square(gx, gy):
                    self.lidar_obstacles[(gx, gy)] = current_time
            angle += msg.angle_increment

    def world_to_grid(self, wx, wy):
        gx = int((wx - self.map_origin[0]) / self.map_resolution)
        gy = int((wy - self.map_origin[1]) / self.map_resolution)
        return gx, gy

    def is_within_square(self, gx, gy):
        square_gx, square_gy = self.world_to_grid(*self.square_center)
        half_size = int(self.square_size / (2 * self.map_resolution))
        return (square_gx - half_size <= gx <= square_gx + half_size and
                square_gy - half_size <= gy <= square_gy + half_size)

    def generate_inflated_costmap(self):
        costmap = np.full((self.map_height, self.map_width), -1)
        
        for y in range(self.map_height):
            for x in range(self.map_width):
                if self.is_within_square(x, y):
                    if self.map_data[y, x] == 100:
                        costmap[y, x] = 100
                    else:
                        costmap[y, x] = 0
        
        # Remove outdated LiDAR obstacles
        current_time = rospy.Time.now().to_sec()
        self.lidar_obstacles = {k: v for k, v in self.lidar_obstacles.items() if current_time - v < self.lidar_decay_time}

        # Add LiDAR obstacles
        for (gx, gy), _ in self.lidar_obstacles.items():
            if self.is_within_square(gx, gy):
                costmap[gy, gx] = 100
        
        return self.apply_inflation(costmap)

    def apply_inflation(self, costmap):
        binary_map = np.zeros_like(costmap, dtype=np.uint8)
        binary_map[costmap == 100] = 1
        distances = distance_transform_edt(1 - binary_map) * self.map_resolution
        inflated_costmap = np.copy(costmap)
        inflated_costmap[(distances <= self.inflation_radius) & (costmap != -1)] = 100
        return inflated_costmap

    def update_and_publish_costmap(self, event):
        if self.map_data is None:
            return
        costmap = self.generate_inflated_costmap()
        self.publish_costmap(costmap)

    def publish_costmap(self, costmap):
        costmap_msg = OccupancyGrid()
        costmap_msg.header.stamp = rospy.Time.now()
        costmap_msg.header.frame_id = "map"
        costmap_msg.info.resolution = self.map_resolution
        costmap_msg.info.width = self.map_width
        costmap_msg.info.height = self.map_height
        costmap_msg.info.origin.position.x = self.map_origin[0]
        costmap_msg.info.origin.position.y = self.map_origin[1]
        costmap_msg.info.origin.orientation.w = 1.0
        costmap_msg.data = costmap.flatten().tolist()
        self.costmap_pub.publish(costmap_msg)

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    generator = CostmapGenerator()
    generator.run()





"""










#!/usr/bin/env python
import rospy
import numpy as np
from scipy.ndimage import distance_transform_edt
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
import tf

class CostmapGenerator:
    def __init__(self):
        rospy.init_node("costmap_generator", anonymous=True)

        # Subscribers
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.lidar_callback)

        # Publisher
        self.costmap_pub = rospy.Publisher("/local_costmap", OccupancyGrid, queue_size=10, latch=True)

        # Internal state
        self.map_data = None
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0.05
        self.map_origin = (0, 0)

        # Parameters
        self.inflation_radius = rospy.get_param("~inflation_radius", 0.3)
        self.lidar_decay_time = rospy.get_param("~lidar_decay_time", 1.0)
        self.square_size = rospy.get_param("~square_size", 5.0)  # Keep this constant

        # TF Listener
        self.tf_listener = tf.TransformListener()

        # Store LiDAR obstacles as dictionary (key: (gx, gy), value: timestamp)
        self.lidar_obstacles = {}

        # Timer for costmap updates
        self.update_timer = rospy.Timer(rospy.Duration(0.5), self.update_and_publish_costmap)

    def map_callback(self, msg):
        rospy.loginfo("Map received.")
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin = (msg.info.origin.position.x, msg.info.origin.position.y)

    def lidar_callback(self, msg):
        ugv_pos = self.get_ugv_position()
        if ugv_pos is None:
            return
        lidar_x, lidar_y = ugv_pos

        angle = msg.angle_min
        current_time = rospy.Time.now().to_sec()
        
        for r in msg.ranges:
            if 0.1 < r < msg.range_max:
                wx = lidar_x + r * np.cos(angle)
                wy = lidar_y + r * np.sin(angle)
                gx, gy = self.world_to_grid(wx, wy)
                if self.is_within_square(gx, gy, ugv_pos):
                    self.lidar_obstacles[(gx, gy)] = current_time
            angle += msg.angle_increment

    def get_ugv_position(self):
        try:
            self.tf_listener.waitForTransform("/map", "/base_link", rospy.Time(0), rospy.Duration(1.0))
            (trans, _) = self.tf_listener.lookupTransform("/map", "/base_link", rospy.Time(0))
            return trans[0], trans[1]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("TF transform failed for UGV position")
            return None

    def world_to_grid(self, wx, wy):
        gx = int((wx - self.map_origin[0]) / self.map_resolution)
        gy = int((wy - self.map_origin[1]) / self.map_resolution)
        return gx, gy

    def is_within_square(self, gx, gy, ugv_pos):
        square_gx, square_gy = self.world_to_grid(*ugv_pos)
        half_size = int(self.square_size / (2 * self.map_resolution))
        return (square_gx - half_size <= gx <= square_gx + half_size and
                square_gy - half_size <= gy <= square_gy + half_size)

    def generate_inflated_costmap(self):
        ugv_pos = self.get_ugv_position()
        if ugv_pos is None:
            return None

        costmap = np.full((self.map_height, self.map_width), -1)
        
        for y in range(self.map_height):
            for x in range(self.map_width):
                if self.is_within_square(x, y, ugv_pos):
                    if self.map_data[y, x] == 100:
                        costmap[y, x] = 100
                    else:
                        costmap[y, x] = 0
        
        # Remove outdated LiDAR obstacles
        current_time = rospy.Time.now().to_sec()
        self.lidar_obstacles = {k: v for k, v in self.lidar_obstacles.items() if current_time - v < self.lidar_decay_time}

        # Add LiDAR obstacles
        for (gx, gy), _ in self.lidar_obstacles.items():
            if self.is_within_square(gx, gy, ugv_pos):
                costmap[gy, gx] = 100
        
        return self.apply_inflation(costmap)

    def apply_inflation(self, costmap):
        binary_map = np.zeros_like(costmap, dtype=np.uint8)
        binary_map[costmap == 100] = 1
        distances = distance_transform_edt(1 - binary_map) * self.map_resolution
        inflated_costmap = np.copy(costmap)
        inflated_costmap[(distances <= self.inflation_radius) & (costmap != -1)] = 100
        return inflated_costmap

    def update_and_publish_costmap(self, event):
        if self.map_data is None:
            return
        costmap = self.generate_inflated_costmap()
        if costmap is not None:
            self.publish_costmap(costmap)

    def publish_costmap(self, costmap):
        costmap_msg = OccupancyGrid()
        costmap_msg.header.stamp = rospy.Time.now()
        costmap_msg.header.frame_id = "map"
        costmap_msg.info.resolution = self.map_resolution
        costmap_msg.info.width = self.map_width
        costmap_msg.info.height = self.map_height
        costmap_msg.info.origin.position.x = self.map_origin[0]
        costmap_msg.info.origin.position.y = self.map_origin[1]
        costmap_msg.info.origin.orientation.w = 1.0
        costmap_msg.data = costmap.flatten().tolist()
        self.costmap_pub.publish(costmap_msg)

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    generator = CostmapGenerator()
    generator.run()
