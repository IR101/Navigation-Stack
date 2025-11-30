#!/usr/bin/env python
import rospy
import numpy as np
import tf
import cv2
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class ScanMatcher:
    def __init__(self):
        rospy.init_node("scan_matcher", anonymous=True)

        # Subscribers
        rospy.Subscriber("/scan", LaserScan, self.lidar_callback)
        rospy.Subscriber("/map", OccupancyGrid, self.map_callback)

        # Publisher to send initial pose to AMCL
        self.initial_pose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)

        # Transform listener (for coordinate transformations)
        self.tf_listener = tf.TransformListener()

        # Storage variables
        self.map_data = None
        self.map_resolution = None
        self.map_origin = None
        self.lidar_data = None

        # Only process once
        self.initial_pose_sent = False

    def map_callback(self, msg):
        """Loads the occupancy grid map for scan matching"""
        rospy.loginfo("Received prebuilt map.")
        self.map_resolution = msg.info.resolution
        self.map_origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        width, height = msg.info.width, msg.info.height

        # Convert occupancy grid to binary image (255 = occupied, 0 = free)
        self.map_data = np.array(msg.data, dtype=np.int8).reshape((height, width))
        self.map_data = np.where(self.map_data > 50, 255, 0).astype(np.uint8)  # Thresholding
        self.map_data = cv2.flip(self.map_data, 0)  # Flip vertically

    def lidar_callback(self, scan_msg):
        """Processes LiDAR scans, matches with the map, and finds the best pose"""
        if self.map_data is None or self.initial_pose_sent:
            return

        rospy.loginfo("Processing LiDAR scan for pose estimation...")

        # Convert LiDAR scan to a point cloud
        lidar_points = self.convert_scan_to_points(scan_msg)

        if lidar_points is None or len(lidar_points) == 0:
            rospy.logwarn("No valid LiDAR points detected!")
            return

        # Perform scan matching to find the best pose
        best_pose = self.match_scan_to_map(lidar_points)

        if best_pose:
            rospy.loginfo(f"Best pose found: {best_pose}")
            self.publish_initial_pose(best_pose)
            self.initial_pose_sent = True
        else:
            rospy.logwarn("Failed to determine initial pose.")

    def convert_scan_to_points(self, scan_msg):
        """Converts a LiDAR scan into Cartesian coordinates in the base_laser frame"""
        angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(scan_msg.ranges))
        ranges = np.array(scan_msg.ranges)

        # Remove infinite and out-of-range values
        valid = (ranges > scan_msg.range_min) & (ranges < scan_msg.range_max)
        x = ranges[valid] * np.cos(angles[valid])
        y = ranges[valid] * np.sin(angles[valid])

        if x.size == 0:
            return None

        lidar_points = np.vstack((x, y)).T

        # Transform LiDAR points from "base_laser" frame to "map" frame
        try:
            (trans, rot) = self.tf_listener.lookupTransform("map", scan_msg.header.frame_id, rospy.Time(0))
            yaw = euler_from_quaternion(rot)[2]
            cos_yaw, sin_yaw = np.cos(yaw), np.sin(yaw)

            transformed_points = np.array([
                [cos_yaw * p[0] - sin_yaw * p[1] + trans[0],
                 sin_yaw * p[0] + cos_yaw * p[1] + trans[1]]
                for p in lidar_points
            ])
            return transformed_points
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("TF Transform failed. Using raw LiDAR points.")
            return lidar_points

    def match_scan_to_map(self, lidar_points):
        """Aligns the LiDAR scan with the prebuilt map using correlation-based matching"""
        max_correlation = -float("inf")
        best_pose = None

        # Define search range
        search_x = np.linspace(self.map_origin[0], self.map_origin[0] + self.map_data.shape[1] * self.map_resolution, 20)
        search_y = np.linspace(self.map_origin[1], self.map_origin[1] + self.map_data.shape[0] * self.map_resolution, 20)
        search_theta = np.linspace(-np.pi, np.pi, 10)

        for x in search_x:
            for y in search_y:
                for theta in search_theta:
                    score = self.compute_correlation(x, y, theta, lidar_points)
                    if score > max_correlation:
                        max_correlation = score
                        best_pose = (x, y, theta)

        return best_pose

    def compute_correlation(self, x, y, theta, lidar_points):
        """Computes how well the LiDAR scan aligns with the map at a given pose"""
        cos_theta, sin_theta = np.cos(theta), np.sin(theta)
        transformed_points = np.array([
            [cos_theta * p[0] - sin_theta * p[1] + x,
             sin_theta * p[0] + cos_theta * p[1] + y]
            for p in lidar_points
        ])

        # Convert transformed points to map indices
        map_x = ((transformed_points[:, 0] - self.map_origin[0]) / self.map_resolution).astype(int)
        map_y = ((transformed_points[:, 1] - self.map_origin[1]) / self.map_resolution).astype(int)

        valid = (map_x >= 0) & (map_x < self.map_data.shape[1]) & (map_y >= 0) & (map_y < self.map_data.shape[0])
        return np.sum(self.map_data[map_y[valid], map_x[valid]] > 100)

    def publish_initial_pose(self, pose):
        """Publishes the found initial pose to AMCL"""
        x, y, theta = pose
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.stamp = rospy.Time.now()
        initial_pose.header.frame_id = "map"
        initial_pose.pose.pose.position.x = x
        initial_pose.pose.pose.position.y = y

        quat = quaternion_from_euler(0, 0, theta)
        initial_pose.pose.pose.orientation.z = quat[2]
        initial_pose.pose.pose.orientation.w = quat[3]

        self.initial_pose_pub.publish(initial_pose)
        rospy.loginfo("Initial pose sent to AMCL!")

if __name__ == "__main__":
    try:
        ScanMatcher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
