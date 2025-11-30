#!/usr/bin/env python

import rospy
import numpy as np
import tf
import pcl
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from sklearn.neighbors import NearestNeighbors

class LiDAROdometry:
    def __init__(self):
        rospy.init_node("lidar_odometry", anonymous=True)

        # Subscribers & Publishers
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.lidar_callback, queue_size=1)
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=1)

        # TF Broadcaster
        self.odom_broadcaster = tf.TransformBroadcaster()

        # Store previous scan
        self.prev_scan = None

        # Initialize pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # Yaw angle

        rospy.loginfo("LiDAR Odometry Node Started")

    def lidar_callback(self, scan):
        if self.prev_scan is None:
            self.prev_scan = self.scan_to_pointcloud(scan)
            return

        # Convert scan to point cloud
        curr_scan = self.scan_to_pointcloud(scan)

        # Compute transformation using basic ICP
        transformation = self.icp_match(self.prev_scan, curr_scan)
        if transformation is not None:
            dx, dy, dtheta = transformation
            self.update_pose(dx, dy, dtheta)

        # Publish odometry message
        self.publish_odometry(scan.header.stamp)

        # Store current scan as previous scan for next iteration
        self.prev_scan = curr_scan

    def scan_to_pointcloud(self, scan):
        """ Convert LaserScan to 2D Point Cloud """
        angle = scan.angle_min
        points = []
        for r in scan.ranges:
            if scan.range_min < r < scan.range_max:  # Ignore invalid ranges
                x = r * np.cos(angle)
                y = r * np.sin(angle)
                points.append([x, y])
            angle += scan.angle_increment
        return points  # Keep as a Python list for compatibility

    def icp_match(self, prev_points, curr_points):
        """ Apply a basic ICP algorithm using Nearest Neighbors """
        if len(prev_points) < 10 or len(curr_points) < 10:
            rospy.logwarn("Insufficient points for ICP")
            return None

        prev_points = np.array(prev_points)
        curr_points = np.array(curr_points)

        # Nearest neighbor search
        nn = NearestNeighbors(n_neighbors=1, algorithm='kd_tree').fit(prev_points)
        distances, indices = nn.kneighbors(curr_points)

        # Compute transformation using Least Squares
        dx = np.mean(prev_points[indices[:, 0], 0] - curr_points[:, 0])
        dy = np.mean(prev_points[indices[:, 0], 1] - curr_points[:, 1])
        dtheta = 0.0  # Simplified ICP (only translation, no rotation)

        return dx, dy, dtheta

    def update_pose(self, dx, dy, dtheta):
        """ Update robot pose based on estimated transformation """
        self.x += dx * np.cos(self.theta) - dy * np.sin(self.theta)
        self.y += dx * np.sin(self.theta) + dy * np.cos(self.theta)
        self.theta += dtheta

    def publish_odometry(self, timestamp):
        """ Publish Odometry message """
        odom = Odometry()
        odom.header.stamp = timestamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        # Set position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # Set orientation
        quaternion = tf.transformations.quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation = Quaternion(*quaternion)

        # Publish odometry
        self.odom_pub.publish(odom)

        # Publish TF transform
        self.odom_broadcaster.sendTransform(
            (self.x, self.y, 0),
            quaternion,
            timestamp,
            "base_link",
            "odom"
        )

if __name__ == "__main__":
    try:
        LiDAROdometry()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
