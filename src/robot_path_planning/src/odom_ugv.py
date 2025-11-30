#!/usr/bin/env python

"""
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
import math

class OdometryPublisher:
    def __init__(self):
        rospy.init_node("odom_publisher", anonymous=True)

        # Subscribers & Publishers
        rospy.Subscriber("/arduino_ugv_vel", Twist, self.velocity_callback)
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)
        self.odom_broadcaster = tf.TransformBroadcaster()

        # Initial Pose & Time
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = rospy.Time.now()

    def velocity_callback(self, msg):
        # Time calculation
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time

        # Extract linear and angular velocities
        vx = msg.linear.x  # Forward velocity
        vy = msg.linear.y  # Sideways velocity (not used for differential drive)
        vth = msg.angular.z  # Rotational velocity

        # Odometry calculations (assuming differential drive)
        delta_x = (vx * math.cos(self.theta) - vy * math.sin(self.theta)) * dt
        delta_y = (vx * math.sin(self.theta) + vy * math.cos(self.theta)) * dt
        delta_theta = vth * dt

        # Update pose
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Publish Odometry Message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        # Set position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # Set orientation using quaternion
        quat = tf.transformations.quaternion_from_euler(0, 0, self.theta)
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]

        # Set velocity
        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = vy
        odom_msg.twist.twist.angular.z = vth

        # Publish odometry
        self.odom_pub.publish(odom_msg)

        # Publish TF transform (odom base_link)
        self.odom_broadcaster.sendTransform(
            (self.x, self.y, 0),
            quat,
            current_time,
            "base_link",
            "odom"
        )

if __name__ == "__main__":
    try:
        odom_publisher = OdometryPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass



#!/usr/bin/env python

import rospy
import tf
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

class LidarOdom:
    def __init__(self):
        rospy.init_node('lidar_odom', anonymous=True)

        # Subscribing to Lidar scan data
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        # Publishing Odometry data
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)

        # TF Broadcaster for Odom -> Base_link
        self.odom_broadcaster = tf.TransformBroadcaster()

        # Initial pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Previous scan (for simple motion estimation)
        self.prev_ranges = None

        # Time tracking
        self.prev_time = rospy.Time.now()

    def scan_callback(self, scan):
        if self.prev_ranges is None:
            self.prev_ranges = scan.ranges
            return
        
        # Get time difference
        curr_time = rospy.Time.now()
        dt = (curr_time - self.prev_time).to_sec()
        self.prev_time = curr_time

        # Estimate motion (this is a simple approximation)
        delta_x, delta_y, delta_theta = self.estimate_motion(scan.ranges)

        # Update pose
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Publish Odometry message
        self.publish_odom(curr_time)

        # Update previous scan data
        self.prev_ranges = scan.ranges

    def estimate_motion(self, ranges):
        
        #Estimate motion by comparing consecutive LiDAR scans.
        #This is a simple approach and assumes small displacements.
        
        if self.prev_ranges is None:
            return 0.0, 0.0, 0.0

        delta_x, delta_y, delta_theta = 0.0, 0.0, 0.0

        # Compute simple scan matching (basic heuristic)
        for i in range(len(ranges)):
            if ranges[i] < 0.1 or self.prev_ranges[i] < 0.1:  # Ignore invalid readings
                continue

            diff = ranges[i] - self.prev_ranges[i]

            # Heuristic: If all points shift similarly, assume forward motion
            if abs(diff) > 0.05:  # Threshold for detecting motion
                delta_x += diff * 0.05  # Scale movement estimation
                delta_y += diff * 0.02  # Assume slight lateral shift
                delta_theta += diff * 0.01  # Assume slight rotation

        return delta_x, delta_y, delta_theta

    def publish_odom(self, curr_time):
        odom = Odometry()
        odom.header.stamp = curr_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        # Set position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # Convert theta to quaternion
        quat = tf.transformations.quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation = Quaternion(*quat)

        # Publish Transform (TF)
        self.odom_broadcaster.sendTransform(
            (self.x, self.y, 0),
            quat,
            curr_time,
            "base_link",
            "odom"
        )

        # Publish Odometry
        self.odom_pub.publish(odom)

if __name__ == '__main__':
    try:
        LidarOdom()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

"""





import rospy
import tf
import math
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray

class ArduinoOdom:
    def __init__(self):
        rospy.init_node("arduino_odom_publisher")

        # Parameters
        self.wheel_base = 0.406  # Distance between wheels (meters)
        self.x = 0.0  # Robot's x position
        self.y = 0.0  # Robot's y position
        self.theta = 0.0  # Robot's orientation
        self.last_time = rospy.Time.now()

        # Publishers
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)
        self.tf_broadcaster = tf.TransformBroadcaster()

        # Subscriber to Arduino wheel velocity feedback
        rospy.Subscriber("/arduino_ugv_vel", Float32MultiArray, self.arduino_vel_callback)

        rospy.loginfo("Arduino Odometry Node Initialized")

    def arduino_vel_callback(self, msg):
        if len(msg.data) < 2:
            rospy.logwarn("Invalid wheel velocity data received from Arduino")
            return

        # Get current time and compute time difference
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time

        # Extract velocities from message
        v_left = msg.data[0]  # Left wheel velocity (m/s)
        v_right = msg.data[1]  # Right wheel velocity (m/s)

        # Compute robot velocity
        vx = (v_right + v_left) / 2  # Linear velocity (m/s)
        vtheta = (v_right - v_left) / self.wheel_base  # Angular velocity (rad/s)

        # Update pose using velocity integration
        self.x += vx * math.cos(self.theta) * dt
        self.y += vx * math.sin(self.theta) * dt
        self.theta += vtheta * dt

        # Normalize theta to keep it within [-pi, pi]
        self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi

        # Create quaternion from yaw angle
        quaternion = tf.transformations.quaternion_from_euler(0, 0, self.theta)

        # Publish odometry message
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = quaternion[0]
        odom.pose.pose.orientation.y = quaternion[1]
        odom.pose.pose.orientation.z = quaternion[2]
        odom.pose.pose.orientation.w = quaternion[3]
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = vtheta

        self.odom_pub.publish(odom)

        # Publish TF transform (odom -> base_link)
        self.tf_broadcaster.sendTransform(
            (self.x, self.y, 0),
            quaternion,
            current_time,
            "base_link",
            "odom"
        )

if __name__ == "__main__":
    try:
        arduino_odom = ArduinoOdom()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


"""




import rospy
import tf
import math
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray

class ArduinoOdom:
    def __init__(self):
        rospy.init_node("arduino_odom_publisher")

        # Parameters
        self.wheel_base = 0.406  # Distance between wheels (meters)
        self.x = 0.0  # Robot's x position
        self.y = 0.0  # Robot's y position
        self.theta = 0.0  # Robot's orientation
        self.last_time = rospy.Time.now()

        # Publishers
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)
        self.tf_broadcaster = tf.TransformBroadcaster()

        # Subscriber to Arduino wheel velocity feedback
        rospy.Subscriber("/arduino_ugv_vel", Float32MultiArray, self.arduino_vel_callback)

        # Subscriber to IMU data
        rospy.Subscriber("/imu/data_raw", Imu, self.imu_callback)

        rospy.loginfo("Arduino Odometry Node Initialized")

        # IMU Data (linear velocity and angular velocity)
        self.imu_linear_velocity = 0.0
        self.imu_angular_velocity = 0.0

    def imu_callback(self, msg):
        # Extract linear velocity in x and angular velocity in z from IMU
        self.imu_linear_velocity = msg.linear_acceleration.x  # Linear acceleration in x (m/s), can be used for velocity estimation
        self.imu_angular_velocity = msg.angular_velocity.z  # Angular velocity in z (rad/s)

    def arduino_vel_callback(self, msg):
        # Check if the received message contains enough data (at least 2 elements)
        if len(msg.data) < 2:
            rospy.logwarn("Invalid velocity data received from Arduino. Expected at least 2 elements, but got: {}".format(len(msg.data)))
            return  # Return early if data is invalid

        # Extract velocities from message (left and right wheels)
        v_left = msg.data[0]  # Left wheel velocity (m/s)
        v_right = msg.data[1]  # Right wheel velocity (m/s)

        rospy.loginfo("Left wheel velocity: {} m/s, Right wheel velocity: {} m/s".format(v_left, v_right))

        # Get current time and compute time difference
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time

        # Compute robot velocity using wheel encoders
        wheel_vx = (v_right + v_left) / 2  # Linear velocity (m/s)
        wheel_vtheta = (v_right - v_left) / self.wheel_base  # Angular velocity (rad/s)

        # Combine wheel encoder velocity and IMU linear velocity for a more accurate estimate
        vx = 0.7 * wheel_vx + 0.3 * self.imu_linear_velocity  # Weighted average (you can adjust the weights)
        vtheta = wheel_vtheta + self.imu_angular_velocity  # Combine angular velocities

        # Update pose using velocity integration
        self.x += vx * math.cos(self.theta) * dt
        self.y += vx * math.sin(self.theta) * dt
        self.theta += vtheta * dt

        # Normalize theta to keep it within [-pi, pi]
        self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi

        # Create quaternion from yaw angle
        quaternion = tf.transformations.quaternion_from_euler(0, 0, self.theta)

        # Publish odometry message
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = quaternion[0]
        odom.pose.pose.orientation.y = quaternion[1]
        odom.pose.pose.orientation.z = quaternion[2]
        odom.pose.pose.orientation.w = quaternion[3]
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = vtheta

        self.odom_pub.publish(odom)

        # Publish TF transform (odom -> base_link)
        self.tf_broadcaster.sendTransform(
            (self.x, self.y, 0),
            quaternion,
            current_time,
            "base_link",
            "odom"
        )

if __name__ == "__main__":
    try:
        arduino_odom = ArduinoOdom()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

"""