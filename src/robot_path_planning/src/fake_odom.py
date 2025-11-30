#!/usr/bin/env python

import rospy
import tf
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class CmdVelOdom:
    def __init__(self):
        rospy.init_node("cmd_vel_odom_publisher")

        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)
        self.tf_broadcaster = tf.TransformBroadcaster()
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_callback)

        self.last_time = rospy.Time.now()

        # Initialize pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Velocity input
        self.vx = 0.0
        self.vz = 0.0

        self.timer = rospy.Timer(rospy.Duration(0.05), self.update_odom)  # 20 Hz

    def cmd_callback(self, msg):
        self.vx = msg.linear.x
        self.vz = msg.angular.z

    def update_odom(self, event):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time

        # Integrate position
        delta_x = self.vx * math.cos(self.theta) * dt
        delta_y = self.vx * math.sin(self.theta) * dt
        delta_theta = self.vz * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Quaternion for yaw rotation
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.theta)

        # Create Odometry message
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = odom_quat[0]
        odom.pose.pose.orientation.y = odom_quat[1]
        odom.pose.pose.orientation.z = odom_quat[2]
        odom.pose.pose.orientation.w = odom_quat[3]
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = self.vz

        self.odom_pub.publish(odom)

        # Broadcast transform
        self.tf_broadcaster.sendTransform(
            (self.x, self.y, 0),
            odom_quat,
            current_time,
            "base_link",
            "odom"
        )

if __name__ == "__main__":
    CmdVelOdom()
    rospy.spin()
