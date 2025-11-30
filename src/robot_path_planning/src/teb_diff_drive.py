#!/usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Twist

class TEBToDiffDrive:
    def __init__(self):
        rospy.init_node('teb_to_diff_drive_converter', anonymous=True)
        
        # Subscribe to TEB trajectory
        self.teb_sub = rospy.Subscriber('/move_base/TebLocalPlannerROS/teb_poses', Path, self.teb_callback)
        
        # Publish modified trajectory
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        self.prev_time = None
        self.prev_pose = None
        
        # Gain factors
        self.linear_gain = rospy.get_param('~linear_gain', 1.2)
        self.angular_gain = rospy.get_param('~angular_gain', 1.5)
        
    def teb_callback(self, path_msg):
        if len(path_msg.poses) < 2:
            return  # Not enough points for a trajectory

        # Take the first two poses to calculate velocity
        pose1 = path_msg.poses[0].pose.position
        pose2 = path_msg.poses[1].pose.position
        
        # Get time difference
        curr_time = path_msg.header.stamp.to_sec()
        if self.prev_time is None:
            self.prev_time = curr_time
            self.prev_pose = pose1
            return
        
        dt = curr_time - self.prev_time
        if dt <= 0:
            return

        # Calculate linear velocity
        dx = pose2.x - self.prev_pose.x
        dy = pose2.y - self.prev_pose.y
        linear_velocity = (np.sqrt(dx**2 + dy**2) / dt) * self.linear_gain
        
        # Calculate angular velocity
        yaw1 = np.arctan2(dy, dx)
        yaw2 = np.arctan2(pose2.y - pose1.y, pose2.x - pose1.x)
        angular_velocity = ((yaw2 - yaw1) / dt) * self.angular_gain
        
        # Publish cmd_vel
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = linear_velocity
        cmd_vel_msg.angular.z = angular_velocity
        self.cmd_vel_pub.publish(cmd_vel_msg)
        
        # Update previous values
        self.prev_time = curr_time
        self.prev_pose = pose2

if __name__ == '__main__':
    try:
        teb_to_diff_drive = TEBToDiffDrive()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
