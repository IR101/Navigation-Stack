#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Path

class TEBFollower:
    def __init__(self):
        rospy.init_node('teb_follower', anonymous=True)
        
        # Subscribe to robot pose from AMCL
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_callback)
        # Subscribe to the local plan published by TEB
        rospy.Subscriber('/move_base/TebLocalPlannerROS/local_plan', Path, self.local_plan_callback)
        # Subscribe to velocity commands from TEB planner
        rospy.Subscriber('/teb/cmd_vel', Twist, self.cmd_vel_callback)
        
        # Publisher for velocity commands
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=3)
        # Publisher for visualizing the local plan in RViz
        self.local_plan_pub = rospy.Publisher('/local_path', Path, queue_size=3)
        
        self.robot_pose = None
        rospy.loginfo("TEB Follower node initialized.")

    def pose_callback(self, msg):
        self.robot_pose = msg.pose.pose

    def local_plan_callback(self, msg):
        rospy.loginfo("Received local plan with {} waypoints.".format(len(msg.poses)))
        msg.header.frame_id = "map"  # Ensure frame consistency
        msg.header.stamp = rospy.Time.now()
        self.local_plan_pub.publish(msg)

    def cmd_vel_callback(self, msg):
        rospy.loginfo("Received velocity command from TEB.")
        self.cmd_vel_pub.publish(msg)
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        follower = TEBFollower()
        follower.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("TEB Follower node terminated.")

