#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

def pose_callback(msg):
    odom_msg = Odometry()
    odom_msg.header = msg.header
    odom_msg.pose = msg.pose
    odom_pub.publish(odom_msg)

rospy.init_node('pose_to_odom_converter')
odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
rospy.Subscriber('/poseupdate', PoseWithCovarianceStamped, pose_callback)
rospy.spin()
