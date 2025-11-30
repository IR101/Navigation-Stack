#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult
from geometry_msgs.msg import PoseStamped

class MoveBaseReplanner:
    def __init__(self):
        rospy.init_node('move_base_replanner', anonymous=True)
        
        # Move Base action client
        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base action server.")
        
        # Subscribe to move_base result
        rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.replan_callback)
        
        # Publisher for new goal
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        
        # Store last goal
        self.last_goal = None
        
        rospy.loginfo("MoveBase Replanner Node Started")
    
    def send_goal(self, goal):
        """ Sends a goal to move_base and stores it """
        self.last_goal = goal
        
        move_base_goal = MoveBaseGoal()
        move_base_goal.target_pose = goal
        
        rospy.loginfo("Sending new goal to move_base")
        self.client.send_goal(move_base_goal)
    
    def replan_callback(self, msg):
        """ Replans if the move_base goal fails """
        if msg.status.status in [4, 5]:  # Status 4: ABORTED, Status 5: REJECTED
            rospy.logwarn("Trajectory infeasible! Replanning...")
            if self.last_goal:
                rospy.sleep(1.0)  # Small delay before replanning
                self.goal_pub.publish(self.last_goal)
                rospy.loginfo("New goal published to move_base_simple/goal")
            else:
                rospy.logwarn("No last goal stored. Cannot replan!")
    
if __name__ == '__main__':
    try:
        replanner = MoveBaseReplanner()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("MoveBase Replanner Node Shutting Down")
