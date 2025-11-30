#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from move_base_msgs.msg import MoveBaseActionResult
from collections import defaultdict

# Store the latest joint values
joint_cache = defaultdict(lambda: None)
required_joints = {'lifter_joint1', 'revolute_joint', 'slider_joint'}

# Timeout settings for missing joints
MISSING_JOINT_TIMEOUT = 2.0  # Seconds
last_update_time = {}

def goal_reached_callback(msg):
    if msg.status.status == 3:  # Goal reached
        rospy.loginfo("Goal reached. Enter joint values:")
        lifter_joint1 = float(input("Enter value for lifter_joint1: "))
        revolute_joint = float(input("Enter value for revolute_joint: "))
        slider_joint = float(input("Enter value for slider_joint: "))
        publish_joint_states(lifter_joint1, revolute_joint, slider_joint)

def publish_joint_states(lifter_joint1, revolute_joint, slider_joint):
    joint_state_msg = JointState()
    joint_state_msg.header.stamp = rospy.Time.now()
    joint_state_msg.name = ['lifter_joint1', 'revolute_joint', 'slider_joint']
    joint_state_msg.position = [lifter_joint1, revolute_joint, slider_joint]
    joint_state_pub.publish(joint_state_msg)
    rospy.loginfo("Published joint states to RViz.")

def joint_state_callback(msg):
    global last_update_time

    received_joints = set(msg.name)
    rospy.loginfo("Received joint names: {}".format(msg.name))

    # Update the joint cache with the latest values
    for i, joint in enumerate(msg.name):
        joint_cache[joint] = msg.position[i]
        last_update_time[joint] = rospy.get_time()

    # Check for missing joints
    missing_joints = required_joints - received_joints
    stale_joints = {joint for joint in required_joints if joint in last_update_time and rospy.get_time() - last_update_time[joint] > MISSING_JOINT_TIMEOUT}

    if missing_joints:
        rospy.logwarn("Temporarily missing joints: {}".format(list(missing_joints)))
    
    if stale_joints:
        rospy.logerr("Joints missing for too long: {}".format(list(stale_joints)))

    # Debugging: Print the joint cache state
    rospy.loginfo("Joint cache: {}".format(dict(joint_cache)))

if __name__ == '__main__':
    rospy.init_node('joint_state_controller', anonymous=True)
    joint_state_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
    rospy.Subscriber('/move_base/result', MoveBaseActionResult, goal_reached_callback)
    rospy.Subscriber('/joint_states', JointState, joint_state_callback)
    rospy.spin()
