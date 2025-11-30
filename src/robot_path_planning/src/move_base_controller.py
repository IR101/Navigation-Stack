#!/usr/bin/env python

import rospy
import serial
from geometry_msgs.msg import PoseStamped

# Define goal locations
goal_locations = {
    "Room1": [2.243, -3.726, -0.720, 0.694],  
    "Room2": [-4.746, 0.274, 0.977, 0.214],  
    "DLD_lab": [1.522, 4.483, -0.087, 0.996]  
}

# Initialize serial connection
SERIAL_PORT = "/dev/ttyACM0"  # Change this based on your system
BAUD_RATE = 57600
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

# ROS node setup
rospy.init_node("serial_goal_publisher", anonymous=True)
pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
rospy.sleep(1)  # Ensure ROS is fully initialized

def send_goal(goal_name):
    if goal_name in goal_locations:
        goal_data = goal_locations[goal_name]
        goal_msg = PoseStamped()
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position.x = goal_data[0]
        goal_msg.pose.position.y = goal_data[1]
        goal_msg.pose.position.z = 0
        goal_msg.pose.orientation.z = goal_data[2]
        goal_msg.pose.orientation.w = goal_data[3]

        rospy.loginfo("Publishing goal: {} -> {}".format(goal_name, goal_data))
        pub.publish(goal_msg)
    else:
        rospy.logwarn("Received unknown goal: {}".format(goal_name))

while not rospy.is_shutdown():
    try:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if line:
            rospy.loginfo("Received: " + line)
            if "goal_pose" in line:
                message_line = ser.readline().decode('utf-8', errors='ignore').strip()
                if "Message:" in message_line:
                    message = message_line.split("Message:")[1].strip()
                    rospy.loginfo("Parsed goal: " + message)
                    send_goal(message)

    except serial.SerialException as e:
        rospy.logerr("Serial error: {}".format(e))
    except Exception as e:
        rospy.logerr("Error: {}".format(e))
