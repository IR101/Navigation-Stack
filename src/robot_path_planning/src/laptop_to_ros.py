#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Python 2.7

import rospy
import struct
import base64
import websocket
from sensor_msgs.msg import LaserScan
import time

JETSON_IP = "192.168.18.110"
WS_PORT = 8765
TOPIC_NAME = "/scan"

def on_message(ws, message):
    try:
        data = base64.b64decode(message)
        n = struct.unpack("I", data[:4])[0]
        ranges = struct.unpack("%sf" % n, data[4:4 + 4 * n])

        scan_msg = LaserScan()
        scan_msg.header.stamp = rospy.Time.now()
        scan_msg.header.frame_id = "laser"
        scan_msg.angle_min = 0.0
        scan_msg.angle_max = 2 * 3.1415926
        scan_msg.angle_increment = 2 * 3.1415926 / n
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 0.1
        scan_msg.range_min = 0.0
        scan_msg.range_max = 10.0
        scan_msg.ranges = ranges

        pub.publish(scan_msg)
    except Exception as e:
        rospy.logerr("Error parsing LIDAR data: %s" % str(e))

def on_error(ws, error):
    rospy.logerr("WebSocket error: %s" % str(error))

def on_close(ws, close_status_code, close_msg):
    rospy.logwarn("WebSocket connection closed")

def on_open(ws):
    rospy.loginfo("Connected to Jetson LIDAR WebSocket server")

if __name__ == "__main__":
    rospy.init_node("lidar_ws_receiver", anonymous=True)
    pub = rospy.Publisher(TOPIC_NAME, LaserScan, queue_size=1)

    ws_url = "ws://{}:{}".format(JETSON_IP, WS_PORT)

    while not rospy.is_shutdown():
        ws = websocket.WebSocketApp(
            ws_url,
            on_message=on_message,
            on_error=on_error,
            on_close=on_close
        )
        ws.on_open = on_open

        try:
            ws.run_forever(ping_interval=10, ping_timeout=5)
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down LIDAR WebSocket receiver...")
            ws.close()
            break
        except Exception as e:
            rospy.logerr("Unexpected error: %s" % str(e))

        if not rospy.is_shutdown():
            rospy.logwarn("Reconnecting in 1s...")
            time.sleep(1)
