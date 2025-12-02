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














# #!/usr/bin/env python
# # -*- coding: utf-8 -*-

# import rospy
# import struct
# import base64
# import websocket
# import time

# from sensor_msgs.msg import LaserScan
# from geometry_msgs.msg import Twist

# JETSON_IP = "192.168.18.110"
# WS_PORT = 8765

# SCAN_TOPIC = "/scan"
# CMD_TOPIC = "/cmd_vel"

# ws_global = None   # global WS handle


# # ------------------ WebSocket Receive ------------------

# def on_message(ws, message):
#     try:
#         data = base64.b64decode(message)

#         msg_type = data[0]   # first byte

#         # ---------- LIDAR ----------
#         if msg_type == b'L':
#             n = struct.unpack("I", data[1:5])[0]
#             ranges = struct.unpack("%sf" % n, data[5:5 + 4 * n])

#             scan = LaserScan()
#             scan.header.stamp = rospy.Time.now()
#             scan.header.frame_id = "laser"
#             scan.angle_min = 0.0
#             scan.angle_max = 2 * 3.1415926
#             scan.angle_increment = (2 * 3.1415926) / n
#             scan.time_increment = 0.0
#             scan.scan_time = 0.1
#             scan.range_min = 0.05
#             scan.range_max = 10.0
#             scan.ranges = ranges

#             scan_pub.publish(scan)

#         else:
#             rospy.logwarn("Unknown message type received")

#     except Exception as e:
#         rospy.logerr("WS parse error: %s", str(e))


# def on_open(ws):
#     rospy.loginfo("WebSocket connected to Jetson")


# def on_close(ws, *args):
#     rospy.logwarn("WebSocket closed")


# def on_error(ws, error):
#     rospy.logerr("WebSocket error: %s", str(error))


# # ------------------ CMD_VEL Sender ------------------

# def cmd_vel_callback(msg):
#     global ws_global

#     if ws_global is None:
#         return

#     try:
#         payload = struct.pack(
#             "cff",
#             b'C',                 # message tag
#             msg.linear.x,
#             msg.angular.z
#         )

#         encoded = base64.b64encode(payload)
#         ws_global.send(encoded)

#     except Exception as e:
#         rospy.logerr("Failed to send cmd_vel: %s", str(e))


# # ------------------ Main ------------------

# if __name__ == "__main__":

#     rospy.init_node("lidar_ws_receiver", anonymous=True)

#     scan_pub = rospy.Publisher(SCAN_TOPIC, LaserScan, queue_size=1)
#     rospy.Subscriber(CMD_TOPIC, Twist, cmd_vel_callback)

#     ws_url = "ws://{}:{}".format(JETSON_IP, WS_PORT)

#     while not rospy.is_shutdown():
#         ws = websocket.WebSocketApp(
#             ws_url,
#             on_message=on_message,
#             on_open=on_open,
#             on_close=on_close,
#             on_error=on_error
#         )

#         ws_global = ws

#         try:
#             ws.run_forever(ping_interval=10, ping_timeout=5)
#         except Exception as e:
#             rospy.logerr("WS exception: %s", str(e))

#         ws_global = None

#         if not rospy.is_shutdown():
#             rospy.logwarn("Reconnecting in 1 second...")
#             time.sleep(1)
