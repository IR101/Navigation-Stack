#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Python 2.7

import rospy
import math
import base64
import struct
import threading
from sensor_msgs.msg import LaserScan
from websocket_server import WebsocketServer

clients_lock = threading.Lock()
clients = []

def new_client(client, server):
    rospy.loginfo("Client connected: %d" % client['id'])
    with clients_lock:
        clients.append(client)

def client_left(client, server):
    rospy.loginfo("Client disconnected: %d" % client['id'])
    with clients_lock:
        if client in clients:
            clients.remove(client)

def clean_ranges(ranges, max_range):
    """Replace NaN or Infinity with max_range"""
    cleaned = []
    for r in ranges:
        if r is None or math.isnan(r) or math.isinf(r):
            cleaned.append(max_range)
        else:
            cleaned.append(r)
    return cleaned


def scan_callback(msg):
    with clients_lock:
        if not clients:
            return

        # Clean the ranges
        ranges_clean = clean_ranges(msg.ranges, msg.range_max)
        n = len(ranges_clean)

        # Pack data: number of points (uint32) + ranges (float32)
        packet = struct.pack("I", n) + struct.pack("%sf" % n, *ranges_clean)

        # Encode the binary packet as Base64 string
        packet_b64 = base64.b64encode(packet).decode('ascii')

        # Send Base64 string to all clients
        for c in clients[:]:  # copy to avoid modification during iteration
            try:
                server.send_message(c, packet_b64)
            except Exception as e:
                rospy.logerr("WebSocket send error to client %d: %s" % (c['id'], str(e)))
                clients.remove(c)

def main():
    rospy.init_node("scan_websocket_sender", anonymous=False)

    global server
    server = WebsocketServer(host='0.0.0.0', port=8765)  # Removed loglevel
    server.set_fn_new_client(new_client)
    server.set_fn_client_left(client_left)

    # Run WebSocket server in a separate thread
    server_thread = threading.Thread(target=server.run_forever)
    server_thread.daemon = True
    server_thread.start()

    rospy.Subscriber("/scan", LaserScan, scan_callback, queue_size=1)

    rospy.loginfo("WebSocket Server started at ws://<Jetson_IP>:8765")
    rospy.spin()  # Keep ROS running

if __name__ == "__main__":
    main()
