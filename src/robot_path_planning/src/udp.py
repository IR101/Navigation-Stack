#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import LaserScan
import socket
import struct
import numpy as np

# Laptop IP and UDP port
UDP_IP = "192.168.18.32"  # Change to your laptop IP
UDP_PORT = 5005
MTU = 1400  # Max bytes per UDP packet

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def send_scan(ranges):
    n = len(ranges)
    ranges_bytes = ranges.tostring()  # Python 2.7 compatible
    data = struct.pack("I", n) + ranges_bytes

    # Split into MTU-sized packets
    for i in range(0, len(data), MTU):
        sock.sendto(data[i:i+MTU], (UDP_IP, UDP_PORT))

def scan_callback(msg):
    ranges = np.array(msg.ranges, dtype=np.float32)
    send_scan(ranges)

def main():
    rospy.init_node("udp_lidar_sender")
    rospy.Subscriber("/scan", LaserScan, scan_callback)
    rospy.loginfo("UDP LIDAR sender started, sending to {}:{}".format(UDP_IP, UDP_PORT))
    rospy.spin()

if __name__ == "__main__":
    main()
