#!/usr/bin/env python3
import asyncio
import websockets
import struct
import rospy
from sensor_msgs.msg import LaserScan

# WebSocket server on laptop
WS_URL = "ws://192.168.18.32:8765"  

async def send_laser():
    async with websockets.connect(WS_URL) as ws:
        rospy.init_node("ws_lidar_sender")
        
        def callback(scan_msg):
            # Pack LaserScan data: number of points + float32 ranges
            n = len(scan_msg.ranges)
            packed = struct.pack("I", n) + struct.pack("%sf" % n, *scan_msg.ranges)
            # Send over WebSocket
            asyncio.create_task(ws.send(packed))
        
        rospy.Subscriber("/scan", LaserScan, callback)
        rospy.spin()

if __name__ == "__main__":
    asyncio.get_event_loop().run_until_complete(send_laser())
