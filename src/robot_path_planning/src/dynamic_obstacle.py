#!/usr/bin/env python
"""
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import numpy as np

class DynamicObstacleAvoidance:
    def __init__(self):
        rospy.init_node('dynamic_obstacle_avoidance')

        # Subscribers
        rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        rospy.Subscriber('/replan_done', Bool, self.replan_done_callback)  # Global planner confirmation
        rospy.Subscriber('/local_planner_cmd', Twist, self.local_planner_callback)  # Local planner's velocity command

        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.replan_pub = rospy.Publisher('/replan_path', Bool, queue_size=10)

        # Parameters
        self.obstacle_threshold = 0.8  # meters
        self.safe_threshold = 1.2      # Safe distance to resume normal operation
        self.avoidance_active = False
        self.replan_requested = False
        self.replan_done = False
        self.lidar_ranges = []
        self.latest_cmd_vel = Twist()  # Stores latest local planner command

        rospy.loginfo("Dynamic Obstacle Avoidance Node Initialized.")

    def lidar_callback(self, msg):
        self.lidar_ranges = np.array(msg.ranges)
        self.lidar_ranges[self.lidar_ranges == 0] = np.inf  # Ignore invalid readings
        
        front = np.min(self.lidar_ranges[:30].tolist() + self.lidar_ranges[-30:].tolist())
        left = np.min(self.lidar_ranges[60:120])
        right = np.min(self.lidar_ranges[-120:-60])
        back = np.min(self.lidar_ranges[150:210])
        
        # Check for unmapped obstacles (dynamic obstacles)
        if self.detect_unmapped_obstacle(front, left, right, back):
            rospy.logwarn("Dynamic obstacle detected! Stopping and avoiding.")
            self.avoidance_active = True
            self.stop_ugv()
            self.avoid_obstacle(front, left, right, back)
        elif self.avoidance_active and front > self.safe_threshold and not self.replan_requested:
            rospy.loginfo("Path is clear. Requesting replan.")
            self.trigger_replan()
        elif self.replan_done:
            rospy.loginfo("Replanning confirmed. Resuming normal operation.")
            self.replan_done = False
            self.avoidance_active = False
            self.replan_requested = False

    def detect_unmapped_obstacle(self, front, left, right, back):
        #Checks if an unmapped obstacle is detected. 
        return front < self.obstacle_threshold or left < self.obstacle_threshold or right < self.obstacle_threshold

    def local_planner_callback(self, msg):
        # Receives velocity commands from the local planner 
        self.latest_cmd_vel = msg  # Store the latest command
        
        # If no obstacle, forward the local planner's velocity command
        if not self.avoidance_active:
            self.cmd_vel_pub.publish(msg)

    def replan_done_callback(self, msg):
        # Callback to confirm that replanning has been completed. 
        if msg.data:
            rospy.loginfo("Received replanning confirmation.")
            self.replan_done = True

    def stop_ugv(self):
        #Stops the UGV immediately when an obstacle is detected. 
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        rospy.sleep(0.5)

    def avoid_obstacle(self, front, left, right, back):
        # Moves to the area with the most free space before stopping. 
        twist = Twist()
        
        # Determine best direction to move
        free_spaces = {'left': left, 'right': right, 'back': back}
        best_direction = max(free_spaces, key=free_spaces.get)
        
        if free_spaces[best_direction] > self.obstacle_threshold:
            if best_direction == 'left':
                twist.linear.x = 0.0
                twist.angular.z = 0.5
            elif best_direction == 'right':
                twist.linear.x = 0.0
                twist.angular.z = -0.5
            elif best_direction == 'back':
                twist.linear.x = -0.2
                twist.angular.z = 0.0
                
            self.cmd_vel_pub.publish(twist)
            rospy.sleep(1.5)  # Move for a short duration
            rospy.loginfo("Moved towards {}, stopping UGV.".format(best_direction))
            self.stop_ugv()
        else:
            rospy.logwarn("No clear space available! Staying stopped.")

    def trigger_replan(self):
        # Stops the UGV and requests a new path from the global planner. 
        self.stop_ugv()
        if not self.replan_requested:
            rospy.loginfo("Requesting global planner to replan.")
            self.replan_pub.publish(True)
            self.replan_requested = True

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        avoidance = DynamicObstacleAvoidance()
        avoidance.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Dynamic obstacle avoidance node terminated.")



import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class LidarNavigator:
    def __init__(self):
        rospy.init_node("lidar_obstacle_avoider")

        # Subscriber for LIDAR
        rospy.Subscriber("/scan", LaserScan, self.lidar_callback)

        # Subscriber for local planner cmd
        rospy.Subscriber('/local_cmd_vel', Twist, self.local_planner_callback)

        # Publisher to robot's velocity command
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Parameters
        self.safe_distance = 0.8  # meters
        self.angular_speed = 0.8  # rad/s

        # Store the latest local planner command
        self.latest_local_cmd = Twist()

        rospy.loginfo("Lidar Navigator with local planner integration initialized.")

    def local_planner_callback(self, msg):
        #Stores latest velocity command from local planner.
        self.latest_local_cmd = msg

    def lidar_callback(self, scan):
        twist = Twist()

        # Divide scan into sectors
        ranges = scan.ranges
        angle_min = scan.angle_min
        angle_increment = scan.angle_increment
        num_points = len(ranges)

        front_angles = []
        left_angles = []
        right_angles = []

        for i in range(num_points):
            angle = angle_min + i * angle_increment
            dist = ranges[i]

            if dist == float('inf') or math.isnan(dist):
                continue

            # Forward sector
            if -math.radians(30) <= angle <= math.radians(30):
                front_angles.append(dist)
            # Left sector
            elif math.radians(30) < angle <= math.radians(90):
                left_angles.append(dist)
            # Right sector
            elif -math.radians(90) <= angle < -math.radians(30):
                right_angles.append(dist)

        min_front = min(front_angles) if front_angles else float('inf')
        min_left = min(left_angles) if left_angles else float('inf')
        min_right = min(right_angles) if right_angles else float('inf')

        # Decision logic
        if min_front < self.safe_distance:
            rospy.loginfo_throttle(1, "Obstacle detected! Overriding local planner.")
            # Obstacle avoidance behavior
            if min_left > min_right:
                twist.linear.x = -0.2
                twist.angular.z = self.angular_speed
            else:
                twist.linear.x = -0.2
                twist.angular.z = -self.angular_speed
        else:
            # No obstacle: forward local planner command
            twist = self.latest_local_cmd

        self.cmd_pub.publish(twist)

if __name__ == "__main__":
    try:
        LidarNavigator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass






import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
from std_msgs.msg import Bool


class SimpleAvoider:
    def __init__(self):
        rospy.init_node('simple_dynamic_avoider')

        self.obstacle_threshold = 0.7  # meters
        self.safe_distance = 0.7

        self.latest_cmd = Twist()
        self.avoid_cmd = Twist()

        self.obstacle_detected = False

        rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        rospy.Subscriber('/local_cmd_vel', Twist, self.cmd_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.replan_pub = rospy.Publisher('/replan_path', Bool, queue_size=5)

        rospy.loginfo("Simple dynamic avoider node started.")

    def cmd_callback(self, msg):
        self.latest_cmd = msg

    def lidar_callback(self, scan):
        ranges = np.array(scan.ranges)
        ranges = np.nan_to_num(ranges)
        ranges[ranges == float('inf')] = 10.0
        ranges = np.clip(ranges, 0.05, 10.0)

        # Define sectors
        num_ranges = len(ranges)
        left = ranges[num_ranges // 2 + 30:num_ranges // 2 + 90]   # ~left 30-90
        right = ranges[num_ranges // 2 - 90:num_ranges // 2 - 30]  # ~right 30-90
        front = ranges[num_ranges // 2 - 35:num_ranges // 2 + 35]  # ~front 60

        # Obstacle in front?
        if np.any(front < self.obstacle_threshold):
            self.obstacle_detected = True
            self.avoid_cmd = self.generate_avoidance(left, right)
        else:
            self.obstacle_detected = False

    def generate_avoidance(self, left, right):
        avg_left = np.mean(left)
        avg_right = np.mean(right)
        twist = Twist()
        twist.linear.x = -0.4  # Slow forward motion
        rospy.sleep(0.5)  # Move for a short duration
        return twist

    def run(self):
        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            if self.obstacle_detected:
                self.cmd_vel_pub.publish(self.avoid_cmd)
                self.replan_pub.publish(True)
            else:
                self.cmd_vel_pub.publish(self.latest_cmd)
            rate.sleep()

if __name__ == '__main__':
    try:
        node = SimpleAvoider()
        node.run()
    except rospy.ROSInterruptException:
        pass







import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import time


class SimpleAvoider:
    def __init__(self):
        rospy.init_node('simple_dynamic_avoider')

        self.obstacle_threshold = 0.6  # meters

        self.latest_cmd = Twist()
        self.back_cmd = Twist()
        self.back_cmd.linear.x = -0.5  # backward speed

        self.backing_up = False
        self.back_start_time = None
        self.back_duration = 2.5  # seconds

        rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        rospy.Subscriber('/local_cmd_vel', Twist, self.cmd_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

        rospy.loginfo("UGV simple avoider started.")

    def cmd_callback(self, msg):
        self.latest_cmd = msg

    def lidar_callback(self, scan):
        ranges = np.array(scan.ranges)

        # Replace NaNs and infs (Python 2 compatible)
        ranges[np.isnan(ranges)] = 10.0
        ranges[np.isinf(ranges)] = 10.0
        ranges = np.clip(ranges, 0.05, 10.0)

        num_ranges = len(ranges)
        front = ranges[num_ranges // 2 - 10:num_ranges // 2 + 10]

        if np.any(front < self.obstacle_threshold) and not self.backing_up:
            rospy.loginfo("Obstacle within 0.3m. Backing up.")
            self.backing_up = True
            self.back_start_time = time.time()

    def run(self):
        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            now = time.time()

            if self.backing_up:
                #while now - self.back_start_time < self.back_duration:
                self.back_cmd.linear.x = -0.5
                self.back_cmd.angular.z = 0.0
                self.cmd_vel_pub.publish(self.back_cmd) 
                rospy.sleep(1.5)
                self.back_cmd.linear.x = 0.0
                self.back_cmd.angular.z = 0.5
                self.cmd_vel_pub.publish(self.back_cmd) 
                rospy.sleep(1.5)
                self.back_cmd.linear.x = 0.5
                self.back_cmd.angular.z = 0.0
                self.cmd_vel_pub.publish(self.back_cmd) 
                rospy.sleep(1.5)
                self.backing_up = False
                # if now - self.back_start_time < self.back_duration:
                #     self.cmd_vel_pub.publish(self.back_cmd)
                
                # else:
                #     rospy.loginfo("Backup complete. Resuming local control.")
                #     self.back_cmd.linear.x = 0.0
                #     self.back_cmd.angular.z = 0.5
                #     self.cmd_vel_pub.publish(self.back_cmd) 
                #     rospy.sleep(1.5)
                #     self.back_cmd.linear.x = 0.5
                #     self.back_cmd.angular.z = 0.0
                #     self.cmd_vel_pub.publish(self.back_cmd) 
                #     rospy.sleep(1.5)
                #     self.backing_up = False
            else:
                self.cmd_vel_pub.publish(self.latest_cmd)

            rate.sleep()


if __name__ == '__main__':
    try:
        node = SimpleAvoider()
        node.run()
    except rospy.ROSInterruptException:
        pass

        """








#!/usr/bin/env python
# -*- coding: utf-8 -*-

# import rospy
# from sensor_msgs.msg import LaserScan
# from geometry_msgs.msg import Twist
# import numpy as np
# import time


# class SimpleAvoider:
#     def __init__(self):
#         rospy.init_node('simple_dynamic_avoider')

#         self.obstacle_threshold = 0.6  # meters

#         self.latest_cmd = Twist()
#         self.back_cmd = Twist()
#         self.back_cmd.linear.x = -0.5  # backward speed

#         self.backing_up = False
#         self.back_start_time = None
#         self.back_duration = 2.5  # seconds

#         rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
#         rospy.Subscriber('/local_cmd_vel', Twist, self.cmd_callback)
#         self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

#         rospy.loginfo("UGV simple avoider started.")

#     def cmd_callback(self, msg):
#         self.latest_cmd = msg

#     def lidar_callback(self, scan):
#         ranges = np.array(scan.ranges)

#         # Replace NaNs and infs (Python 2 compatible)
#         ranges[np.isnan(ranges)] = 10.0
#         ranges[np.isinf(ranges)] = 10.0
#         ranges = np.clip(ranges, 0.05, 10.0)

#         num_ranges = len(ranges)
#         front = ranges[num_ranges // 2 - 10:num_ranges // 2 + 10]

#         if np.any(front < self.obstacle_threshold) and not self.backing_up:
#             rospy.loginfo("Obstacle within 0.3m. Backing up.")
#             self.backing_up = True
#             self.back_start_time = time.time()

#     def run(self):
#         rate = rospy.Rate(15)
#         while not rospy.is_shutdown():
#             now = time.time()

#             if self.backing_up:
#                 #while now - self.back_start_time < self.back_duration:
#                 self.back_cmd.linear.x = -0.5
#                 self.back_cmd.angular.z = 0.0
#                 self.cmd_vel_pub.publish(self.back_cmd) 
#                 rospy.sleep(1.5)
#                 self.back_cmd.linear.x = 0.0
#                 self.back_cmd.angular.z = 0.5
#                 self.cmd_vel_pub.publish(self.back_cmd) 
#                 rospy.sleep(1.5)
#                 self.back_cmd.linear.x = 0.5
#                 self.back_cmd.angular.z = 0.0
#                 self.cmd_vel_pub.publish(self.back_cmd) 
#                 rospy.sleep(1.5)
#                 self.backing_up = False
#                 if now - self.back_start_time < self.back_duration:
#                     self.cmd_vel_pub.publish(self.back_cmd)
                
#                 else:
#                     rospy.loginfo("Backup complete. Resuming local control.")
#                     self.back_cmd.linear.x = 0.0
#                     self.back_cmd.angular.z = 0.5
#                     self.cmd_vel_pub.publish(self.back_cmd) 
#                     rospy.sleep(1.5)
#                     self.back_cmd.linear.x = 0.5
#                     self.back_cmd.angular.z = 0.0
#                     self.cmd_vel_pub.publish(self.back_cmd) 
#                     rospy.sleep(1.5)
#                     self.backing_up = False
#             else:
#                 self.cmd_vel_pub.publish(self.latest_cmd)

#             rate.sleep()


# if __name__ == '__main__':
#     try:
#         node = SimpleAvoider()
#         node.run()
#     except rospy.ROSInterruptException:
#         pass

















#!/usr/bin/env python3

import rospy
import math
import numpy as np

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path

class VelocitySafetyFilter(object):

    def __init__(self):
        rospy.init_node("velocity_safety_filter", anonymous=False)

        self.latest_cmd = Twist()
        self.scan = None

        # ---------- PARAMETERS ----------
        self.ttc_threshold = 1.0        # seconds
        self.robot_radius = 0.30        # meters
        self.front_fov = 0.6            # radians (~35 deg)
        self.max_turn_speed = 0.8       # rad/s
        self.local_path = None

        rospy.Subscriber("/local_path", Path, self.path_cb, queue_size=1)
        rospy.Subscriber("/local_cmd_vel", Twist, self.cmd_cb, queue_size=1)
        rospy.Subscriber("/scan", LaserScan, self.scan_cb, queue_size=1)

        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        rospy.loginfo("Velocity Safety Filter initialized")

    def path_cb(self, msg):
        self.local_path = msg

    def cmd_cb(self, msg):
        self.latest_cmd = msg

    def scan_cb(self, msg):
        self.scan = msg

    def process(self):
        # No scan yet  passthrough
        if self.scan is None:
            self.cmd_pub.publish(self.latest_cmd)
            return

        v = self.latest_cmd.linear.x
        min_ttc = float("inf")

        angle = self.scan.angle_min

        for r in self.scan.ranges:
            if not np.isfinite(r):
                angle += self.scan.angle_increment
                continue

            # Only consider FRONT beams
            if abs(angle) > self.front_fov:
                angle += self.scan.angle_increment
                continue

            closing_speed = v * math.cos(angle)

            # Ignore non closing beams
            if closing_speed <= 0.0:
                angle += self.scan.angle_increment
                continue

            ttc = (r - self.robot_radius) / closing_speed
            min_ttc = min(min_ttc, ttc)

            angle += self.scan.angle_increment

        cmd = Twist()

        if min_ttc > self.ttc_threshold:
            # SAFE  allow planner command
            cmd = self.latest_cmd
            self.cmd_pub.publish(cmd)
            rospy.sleep(0.5)
        else:
            # UNSAFE  STOP and TURN
            cmd.linear.x = 0.0
            direction = self.choose_turn_direction()
            cmd.angular.z = direction
            self.cmd_pub.publish(cmd)
            rospy.sleep(1.5)
            if self.front_is_clear():
                cmd.linear.x = 0.5
                self.cmd_pub.publish(cmd)
                rospy.sleep(0.5)


    def front_is_clear(self, min_dist=0.8):
        """Returns True if front has enough free space"""
        angle = self.scan.angle_min

        for r in self.scan.ranges:
            if not np.isfinite(r):
                angle += self.scan.angle_increment
                continue

            if abs(angle) < self.front_fov:
                if r < min_dist:
                    return False

            angle += self.scan.angle_increment

        return True

    def choose_turn_direction(self):
        # Default LIDAR-based logic
        angles = np.linspace(
            self.scan.angle_min,
            self.scan.angle_max,
            len(self.scan.ranges)
        )
        ranges = np.array(self.scan.ranges, dtype=np.float32)

        left = ranges[(angles > 0.3) & (angles < 1.5)]
        right = ranges[(angles < -0.3) & (angles > -1.5)]

        left = left[np.isfinite(left)]
        right = right[np.isfinite(right)]

        left_clear = np.mean(left) if len(left) else 0.0
        right_clear = np.mean(right) if len(right) else 0.0

        # Default direction based on LIDAR
        direction = self.max_turn_speed if left_clear > right_clear else -self.max_turn_speed

        # Bias toward local path if available
        if self.local_path and len(self.local_path.poses) > 0:
            # Take first pose in path
            path_pose = self.local_path.poses[0].pose.position
            # Calculate angle to path in robot frame (approx)
            dx = path_pose.x
            dy = path_pose.y
            path_angle = math.atan2(dy, dx)

            # Bias LIDAR decision: prefer turning toward path
            if path_angle > 0:
                direction = max(direction, 0.2)  # slight right bias if path is to left
            else:
                direction = min(direction, -0.2)  # slight left bias if path is to right

        return direction

    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.process()
            rate.sleep()


if __name__ == "__main__":
    try:
        VelocitySafetyFilter().run()
    except rospy.ROSInterruptException:
        pass



