#!/usr/bin/env python

"""
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Bool
import numpy as np
import heapq
import math


class AStarGlobalPlanner:
    def __init__(self):
        rospy.init_node('global_planner', anonymous=True)

        # Subscribers
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.inflated_map_sub = rospy.Subscriber('global_costmap', OccupancyGrid, self.inflated_map_callback)
        self.amcl_sub = rospy.Subscriber('/poseupdate', PoseWithCovarianceStamped, self.amcl_callback)
        self.replan_sub = rospy.Subscriber('/replan_path', Bool, self.replan_callback)

        # Publishers
        self.path_pub = rospy.Publisher('/global_path', Path, queue_size=10)
        self.replan_ack_pub = rospy.Publisher('/replan_ack', Bool, queue_size=10)  # Replan acknowledgment

        # Internal state
        self.map_data = None
        self.costmap_data = None
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0
        self.map_origin = (0, 0)
        self.robot_pose = None  # To store robot's current position
        self.goal_msg = None  # Store full goal message for continuous updates
        self.replan_ack = False

        # Timer for continuous replanning
        #self.timer = rospy.Timer(rospy.Duration(5), self.update_global_path)

    def amcl_callback(self, msg):
        self.robot_pose = msg.pose.pose

    def map_callback(self, msg):
        if not msg.data:
            rospy.logwarn("Received empty map data!")
            return
        self.map_data = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin = (msg.info.origin.position.x, msg.info.origin.position.y)

    def inflated_map_callback(self, msg):
        if not msg.data:
            rospy.logwarn("Received empty costmap data!")
            return
        self.costmap_data = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))

    def goal_callback(self, goal_msg):
        self.goal_msg = goal_msg  # Store goal for continuous update
        #rospy.loginfo("New goal received: (%.2f, %.2f)", goal_msg.pose.position.x, goal_msg.pose.position.y)

    def is_valid_cell(self, x, y):
        if 0 <= x < self.map_width and 0 <= y < self.map_height:
            cell_cost = self.costmap_data[y][x]
            return 0 <= cell_cost < 100
        return False

    def world_to_map(self, x, y):
        mx = int((x - self.map_origin[0]) / self.map_resolution)
        my = int((y - self.map_origin[1]) / self.map_resolution)
        return mx, my

    def map_to_world(self, mx, my):
        wx = mx * self.map_resolution + self.map_origin[0]
        wy = my * self.map_resolution + self.map_origin[1]
        return wx, wy

    def heuristic(self, a, b):
        return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

    def a_star(self, start, goal):
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        open_list = []
        heapq.heappush(open_list, (0, start))
        costs = {start: 0}
        came_from = {}

        while open_list:
            _, current = heapq.heappop(open_list)
            if current == goal:
                return self.reconstruct_path(came_from, start, goal)
            for dx, dy in directions:
                neighbor = (current[0] + dx, current[1] + dy)
                if self.is_valid_cell(neighbor[0], neighbor[1]):
                    new_cost = costs[current] + 1
                    heuristic_cost = new_cost + self.heuristic(neighbor, goal)
                    if neighbor not in costs or new_cost < costs[neighbor]:
                        costs[neighbor] = new_cost
                        heapq.heappush(open_list, (heuristic_cost, neighbor))
                        came_from[neighbor] = current
        return None

    def reconstruct_path(self, came_from, start, goal):
        current = goal
        path = []
        while current != start:
            path.append(current)
            current = came_from[current]
        path.reverse()
        return path

    def publish_path(self, path):
        global_path = Path()
        global_path.header.frame_id = "map"
        global_path.header.stamp = rospy.Time.now()
        for cell in path:
            pose = PoseStamped()
            wx, wy = self.map_to_world(cell[0], cell[1])
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.orientation.w = 1.0
            global_path.poses.append(pose)
        self.path_pub.publish(global_path)
        rospy.loginfo("Global path published.")

    def replan_callback(self, msg):
        if msg.data and self.robot_pose and self.goal_msg:
            rospy.logwarn("Manual replan requested.")
            self.update_global_path(None)
            self.replan_ack_pub.publish(True)

    def update_global_path(self, event):
        if not all([self.goal_msg, self.robot_pose, self.map_data is not None, self.costmap_data is not None]):
            return

        start = self.world_to_map(self.robot_pose.position.x, self.robot_pose.position.y)
        goal = self.world_to_map(self.goal_msg.pose.position.x, self.goal_msg.pose.position.y)

        if not self.is_valid_cell(*goal):
            rospy.logwarn_throttle(0.5, "Goal is inside an obstacle.")
            return

        path = self.a_star(start, goal)
        if path:
            self.publish_path(path)
        else:
            rospy.logwarn_throttle(0.5, "No path found during update.")

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    planner = AStarGlobalPlanner()
    planner.run()

    
    """







#!/usr/bin/env python




import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Bool
import heapq
import math
import numpy as np
import scipy.ndimage


class AStarGlobalPlanner:
    def __init__(self):
        rospy.init_node('lightweight_global_planner', anonymous=True)

        # Internal state
        self.map = None
        self.inflated_map = None
        self.map_width = 0
        self.map_height = 0
        self.resolution = 0
        self.origin = (0, 0)
        self.robot_pose = None
        self.goal = None

        self.inflation_radius = 0.35  # meters

        # Subscribers
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.pose_sub = rospy.Subscriber('/poseupdate', PoseWithCovarianceStamped, self.pose_callback)
        self.replan_sub = rospy.Subscriber('/replan_path', Bool, self.replan_callback)

        # Publishers
        self.path_pub = rospy.Publisher('/global_path', Path, queue_size=10)
        self.replan_ack_pub = rospy.Publisher('/replan_ack', Bool, queue_size=10)

  
    def map_callback(self, msg):
        if not msg.data:
            rospy.logwarn("Received empty map.")
            return

        self.map = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.resolution = msg.info.resolution
        self.origin = (msg.info.origin.position.x, msg.info.origin.position.y)

        self.inflate_obstacles()
        rospy.loginfo("Static map received and inflated.")

    def inflate_obstacles(self):
        #Inflate obstacles by self.inflation_radius (in meters).
        inflation_cells = int(self.inflation_radius / self.resolution)

        # Mark all non-zero (occupied or unknown) as 1, rest as 0
        binary_map = (self.map > 0).astype(np.uint8)

        # Apply binary dilation to inflate obstacles
        structure = np.ones((2 * inflation_cells + 1, 2 * inflation_cells + 1), dtype=np.uint8)
        inflated = scipy.ndimage.binary_dilation(binary_map, structure=structure).astype(np.uint8)

        self.inflated_map = inflated

    def pose_callback(self, msg):
        self.robot_pose = msg.pose.pose

    def world_to_map(self, x, y):
        mx = int((x - self.origin[0]) / self.resolution)
        my = int((y - self.origin[1]) / self.resolution)
        return mx, my

    def map_to_world(self, mx, my):
        x = mx * self.resolution + self.origin[0] + self.resolution / 2
        y = my * self.resolution + self.origin[1] + self.resolution / 2
        return x, y

    def is_valid(self, x, y):
        if 0 <= x < self.map_width and 0 <= y < self.map_height:
            return self.inflated_map[y][x] == 0
        return False

    def heuristic(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def a_star(self, start, goal):
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}

        directions = [(-1, 0), (1, 0), (0, -1), (0, 1),
                      (-1, -1), (-1, 1), (1, -1), (1, 1)]

        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal:
                return self.reconstruct_path(came_from, current)

            for dx, dy in directions:
                neighbor = (current[0] + dx, current[1] + dy)
                if not self.is_valid(neighbor[0], neighbor[1]):
                    continue

                move_cost = math.hypot(dx, dy)
                tentative_g = g_score[current] + move_cost

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score, neighbor))

        return None

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return path[::-1]

    def goal_callback(self, msg):
        if self.map is None or self.robot_pose is None:
            rospy.logwarn("Missing map or pose.")
            return

        start = self.world_to_map(self.robot_pose.position.x, self.robot_pose.position.y)
        goal = self.world_to_map(msg.pose.position.x, msg.pose.position.y)

        if not self.is_valid(*goal):
            rospy.logwarn("Goal is not reachable.")
            return

        rospy.loginfo("Planning path from {} to {}".format(start, goal))
        path = self.a_star(start, goal)

        if path:
            self.publish_path(path)
            self.goal = msg
            self.replan_ack_pub.publish(True)
        else:
            rospy.logwarn("No path found.")

    def replan_callback(self, msg):
        if msg.data and self.goal and self.robot_pose:
            rospy.loginfo("Replanning triggered.")
            self.goal_callback(self.goal)

    def publish_path(self, path):
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.Time.now()

        for cell in path:
            pose = PoseStamped()
            x, y = self.map_to_world(cell[0], cell[1])
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        rospy.loginfo("Published path with {} points.".format(len(path)))

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    AStarGlobalPlanner().run()
