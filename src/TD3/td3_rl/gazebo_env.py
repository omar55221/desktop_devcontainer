import math
import random
import time

import numpy as np
import rclpy
import rclpy.logging
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from squaternion import Quaternion
from std_srvs.srv import Empty
from visualization_msgs.msg import Marker, MarkerArray

GOAL_REACHED_DIST = 0.5
COLLISION_DIST = 0.25
TIME_DELTA = 0.1


def check_pos(x, y):
    # Check if goal position is in an obstacle
    obstacles = [
        (-3.8, -6.2, 6.2, 3.8), (-1.3, -2.7, 4.7, -0.2), (-0.3, -4.2, 2.7, 1.3),
        (-0.8, -4.2, -2.3, -4.2), (-1.3, -3.7, -0.8, -2.7), (4.2, 0.8, -1.8, -3.2),
        (4, 2.5, 0.7, -3.2), (6.2, 3.8, -3.3, -4.2), (4.2, 1.3, 3.7, 1.5), (-3.0, -7.2, 0.5, -1.5)
    ]
    if any(x1 > x > x2 and y1 > y > y2 for x1, x2, y1, y2 in obstacles) or not (-4.5 <= x <= 4.5 and -4.5 <= y <= 4.5):
        return False
    return True


class GazeboEnv(Node):
    def __init__(self, environment_dim):
        super().__init__('gazebo_env')
        self.environment_dim = environment_dim
        self.odom_x = 0
        self.odom_y = 0

        self.goal_x = 1
        self.goal_y = 0.0
        
        self.max_distance = 3.5

        self.upper, self.lower = 5.0, -5.0
        self.scan_data = np.ones(self.environment_dim) * self.max_distance
        self.last_odom = None

        # Set up the ROS publishers and subscribers
        self.vel_pub = self.create_publisher(Twist, "/cmd_vel", 1)
        self.goal_point_publisher = self.create_publisher(MarkerArray, "goal_point", 3)
        
        self.scan_sub = self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        
        self.set_entity_state = self.create_client(SetEntityState, "/gazebo/set_entity_state")
        self.unpause = self.create_client(Empty, "/unpause_physics")
        self.pause = self.create_client(Empty, "/pause_physics")
        self.reset_proxy = self.create_client(Empty, "/reset_world")

    def scan_callback(self, scan):
        mod = len(scan.ranges) // self.environment_dim
        self.scan_data = [
            min(self.max_distance, scan.ranges[i]) if not np.isnan(scan.ranges[i]) else 0
            for i in range(0, len(scan.ranges), mod)
        ]

    def odom_callback(self, od_data):
        self.last_odom = od_data

    # Perform an action and read a new state
    def step(self, action):
        # Ensure action is of float type
        action = [float(a) for a in action]
        
        # Publish the robot action
        vel_cmd = Twist()
        vel_cmd.linear.x = action[0]
        vel_cmd.angular.z = action[1]
        self.vel_pub.publish(vel_cmd)
        self.publish_markers(action)

        self.unpause.call_async(Empty.Request())
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=TIME_DELTA))
        self.pause.call_async(Empty.Request())

        done, collision, min_laser = self.observe_collision(self.scan_data)
        
        self.odom_x = self.last_odom.pose.pose.position.x
        self.odom_y = self.last_odom.pose.pose.position.y

        quaternion = Quaternion(
            self.last_odom.pose.pose.orientation.w,
            self.last_odom.pose.pose.orientation.x,
            self.last_odom.pose.pose.orientation.y,
            self.last_odom.pose.pose.orientation.z,
        )
        euler = quaternion.to_euler(degrees=False)
        angle = round(euler[2], 4)

        # Calculate distance to the goal from the robot
        distance = np.linalg.norm(
            [self.odom_x - self.goal_x, self.odom_y - self.goal_y]
        )

        # Calculate the relative angle between the robots heading and heading toward the goal
        skew_x = self.goal_x - self.odom_x
        skew_y = self.goal_y - self.odom_y
        dot = skew_x * 1 + skew_y * 0
        mag1 = math.sqrt(math.pow(skew_x, 2) + math.pow(skew_y, 2))
        mag2 = math.sqrt(math.pow(1, 2) + math.pow(0, 2))
        beta = math.acos(dot / (mag1 * mag2))
        if skew_y < 0:
            if skew_x < 0:
                beta = -beta
            else:
                beta = 0 - beta
        theta = beta - angle
        if theta > np.pi:
            theta = np.pi - theta
            theta = -np.pi - theta
        if theta < -np.pi:
            theta = -np.pi - theta
            theta = np.pi - theta

        # Detect if the goal has been reached and give a large positive reward
        target = distance < GOAL_REACHED_DIST
        done = done or target
        robot_state = [distance, theta, action[0], action[1]]
        state = np.append(self.scan_data, robot_state)

        reward = self.get_reward(target, collision, action, min_laser)
        return state, reward, done, target

    def reset(self):
        # Resets the state of the environment and returns an initial observation.
        self.reset_proxy.call_async(Empty.Request())

        angle = np.random.uniform(-np.pi, np.pi)        

        x = 0
        y = 0
        position_ok = False
        while not position_ok:
            x = np.random.uniform(-4.5, 4.5)
            y = np.random.uniform(-4.5, 4.5)
            position_ok = check_pos(x, y)
        
        self.change_object_position("burger", x, y, angle)

        self.odom_x = x
        self.odom_y = y

        # set a random goal in empty space in environment
        self.change_goal()
        # randomly scatter boxes in the environment
        self.random_box()
        self.publish_markers([0.0, 0.0])

        self.unpause.call_async(Empty.Request())
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=TIME_DELTA))
        self.pause.call_async(Empty.Request())

        distance = np.linalg.norm(
            [self.odom_x - self.goal_x, self.odom_y - self.goal_y]
        )

        skew_x = self.goal_x - self.odom_x
        skew_y = self.goal_y - self.odom_y

        dot = skew_x * 1 + skew_y * 0
        mag1 = math.sqrt(math.pow(skew_x, 2) + math.pow(skew_y, 2))
        mag2 = math.sqrt(math.pow(1, 2) + math.pow(0, 2))
        beta = math.acos(dot / (mag1 * mag2))

        if skew_y < 0:
            if skew_x < 0:
                beta = -beta
            else:
                beta = 0 - beta
        theta = beta - angle

        if theta > np.pi:
            theta = np.pi - theta
            theta = -np.pi - theta
        if theta < -np.pi:
            theta = -np.pi - theta
            theta = np.pi - theta

        robot_state = [distance, theta, 0.0, 0.0]
        state = np.append(self.scan_data, robot_state)
        return state
    
    def change_object_position(self, name, x, y, angle):
        quaternion = Quaternion.from_euler(0.0, 0.0, angle)
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = 0.0
        pose.orientation.x = quaternion.x
        pose.orientation.y = quaternion.y
        pose.orientation.z = quaternion.z
        pose.orientation.w = quaternion.w
        
        request = SetEntityState.Request()
        request.state.name = name
        request.state.pose = pose
        self.set_entity_state.call_async(request)

    def change_goal(self):
        # Place a new goal and check if its location is not on one of the obstacles
        if self.upper < 10:
            self.upper += 0.004
        if self.lower > -10:
            self.lower -= 0.004

        goal_ok = False

        while not goal_ok:
            self.goal_x = self.odom_x + random.uniform(self.upper, self.lower)
            self.goal_y = self.odom_y + random.uniform(self.upper, self.lower)
            goal_ok = check_pos(self.goal_x, self.goal_y)

    def random_box(self):
        # Randomly change the location of the boxes in the environment on each reset to randomize the training
        # environment
        for i in range(4):
            name = "cardboard_box_" + str(i)

            x, y = 0, 0
            box_ok = False
            while not box_ok:
                x = np.random.uniform(-6, 6)
                y = np.random.uniform(-6, 6)
                box_ok = check_pos(x, y)
                distance_to_robot = np.linalg.norm([x - self.odom_x, y - self.odom_y])
                distance_to_goal = np.linalg.norm([x - self.goal_x, y - self.goal_y])
                if distance_to_robot < 1.5 or distance_to_goal < 1.5:
                    box_ok = False
            
            self.change_object_position(name, x, y, 0.0)

    def publish_markers(self, action):
        # Publish visual data in Rviz
        markerArray = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.type = marker.CYLINDER
        marker.action = marker.ADD
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.01
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = self.goal_x
        marker.pose.position.y = self.goal_y
        marker.pose.position.z = 0.0

        markerArray.markers.append(marker)

        self.goal_point_publisher.publish(markerArray)

    @staticmethod
    def observe_collision(laser_data):
        # Detect a collision from laser data
        min_laser = min(laser_data)
        if min_laser < COLLISION_DIST:
            return True, True, min_laser
        return False, False, min_laser

    @staticmethod
    def get_reward(target, collision, action, min_laser):
        if target:
            return 200.0
        elif collision:
            return -100.0
        else:
            r3 = lambda x: 1 - x if x < 1 else 0.0
            return action[0] / 2 - abs(action[1]) / 2 - r3(min_laser) / 2
