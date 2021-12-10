#!/usr/bin/env python3
"""
ESE 680
RRT assignment
Author: Hongrui Zheng

This file contains the class definition for tree nodes and RRT
Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf
"""
import numpy as np
from numpy import linalg as LA
import math

import rospy
import tf
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry

import matplotlib.pyplot as plt
import cv2
# TODO: import as you need

# class def for tree nodes
# It's up to you if you want to use this
class Node(object):
	def __init__(self):
		self.x = None
		self.y = None
		self.parent = None
		self.cost = None # only used in RRT*
		self.is_root = false

# class def for RRT
class RRT(object):
	def __init__(self):
		pf_topic = '/odom'
		scan_topic = '/scan'
		nav_topic = '/nav'

		# Occupancy Grid
		self.world_size = (500, 200)
		self.occupancy_grid = np.ones(self.world_size)
		self.yaw = 0

		angle_min = -3.141592741
		angle_max =  3.141592741
		angle_inc =  0.005823155
		n = 1080

		self.angles = np.array([angle_min + i*angle_inc for i in range(n)])
		self.current_pos = np.array([0,0])

		rospy.Subscriber(pf_topic, Odometry, self.pf_callback, queue_size=10)
		rospy.Subscriber(scan_topic, LaserScan, self.scan_callback, queue_size=10)
		rospy.Publisher(nav_topic, AckermannDriveStamped, queue_size=10)

	def scan_callback(self, scan_msg):
		# print(self.occupancy_grid)
		# print(len(self.occupancy_grid))
		# print(scan_msg)
		"""
		LaserScan callback, you should update your occupancy grid here

		Args: 
				scan_msg (LaserScan): incoming message from subscribed topic
		Returns:

		"""
		rear_to_lidar = 0.29275
		x_current, y_current = self.current_pos
		heading_current = self.yaw
		x_lidar = x_current + rear_to_lidar * np.cos(heading_current)
		y_lidar = y_current + rear_to_lidar * np.sin(heading_current)

		distances = scan_msg.ranges
		global_angle = heading_current + self.angles
		x_obstacles = x_lidar + distances * np.cos(global_angle)
		y_obstacles = y_lidar + distances * np.sin(global_angle)

		#convert frame
		x_off = 14.5
		y_off = 0.7
		x_grid = x_obstacles + x_off
		y_grid = y_obstacles + y_off
		x_grid /= 0.05
		y_grid /= 0.05

		x_grid = x_grid.round(0).astype(int)
		y_grid = y_grid.round(0).astype(int)

		x_grid[x_grid > 100000] = 0
		y_grid[y_grid > 100000] = 0

		for i in range(np.maximum(x_grid[0] -6, 0), np.minimum(x_grid[0] +6, self.world_size[0]-1)):
			for j in range(np.maximum(y_grid[0] -6, 0), np.minimum(y_grid[0] +6, self.world_size[1]-1)):
				# print(i, j)
				self.occupancy_grid[i][j] = 0

		# cv2.imshow('Maps', cv2.resize(self.occupancy_grid, (960, 540))) 
		# cv2.waitKey(3)


		return None

	def pf_callback(self, pose_msg):
		"""
		The pose callback when subscribed to particle filter's inferred pose
		Here is where the main RRT loop happens

		Args: 
				pose_msg (PoseStamped): incoming message from subscribed topic
		Returns:

		"""
		position = pose_msg.pose.pose.position
		orientation = pose_msg.pose.pose.orientation
		self.current_pos = np.array([position.x, position.y])
		
		quarternion = [orientation.x, orientation.y, orientation.z, orientation.w]
		(roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)
		
		# robot orientation and position
		self.yaw = yaw
		print(self.current_pos)
		# print(self.yaw)

		return None

	def nav_callback(self, nav_msg):
		return None
	
	def sample(self):
		"""
		This method should randomly sample the free space, and returns a viable point

		Args:
		Returns:
				(x, y) (float float): a tuple representing the sampled point

		"""
		x = None
		y = None
		return (x, y)

	def nearest(self, tree, sampled_point):
		"""
		This method should return the nearest node on the tree to the sampled point

		Args:
				tree ([]): the current RRT tree
				sampled_point (tuple of (float, float)): point sampled in free space
		Returns:
				nearest_node (int): index of neareset node on the tree
		"""
		nearest_node = 0
		return nearest_node

	def steer(self, nearest_node, sampled_point):
		"""
		This method should return a point in the viable set such that it is closer 
		to the nearest_node than sampled_point is.

		Args:
				nearest_node (Node): nearest node on the tree to the sampled point
				sampled_point (tuple of (float, float)): sampled point
		Returns:
				new_node (Node): new node created from steering
		"""
		new_node = None
		return new_node

	def check_collision(self, nearest_node, new_node):
		"""
		This method should return whether the path between nearest and new_node is
		collision free.

		Args:
				nearest (Node): nearest node on the tree
				new_node (Node): new node from steering
		Returns:
				collision (bool): whether the path between the two nodes are in collision
				                  with the occupancy grid
		"""
		return True

	def is_goal(self, latest_added_node, goal_x, goal_y):
		"""
		This method should return whether the latest added node is close enough
		to the goal.

		Args:
				latest_added_node (Node): latest added node on the tree
				goal_x (double): x coordinate of the current goal
				goal_y (double): y coordinate of the current goal
		Returns:
				close_enough (bool): true if node is close enoughg to the goal
		"""
		return False

	def find_path(self, tree, latest_added_node):
		"""
		This method returns a path as a list of Nodes connecting the starting point to
		the goal once the latest added node is close enough to the goal

		Args:
				tree ([]): current tree as a list of Nodes
				latest_added_node (Node): latest added node in the tree
		Returns:
				path ([]): valid path as a list of Nodes
		"""
		path = []
		return path



	# The following methods are needed for RRT* and not RRT
	def cost(self, tree, node):
		"""
		This method should return the cost of a node

		Args:
				node (Node): the current node the cost is calculated for
		Returns:
				cost (float): the cost value of the node
		"""
		return 0

	def line_cost(self, n1, n2):
		"""
		This method should return the cost of the straight line between n1 and n2

		Args:
				n1 (Node): node at one end of the straight line
				n2 (Node): node at the other end of the straint line
		Returns:
				cost (float): the cost value of the line
		"""
		return 0

	def near(self, tree, node):
		"""
		This method should return the neighborhood of nodes around the given node

		Args:
				tree ([]): current tree as a list of Nodes
				node (Node): current node we're finding neighbors for
		Returns:a
				neighborhood ([]): neighborhood of nodes as a list of Nodes
		"""
		neighborhood = []
		return neighborhood

def main():
	rospy.init_node('rrt', anonymous=True)
	rrt = RRT()
	rospy.spin()

if __name__ == '__main__':
	main()



