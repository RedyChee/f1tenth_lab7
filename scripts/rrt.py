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

import sys
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
		self.is_root = False

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
		self.resolution = 0.05
		self.plot_width = 12

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
		# reset occupancy grid
		self.occupancy_grid = np.ones(self.world_size)

		# convert frame
		x_obstacles, y_obstacles = self.lidar_to_global(scan_msg.ranges)
		x_grids, y_grids = self.global_to_grid(x_obstacles, y_obstacles)

		# update occupancy grid
		for i in range(len(scan_msg.ranges)):
			x_grid, y_grid = x_grids[i], y_grids[i]
			plot_width = self.plot_width #px

			for j in range(np.maximum(x_grid - plot_width//2, 0), np.minimum(x_grid + plot_width//2, self.world_size[0]-1)+1):
				for k in range(np.maximum(y_grid - plot_width//2, 0), np.minimum(y_grid + plot_width//2, self.world_size[1]-1)+1):
					self.occupancy_grid[j][k] = 0

		# visualize occupancy grid
		cv2.imshow('Maps', self.occupancy_grid) 
		cv2.waitKey(3)


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
		
		quarternion = [orientation.x, orientation.y, orientation.z, orientation.w]
		(roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)
		
		# update robot orientation and position
		self.yaw = yaw
		self.current_pos = np.array([position.x, position.y])
		# print(self.current_pos)
		# print(self.yaw)

		return None

	def nav_callback(self, nav_msg):
		return None
	
	def lidar_to_global(self, distances):
		# lidar -> car -> global
		rear_to_lidar = 0.29275
		x_lidar = self.current_pos[0] + rear_to_lidar * np.cos(self.yaw)
		y_lidar = self.current_pos[1] + rear_to_lidar * np.sin(self.yaw)

		global_angle = self.yaw + self.angles
		x_obstacles = x_lidar + distances * np.cos(global_angle)
		y_obstacles = y_lidar + distances * np.sin(global_angle)

		return x_obstacles, y_obstacles
	
	def global_to_grid(self, x_global, y_global):
		x_off = 14.5
		y_off = 0.7

		x_grid = (x_global + x_off)/self.resolution
		y_grid = (y_global + y_off)/self.resolution

		# filter out of range values
		x_grid[x_grid > 100000] = 0
		y_grid[y_grid > 100000] = 0

		return x_grid.round(0).astype(int), y_grid.round(0).astype(int)

	
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



