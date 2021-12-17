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
import random
import math
import os
import sys
import rospy
import rospkg
import tf
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

import matplotlib.pyplot as plt
import cv2

from visualizer import plot_marker, plot_sample



# TODO: import as you need

# class def for tree nodes
# It's up to you if you want to use this
class Node(object):
	def __init__(self, x=None, y=None, parent=None, cost=None, is_root=False):
		self.x = x
		self.y = y
		self.parent = parent
		self.cost = cost # only used in RRT*
		self.is_root = is_root

# class def for RRT
class RRT(object):
	def __init__(self, gp_path):
		pf_topic = '/odom'
		scan_topic = '/scan'
		nav_topic = '/nav'

		# Goalpoints
		self.goalpoints = np.genfromtxt(gp_path, delimiter=',')[:, :2]
		self.l = 2.3 #radial distance to goalpoint from car
		self.STEER_LENGTH = 0.35
		self.MINIMUM_GOAL_DISTANCE = 0.1
		self.LOOKAHEAD_DISTANCE = 0.6

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

		rospy.Subscriber(pf_topic, Odometry, self.pf_callback, queue_size=1)
		rospy.Subscriber(scan_topic, LaserScan, self.scan_callback, queue_size=1)
		self.marker_pub = rospy.Publisher('/goal_point', Marker, queue_size = 1)
		self.sample_pub = rospy.Publisher('/sample_point', Marker, queue_size = 1)
		self.drive_pub = rospy.Publisher(nav_topic, AckermannDriveStamped, queue_size = 1)


	def scan_callback(self, scan_msg):
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
		# _, grid_img = cv2.threshold(self.occupancy_grid, 0, 1, cv2.THRESH_BINARY)
		# cv2.namedWindow('Maps', cv2.WINDOW_NORMAL)
		# cv2.resizeWindow('Maps', (960,540))
		# cv2.imshow('Maps', grid_img) 
		cv2.imshow('Maps', self.occupancy_grid) 
		cv2.waitKey(3)



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
		trans = np.array([position.x, position.y, position.z])
		# update robot orientation and position
		self.yaw = yaw
		self.current_pos = np.array([position.x, position.y])
		rospy.logdebug("quartenion = %s", quarternion)
		rospy.logdebug("self.current_pos = %s", self.current_pos)
		# print(self.current_pos)
		# print(self.yaw)

		self.tr_global_to_car = self.get_tr_matrix(quarternion, trans)

		goal_x, goal_y = self.get_goalpoint(self.tr_global_to_car, plot=True)

		tree = []
		paths = []
		starting_node = Node(x=self.current_pos[0], y=self.current_pos[1], is_root=True)
		tree.append(starting_node)

		MAX_ITERATION = 100
		for i in range(MAX_ITERATION):
			sampled_point = self.sample()
			nearest_node_idx, min_distance = self.nearest(tree, sampled_point)
			# rospy.logdebug("min_distance = %s", min_distance)
			new_node = self.steer(tree[nearest_node_idx], sampled_point, min_distance)
			new_node.parent = nearest_node_idx;                
			if (not self.check_collision(tree[nearest_node_idx], new_node)):
				# rrt
				tree.append(new_node)
				# rospy.logdebug("tree = %s", tree)
				if (self.is_goal(new_node, goal_x, goal_y)):
					paths = self.find_path(tree, new_node)
					# rospy.logdebug("paths = %s", paths)
					break
		
		# Pure Pursuit
		l = len(paths)
		rospy.logdebug("l = %s", l)
		if (l==0):
			pass
		else:
			for i in range(l):
				distance = ((paths[l -1 -i].x - self.current_pos[0])**2 + (paths[l -1 -i].y - self.current_pos[1])**2)**0.5
				if (distance >= self.LOOKAHEAD_DISTANCE):
					### BUG HERE ###
					x_target = paths[l -1 -i].x
					y_target = paths[l -1 -i].y
					break

			siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
			cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
			heading_current = np.arctan2(siny_cosp, cosy_cosp)

			# using Pure Pursuit algorithm to navigate the car
			real_distance = ((x_target - self.current_pos[0])**2 + (y_target - self.current_pos[1])**2)**0.5
			lookahead_angle = np.arctan2(y_target - self.current_pos[1], x_target - self.current_pos[0])
			del_y = real_distance * np.sin(lookahead_angle - heading_current)
			angle = 2.00 * del_y / (real_distance **2 )
			rospy.logdebug("angle = %s", angle)
			self.steer_pure_pursuit(angle)



	def steer_pure_pursuit(self, angle): 	
		if -np.pi/18 < angle < np.pi/18:
			velocity = 1
		elif -np.pi/9 < angle <= -np.pi/18 or np.pi/18 <= angle < np.pi/9:
			velocity = 1
		else:
			velocity = 1.5
			
		drive_msg = AckermannDriveStamped()
		drive_msg.header.stamp = rospy.Time.now()
		drive_msg.header.frame_id = "laser"
		drive_msg.drive.steering_angle = angle
		drive_msg.drive.speed = velocity
		self.drive_pub.publish(drive_msg)		

	def nav_callback(self, nav_msg):
		return None
	
	def get_goalpoint(self, tr_global_to_car, plot=True):
		n = len(self.goalpoints)
		# rospy.logdebug("self.goalpoints = %s", self.goalpoints)
		ipt = np.zeros((4, n))
		ipt[:2, :] = self.goalpoints.T
		ipt[3, :] = 1

		#transform to base link (car's frame)
		opt = np.linalg.inv(tr_global_to_car).dot(ipt)
		xy = opt[:2, :].T #transformed
		xy[xy[:,0]<0] = 10 #filter points behind the car
		# xy[xy[:,0]<2.3] = 10

		#select goal point
		distance = np.sum(xy**2, axis=1)
		idx = np.argmin(np.absolute(distance-self.l**2))
		# rospy.logdebug("min_dist = %s", np.min(np.absolute(distance-self.l**2)))
		goal_x, goal_y = self.goalpoints[idx]
		# rospy.logdebug("current_pos = %s", self.current_pos)

		if plot:
			rospy.logdebug("goal_xy = %s", self.goalpoints[idx])
			plot_marker(self.marker_pub, goal_x, goal_y) #visualize goal point

		return goal_x, goal_y
	
	def get_tr_matrix(self, quarternion, trans):
		# get 4x4 transformation matrix
		tr = np.zeros((4,4))
		rot = tf.transformations.quaternion_matrix(quarternion)[:3, :3]
		tr[:3,:3] = rot
		tr[:3, 3] = trans
		tr[-1, -1] = 1 
		return tr

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
		# x_grid[(x_grid > 100000) | (x_grid < 0)] = 0
		# y_grid[(y_grid > 100000) | (y_grid < 0)] = 0
		x_grid[(x_grid >= 499.5)] = 499
		y_grid[(y_grid >= 199.5)] = 199
		x_grid[((x_grid < 0))] = 0
		y_grid[((y_grid < 0))] = 0

		return x_grid.round(0).astype(int), y_grid.round(0).astype(int)

	
	def sample(self):
		"""
		This method should randomly sample the free space, and returns a viable point

		Args:
		Returns:
				(x, y) (float float): a tuple representing the sampled point

		"""
		# rospy.logdebug("%s", self.occupancy_grid)
		# coordinates = np.argwhere(self.occupancy_grid == 1) #why ==0?
		# rospy.logdebug("%s", type(coordinates))
		# rospy.logdebug("%s", coordinates)
		# xy = random.choice(coordinates)
		self.x_limit_top = 2.5
		self.x_limit_bot = 0.5
		self.y_limit_left = 0.4
		self.y_limit_right = -0.88

		# listener = tf.TransformListener()
		# listener.waitForTransform('/map', '/base_link', rospy.Time.now(), rospy.Duration(2.0))
		# trans,rot = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
		# tr = self.get_tr_matrix(rot,trans)

		x = np.random.uniform(self.x_limit_bot, self.x_limit_top)
		y = np.random.uniform(self.y_limit_right, self.y_limit_left)

		xy = self.tr_global_to_car.dot(np.array([x, y , 0, 1]))[:2]
		# self.x_limit_top = tr.dot([self.x_limit_top, 0 , 0, 1])[0]
		# self.x_limit_bot = tr.dot([self.x_limit_bot, 0 , 0, 1])[0]
		# self.y_limit_left = tr.dot([0, self.y_limit_left , 0, 1])[1]
		# self.y_limit_right = tr.dot([0, self.y_limit_right , 0, 1])[1]
		# rospy.logdebug("self.x_limit_top = %s", self.x_limit_top)
		# rospy.logdebug("self.x_limit_bot = %s", self.x_limit_bot)
		# rospy.logdebug("self.y_limit_left = %s", self.y_limit_left)
		# rospy.logdebug("self.y_limit_right = %s", self.y_limit_right)
		# x = np.random.uniform(self.x_limit_bot, self.x_limit_top)
		# y = np.random.uniform(self.y_limit_right, self.y_limit_left)
		rospy.logdebug("xy = %s", xy)
		return xy

	def nearest(self, tree, sampled_point):
		"""
		This method should return the nearest node on the tree to the sampled point

		Args:
				tree ([]): the current RRT tree
				sampled_point (tuple of (float, float)): point sampled in free space
		Returns:
				nearest_node (int): index of neareset node on the tree
		"""
		nearest_node_idx = 0
		# sx, sy = sampled points 
		sx, sy = sampled_point
		min_distance = ((tree[0].x - sx)**2 + (tree[0].y - sy)**2)**0.5
		for idx in range(1, len(tree)):
			node = tree[idx]
			distance = ((node.x - sx)**2 + (node.y - sy)**2)**0.5

			if distance < min_distance:
				min_distance = distance
				nearest_node_idx = idx
		rospy.logdebug("nearest node = %s, %s", tree[nearest_node_idx].x, tree[nearest_node_idx].y)		
		rospy.logdebug("min_dist with nearest node = %s", min_distance)
		return nearest_node_idx, min_distance

	def steer(self, nearest_node, sampled_point, act_distance):
		"""
		This method should return a point in the viable set such that it is closer 
		to the nearest_node than sampled_point is.

		Args:
				nearest_node (Node): nearest node on the tree to the sampled point
				sampled_point (tuple of (float, float)): sampled point
		Returns:
				new_node (Node): new node created from steering
		"""
		x = nearest_node.x + self.STEER_LENGTH / act_distance * (sampled_point[0] - nearest_node.x)
		y = nearest_node.y + self.STEER_LENGTH / act_distance * (sampled_point[1] - nearest_node.y)
		plot_sample(self.sample_pub, x, y)
		rospy.logdebug("steer_node = %s, %s", x, y)
		new_node = Node(x=x, y=y)
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
		collision = False 
		for i in range(100):
			x, y = np.array([nearest_node.x + i * 0.01 * (new_node.x - nearest_node.x)]), np.array([nearest_node.y + i * 0.01 * (new_node.y - nearest_node.y)])
			grid_x, grid_y = self.global_to_grid(x, y)
			grid_x, grid_y = grid_x[0], grid_y[0]
			# if grid_x >= 500 or grid_y >= 200:
				# rospy.logdebug("%s", (grid_x, grid_y))
			if self.occupancy_grid[grid_x][grid_y] == 0: # why not == 0
				collision = True
		
		# print(f'collision: {collision}')

		return collision

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
		close_enough = False
		distance = ((latest_added_node.x - goal_x)**2 + (latest_added_node.y - goal_y)**2)**0.5
		if distance < self.MINIMUM_GOAL_DISTANCE:  # why not <=
			close_enough = True
			rospy.logdebug("distance = %s", distance)
		# rospy.logdebug("close = %s", close_enough)
		return close_enough

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
		path.append(latest_added_node)
		next_node = tree[latest_added_node.parent]

		while (not next_node.is_root):
			path.append(next_node)
			next_node = tree[next_node.parent]
		
		path.append(tree[0])
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
	rospy.init_node('rrt', anonymous=True, log_level=rospy.DEBUG)
	rospack = rospkg.RosPack()
	package_path = rospack.get_path('f1tenth_lab7')
	gp_path = os.path.join(package_path, 'logs/waypoints.csv')

	rrt = RRT(gp_path=gp_path)
	rospy.spin()

if __name__ == '__main__':
	main()



