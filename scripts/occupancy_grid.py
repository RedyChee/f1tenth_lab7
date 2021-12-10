#!/usr/bin/env python

import rospy
import tf
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import cv2

import matplotlib.pyplot as plt
class OccupancyGrid:
    def __init__(self, origin, size, resolution):
        self.origin = origin
        self.gridsize = size
        self.resolution = resolution
        self.robot_x = 0
        self.robot_y = 0
        self.gridmap = -1 * np.ones((size[0], size[1]))
        self.theta = 0
        self.count = 0

        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size=10)
        self.rate = rospy.Rate(10)



    def scan_callback(self, msg):
        data = msg.ranges
        dist_maxrange = msg.range_max

        # enumerate through the ranges
        for i, z in enumerate(data):
            self.update_grid((self.robot_x, self.robot_y), z, dist_maxrange, i)

        cv2.imshow('Maps', cv2.resize(self.gridmap, (960, 540)))
        # imS = cv2.resize(im, (960, 540))    
        cv2.waitKey(3)
        # if self.count == 10000:
        #     plt.imshow(self.gridmap)
        # self.count +=1


    def odom_callback(self, msg):
        quarternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,\
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)

        # robot orientation and position
        self.theta = yaw
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
 

    # Update the gridmap
    def update_grid(self, pos, dist, maxrange, theta):

        occupied = True

        # robot position
        wx = pos[0]
        wy = pos[1]
        theta = math.radians(theta)
        angle = theta + self.theta

        # robot position in grid
        gpos = self.to_grid((wx,wy), self.origin, self.gridsize, self.resolution)
        if gpos is None:
            return
        
        # no obstacle in scan line
        if dist == float('inf') or dist == float('-inf'):
            occupied = False
            dist = maxrange

        # scan line end coordinate
        end_x = wx + dist * np.cos(angle)
        end_y = wy + dist * np.sin(angle)

        # end coordinate in grid
        endgrid = self.to_grid((end_x, end_y), self.origin, self.gridsize, self.resolution)
        if endgrid is None:
            return

        # get the points of the scan line
        points = self.get_line(gpos, endgrid)

        # Update all the points on the line (as not occupied)
        for i, pt in enumerate(points):
            self.update_cell(pt, False)

        # Update the end point of the line (as occupied)
        if occupied:
            self.update_cell(endgrid, True)

    
    # update the cell of the grid (1 : occupied, 0.05 : free)
    def update_cell(self, point, occupied):
        gx, gy = point
        
        if occupied:
            self.gridmap[self.gridsize[0] - gx, self.gridsize[1] - gy] = 1
        else:
            self.gridmap[self.gridsize[0] - gx, self.gridsize[1] - gy] = 0.05


    def to_grid(self, p, origin, size, resolution):
        (px,py) = p
        (origx, origy) = origin
        gx = int((px - origx)/resolution)
        gy = int((py - origy)/resolution)

        if gx >= size[0] or gy >= size[1]:
            return None

        return (gx,gy)


    def to_world(self, g, origin, size, resolution):
        (gx,gy) = g
        (origx, origy) = origin
        px = gx * resolution + origx + 0.5
        py = gy * resolution + origy + 0.5

        if px >= size[0] or py >= size[1]:
            return None

        return (px,py)

      
    # Convert grid coordinate to map index
    def to_index(self, gx, gy, size_x):
        return gy * size_x + gx


    def get_line(self, start, end):

        """Bresenham's Line Algorithm
        Produces a list of tuples from start and end
    
        >>> points1 = get_line((0, 0), (3, 4))
        >>> points2 = get_line((3, 4), (0, 0))
        >>> assert(set(points1) == set(points2))
        >>> print points1
        [(0, 0), (1, 1), (1, 2), (2, 3), (3, 4)]
        >>> print points2
        [(3, 4), (2, 3), (1, 2), (1, 1), (0, 0)]
        """

        # Setup initial conditions
        x1, y1 = start
        x2, y2 = end
        dx = x2 - x1
        dy = y2 - y1
    
        # Determine how steep the line is
        is_steep = abs(dy) > abs(dx)
    
        # Rotate line
        if is_steep:
            x1, y1 = y1, x1
            x2, y2 = y2, x2
    
        # Swap start and end points if necessary and store swap state
        swapped = False
        if x1 > x2:
            x1, x2 = x2, x1
            y1, y2 = y2, y1
            swapped = True
    
        # Recalculate differentials
        dx = x2 - x1
        dy = y2 - y1
    
        # Calculate error
        error = int(dx / 2.0)
        ystep = 1 if y1 < y2 else -1
    
        # Iterate over bounding box generating points between start and end
        y = y1
        points = []
        for x in range(x1, x2 + 1):
            coord = (y, x) if is_steep else (x, y)
            points.append(coord)
            error -= abs(dy)
            if error < 0:
                y += ystep
                error += dx
    
        # Reverse the list if the coordinates were swapped
        if swapped:
            points.reverse()
        return points


def main(): 
    rospy.init_node('occupancygrid')  
    # value chosen based on mymap.yaml
    occupancy = OccupancyGrid(origin=(-51, -51), size=(2048, 2048), resolution=0.1)
    rospy.spin()
 
if __name__ == '__main__':
    main()