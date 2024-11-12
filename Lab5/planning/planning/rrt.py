"""
This file contains the class definition for tree nodes and RRT
Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf
"""
import numpy as np
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, PoseArray
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry, OccupancyGrid
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from visualization_msgs.msg import Marker

from copy import deepcopy

# class def for RRT tree nodes
class RRTNode(object):
    def __init__(self, x=0.0, y=0.0, parent=None, is_root=False):
        # X and Y positions should be in meters relative to the position of the lidar
        self.x = x
        self.y = y
        self.parent = parent
        self.is_root = is_root

# class def for RRT
class RRT():
    def __init__(self, max_grid_distance, grid_resolution, inflation_radius, max_rrt_iters, max_rrt_expansion_distance, search_radius, goal_tolerance):

        # Occupancy Grid
        self.max_grid_distance = max_grid_distance # Length of the side of the occupancy grid in meters
        self.grid_resolution = grid_resolution # Size length of a single square grid cell in meters
        self.inflation_radius = inflation_radius

        self.grid_length = int(self.max_grid_distance / self.grid_resolution) # Length of the side of the occupancy grid in cells/pixels
        self.grid = np.zeros((self.grid_length, self.grid_length), np.int8) # 0 is not occupied, 1 is occupied


        # RRT
        self.tree = []
        self.max_rrt_iters = max_rrt_iters
        self.max_rrt_expansion_distance = max_rrt_expansion_distance
        self.search_radius = search_radius
        self.goal_tolerance = goal_tolerance


    def update_occupancy_grid(self, msg):
        """
        LaserScan callback, you should update your occupancy grid here

        Args:
            msg (LaserScan): incoming message from the lidar
        Returns:

        """
    
        # Note: Your occupancy grid should be be square matrix of size max_grid_distance by max_grid_distance.
        # The first dimension is x and the second is y: grid[x][y]. 
        # 0,0 should be in the bottom right with increasing x being forward and increasing y being left.
        # It should be centered with the car/lidar in the y axis (horizontal) and have the x-axis start at the position of the lidar and move forward

        # Don't forget to to increase the size of each point as you add it to the occupancy grid by the inflation_radius.
        # If your inflation_radius is 1 then a 1x1 obstacle should become a 3x3 obstacle in the grid

        # You will most likely need to convert the polar coordinates of each range point first into cartesian coordinates in meters
        # then into the grid based coordinates used for the occupancy grid. Since the grid does not extend backwards from the lidar you
        # should be able to only check theta values between -pi/2 and pi/2

        self.grid = np.zeros((self.grid_length, self.grid_length), np.int8) # Clear the occupancy grid

        # # TODO: Put your occupancy grid code here
        # Get ranges and angles from scan message
        ranges = np.array(msg.ranges)
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        
        #NOTE: Matthew and Aryan, not sure if this is correct
        # Get a list of all indices of angles that are between -pi/2 and pi/2
        angle_indices = [i for i, a in enumerate(angles) if a > np.pi/2 and a < -np.pi/2]
        angle_indices += [i for i, r in enumerate(ranges) if r == 0.0]
        ranges = np.delete(ranges, angle_indices)
        angles = np.delete(angles, angle_indices)
        #(np.min(ranges))

        # angle_indices = [i for i, r in enumerate(ranges) if r != 0.0]
        # ranges = ranges[angle_indices]
        # angles = angles[angle_indices]
        # for i in angle_indices:
        #     if ranges(i) == 0.0:
                
        # Remove the indices from the ranges and angles
        
        

        # Get a list of x, y coordinates from the angles and ranges, shape: (# coordinates, 2)
        occ_spaces = np.vstack([[r * np.cos(a), r * np.sin(a)] for r, a in zip(ranges, angles)])

        # Shift occ_spaces right by self.grid_max_distance/2 to put (0, 0) in bottom right
        occ_spaces[:, 1] += self.max_grid_distance/2

        # Convert occ_spaces to grid indices
        occ_indices = np.array(np.array(occ_spaces) / self.grid_resolution, dtype=int)

        # Check that points are in bounds
        occ_indices_in_bounds = []
        for point in occ_indices:
            x, y = point
            if x < self.grid_length and y < self.grid_length:
                occ_indices_in_bounds.append([x, y])
        occ_indices_in_bounds = np.vstack(np.array(occ_indices_in_bounds))

        # Inflate points
        for point in occ_indices_in_bounds:
            x_point, y_point = point
            for x in range(-self.inflation_radius, self.inflation_radius + 1):
                for y in range(-self.inflation_radius, self.inflation_radius + 1):
                    x_inflate = x_point + x
                    y_inflate = y_point + y
                    if x_inflate >= 0 and x_inflate < self.grid_length and y_inflate >= 0 and y_inflate < self.grid_length:
                        if x_inflate <= 4 and abs(y_inflate-self.grid_length/2) <= 3:
                            continue
                        self.grid[x_inflate, y_inflate] = 1

        # self.grid[occ_indices_in_bounds[:, 0], occ_indices_in_bounds[:, 1]] = 1

    def plan_path(self, target_point):
        """
        This method should run the RRT path planner and return a path

        Args:
            target_point [x, y] (float float): a tuple/list representing the target point in meters
        Returns:
            [RRTNode]: a list of nodes representing the path (the first node in the list should be the node closest to the goal point

        """

        # Clear the tree
        self.tree = [RRTNode(0.0, 0.0, None, True)]

        # TODO: Put your RRT code here
        for i in range(self.max_rrt_iters):
            # Sample a point
            sampled_point = self.sample()
            if self.check_point_collision(sampled_point):
                continue

            # Find the nearest node to the sampled point
            nearest_node = self.nearest(sampled_point)
            # Steer towards the sampled point
            new_node = self.steer(self.tree[nearest_node], sampled_point)
            # Check if the path between the nearest node and the new node is collision free
            if self.check_line_collision(self.tree[nearest_node], new_node) == False:
                # Add the new node to the tree
                self.tree.append(new_node)

            # Check if goal is reached
            if self.is_goal(new_node, target_point):
                # Find the path
                path = self.find_path(new_node)
                return path
            
        # if max iterations is reached, return current path
        return []

    def sample(self):
        """
        This method should randomly sample the free space of the occupancy grid and return a viable point

        Args:
        Returns:
            (x, y) (float float): a tuple representing the sampled point in meters

        """
        # Note: Don't forget that the X coordinate is from 0 to max_grid_distance 
        #       while the Y coordinate ranges from -max_grid_distance/2 to max_grid_distance/2

        #TODO: Check if the sampled point is in an obstacle, if it is then sample again
        x = np.random.uniform(0, self.max_grid_distance)
        y = np.random.uniform(-self.max_grid_distance/2, self.max_grid_distance/2)

        return (x, y)

    def nearest(self, sampled_point):
        """
        This method should return the nearest node on the tree to the sampled point

        Args:
            tree ([]): the current RRT tree
            sampled_point (tuple of (float, float)): point sampled in free space in meters
        Returns:
            nearest_node (int): index of neareset node on the tree
        """

        # TODO: put your code here

        # Create a 2-D array of x, y coordinates of the tree nodes
        tree_points = np.vstack([[node.x, node.y] for node in self.tree]) # shape: (num_nodes, 2)

        point_distances = tree_points - np.array(sampled_point)

        # Find the distance between the sampled point and all the tree nodes using the L2 norm
        distances = np.linalg.norm(point_distances, axis=1)

        # Find the index of the node with the minimum distance
        nearest_node = np.argmin(distances)

        return nearest_node

    def steer(self, nearest_node, sampled_point):
        """
        This method should return a point in the viable set such that it is closer 
        to the nearest_node than sampled_point is.

        Args:
            nearest_node (Node): nearest node on the tree to the sampled point
            sampled_point (tuple of (float, float)): sampled point in meters
        Returns:
            new_node (Node): new node created from steering
        """

        # TODO: put your code here
        # Find unit vector from nearest node to sampled point
    
        x_dist = sampled_point[0] - nearest_node.x
        y_dist = sampled_point[1] - nearest_node.y
        dist = np.linalg.norm(np.array([x_dist, y_dist]))

        if(dist < self.max_rrt_expansion_distance):
            return RRTNode(sampled_point[0], sampled_point[1], nearest_node)

        unit_vector = np.array([x_dist, y_dist]) / dist

        # Find the new node by moving a distance of max_rrt_expansion_distance along the unit vector
        new_node = RRTNode(
            nearest_node.x + self.max_rrt_expansion_distance * unit_vector[0],
            nearest_node.y + self.max_rrt_expansion_distance * unit_vector[1],
            nearest_node
        )
        
        return new_node


    def check_point_collision(self, point):
        """
        This method should return whether the node is inside an obstacle

        Args:
            point (double x, double y): point to check in meters
        Returns:
            collision (bool): True if the point is in an obstacle in the occupancy grid
        """

        # Hint: In python the bool() constructor will convert non-zero integers to True

        # TODO: put your code here
        # NOTE: Updated 
        # Find the corresponding indices of the point in the boolean grid
        x_shift, y_shift = point
        y_shift += self.max_grid_distance/2
        x, y = np.array(np.array([x_shift, y_shift]) / self.grid_resolution, dtype=int)

        return bool(self.grid[x, y])

    def check_line_collision(self, nearest_node, new_node):
        """
        This method should return whether the path between nearest and new_node is
        collision free.

        Args:
            nearest (Node): nearest node on the tree
            new_node (Node): new node from steering
        Returns:
            collision (bool): True if the path between the two nodes is in collision
                              with the occupancy grid
        """

        # Note: One method to check for collision is to check a number of equally spaced
        # points along the line. You can use self.grid_resolution to figure out 
        # how many points you need to check to ensure you will detect the collision

        # TODO: put your code here
        # NOTE: not tested
        x_dist = new_node.x - nearest_node.x
        y_dist = new_node.y - nearest_node.y
        sample_resolution = math.ceil(max(abs(x_dist), abs(y_dist))/self.grid_resolution)

        for i in range(sample_resolution):
            check_point = ((nearest_node.x + i*x_dist/sample_resolution),(nearest_node.y + i*y_dist/sample_resolution))
            if self.check_point_collision(check_point):
                return True
            else:
                continue
        return False

    def is_goal(self, latest_added_node, goal_point):
        """
        This method should return whether the latest added node is close enough
        to the goal.

        Args:
            latest_added_node (Node): latest added node on the tree
            goal_point [double x, double y]: x,y coordinates of the current goal in meters
        Returns:
            close_enough (bool): true if node is close enough to the goal
        """

        # TODO: put your code here
        if np.linalg.norm(np.array([latest_added_node.x, latest_added_node.y]) - np.array(goal_point)) < self.goal_tolerance:
            return True
        else:
            return False

    def find_path(self, latest_added_node):
        """
        This method returns a path as a list of Nodes connecting the starting point to
        the goal once the latest added node is close enough to the goal

        Args:
            tree ([]): current tree as a list of Nodes
            latest_added_node (Node): latest added node in the tree
        Returns:
            path ([]): valid path as a list of Nodes. The first node in the list should be the latest_added_node
                       and the nodes should move backwards to the starting point
        """
        path = []
        not_start = True

        # TODO: put your code here
        while not_start:
            # Append node to path
            path.append(latest_added_node)
            # Set last added node to the parent node
            latest_added_node = latest_added_node.parent
            # Check if the node is the root node, if not then repeat
            if latest_added_node.is_root:
                not_start = False

        return path