#!/usr/bin/env python
import numpy as np
from numpy import linalg as LA
import math
import random
import rospy
import os
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
# from tf import transform_listener
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

# TODO: import as you need
ITERATIONS = 5000
GOAL_RADIUS = 1.5
STEER_DIST = 0.5
CAR_SIZE = 0.4


# class def for tree nodes
class Node(object):
    def __init__(self, x=0, y=0, parent=None, is_root=False):
        self.x = x  # X index in OccupancyGrid
        self.y = y  # Y index in OccupancyGrid
        self.parent = parent  # parent node
        self.is_root = is_root  # is this the start node


# class def for RRT
class RRT(object):

    def __init__(self):
        # subscibers
        rospy.Subscriber("/pf/viz/inferred_pose", PoseStamped, self.pf_callback)
        rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goalpose_callback)
        # publishers
        topic = 'visualization_marker_array'
        self.markerPublisher = rospy.Publisher(topic, MarkerArray, queue_size="1")
        self.pathPublisher = rospy.Publisher("rrt_path", Path, queue_size="1")
        # class attributes
        self.grid = np.zeros((2, 2))  # OccupancyGrid
        self.map_height = 0
        self.map_width = 0
        self.map_resolution = 1
        self.map_origin_x = 0
        self.map_origin_y = 0
        self.map_origin_rot = 0
        self.got_map = False
        self.tree = []  # list of nodes
        self.x_pos = 0.0  # the cars x position
        self.y_pos = 0.0  # the cars y position
        self.goal = [0.0, 0.0]  # goal position
        self.markerArr = MarkerArray()

    def pf_callback(self, pose_msg):

        # The pose callback when subscribed to particle filter’s inferred pose
        # Here is where you save the cars x and y position for the rrt
        # Args:
        # pose_msg (PoseStamped): incoming message from subscribed topic
        # Returns:
        self.x_pos = pose_msg.position.x
        self.y_pos = pose_msg.position.y

        return None

    def conv_to_gridposition(self, x, y):
        # Convertes the x and y coordiates into grid positions
        # Args:
        # x (float): x coordiantes
        # y (float): y coordiantes
        # Returns:
        # (idx_x , idx_y) (int int): tuple with the x and y position on the grid
        idx_x = x + self.map_origin_x
        idx_y = y + self.map_origin_y
        return idx_x, idx_y

    def conv_to_free_space(self, idx_x, idx_y):
        """
        Convertes the x and y grid positions into world coordinates
        Args:
        idx_x (int): x grid position
        idx_x (int): y grid position
        Returns:
        world_x , world_y) (float float): tuple with the x and y position in world
        coordinates
        """
        world_x = idx_x - self.map_origin_x
        world_y = idx_y - self.map_origin_y
        return world_x, world_y

    def map_callback(self, map_msg):
        """
        Saves the data published by the map_server
        Args:
        map_msg (OccupancyGrid): incoming message from subscribed topic
        Returns:
        """
        self.map_height = map_msg.info.height
        self.map_width = map_msg.info.width
        self.grid = np.zeros((self.map_width, self.map_height))  # OccupancyGrid
        a = 0
        for i in self.grid:
            i[0] = map_msg.data[a]
            a += 1
            i[1] = map_msg.data[a]
            a += 1

        self.map_resolution = map_msg.info.resolution
        self.map_origin_x = map_msg.info.pose.position.x
        self.map_origin_y = map_msg.info.pose.position.y

        return None

    def goalpose_callback(self, goal_msg):
        """
        Set the goalposition on the grid.
        Uses rrt to calculate the waypoints in the occupancy grid.
        Saves the Path.
        Args:
        goal_msg (PoseStamped): incoming message from subscribed topic
        Returns:
        """
        self.goal = [goal_msg.pose.position.x, goal_msg.pose.position.y]

        # function to test your point conversions
        goal_idx = self.conv_to_gridposition(self.goal[0], self.goal[1])
        print("Goal_idx", goal_idx)
        freeSpacePosition = self.conv_to_free_space(goal_idx[0], goal_idx[1])
        self.visualizeMarkers((freeSpacePosition[0], freeSpacePosition[1]))  # create a waypoint in Rviz

        # if this node gets a map
        if self.map_width != 0 and self.map_height != 0:
            # do RRT
            start_pos = [self.map_origin_x, self.map_origin_y]
            goal_region = [goal_idx[0], goal_idx[1]], GOAL_RADIUS
            step_dist = STEER_DIST
            node_0 = Node(start_pos[0], start_pos[1], None, True)  # [x_pos, y_pos, parent_node, is_root]
            self.tree.append(node_0)  # add first node to node tree
            for j in range(0, ITERATIONS):
                randPoint = self.sample()  # get random Point in occupancy grid
                if not self.inside_obstacle(randPoint):
                    k_nearest = self.nearest(self.tree, randPoint)  # find nearest node to the sampled point
                    k_new = self.steer(k_nearest,
                                       randPoint)  # return a node between k_nearest and the random point that is a
                    # STEER_DIST away from the nearest_node. If collision with obstacle return None.
                    if k_new is not None:
                        self.tree.append(k_new)
                        if self.is_goal(k_new, goal_idx[0], goal_idx[1]):
                            path = self.find_path(k_new)  # return list of nodes from start node to goal node
                            print(path)
                            break

                            # and publish the path
        self.publish_path(path)

    def inside_obstacle(self, randPoint):
        return self.grid[randPoint[0], randPoint[1]] == 1

    def sample(self):
        """
        This method should randomly sample the grid space, and returns a viable point
        Args:
        Returns:
        (x, y) (float float): a tuple representing the sampled point on the grid
        """
        x = np.random.random_integers(0, self.map_width - 1)
        y = np.random.random_integers(0, self.map_height - 1)
        xy = (x, y)
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
        nearest_node = None
        nearest_dist = np.sqrt(sampled_point.x ** 2 + sampled_point.y ** 2)
        for node in tree:
            if np.sqrt(node.x ** 2 + node.y ** 2) < nearest_dist:
                nearest_dist = np.sqrt(node.x ** 2 + node.y ** 2)
                nearest_node = node

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
        x = sampled_point[0] - nearest_node.x
        y = sampled_point[1] - nearest_node.y
        d = np.sqrt(x ** 2 + y ** 2)
        x = STEER_DIST * x / d
        y = STEER_DIST * y / d
        new_node = (x, y, nearest_node, False)

        if self.check_collision(nearest_node, new_node):
            return None

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
        x = nearest_node.x - new_node.x
        y = nearest_node.y - new_node.y
        d = dist([nearest_node.x, nearest_node.y], [new_node.x, new_node.y])
        inc_x = x / d
        inc_y = y / d
        collision = False

        for x_i in range(nearest_node.x, new_node.x, inc_x):
            for y_i in range(nearest_node.y, new_node.y, inc_y):
                if self.grid[np.round(x_i, 0), np.round(y_i, 0)]:
                    collision = True
                    return collision
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
        return dist([latest_added_node.x, latest_added_node.y], [goal_x, goal_y]) <= GOAL_RADIUS

    def find_path(self, latest_added_node):
        """
        This method returns a path as a list of Nodes connecting the starting point to
        the goal once the latest added node is close enough to the goal
        Args:
        latest_added_node (Node): latest added node in the tree
        Returns:
        path ([]): valid path as a list of Nodes
        """
        path = []
        tmp_node = latest_added_node
        while not tmp_node.is_root:
            path.append(tmp_node)
            tmp_node = tmp_node.parent
        path.reverse()
        return path

    def publish_path(self, path):
        """
        This method publishes a Path msg with the path positions to the goal
        Args:
        path ([]): node path to the goal
        Returns:
        """
        msg = Path()
        # add node positions from path to the msg
        for node in path:
            pose = PoseStamped()
            pose.pose.position.x = self.conv_to_free_space(node.position.x, node.position.y)
            msg.poses.append(pose)

            # publish the path
        self.pathPublisher.publish(msg)

    def visualizeMarkers(self, points):
        """
        visualize the nodes as markers
        """
        markerArray = self.markerArr
        x = float(points[0])
        y = float(points[1])
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0
        markerArray.markers.append(marker)
        # Renumber the marker IDs
        id = 0
        for m in markerArray.markers:
            m.id = id
            id += 1
        # Publish the MarkerArray
        self.markerPublisher.publish(markerArray)


# Computes the Euclidean distance between two 2D points p1 and p2.
def dist(p1, p2):
    return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def main():
    rospy.init_node('rrt')
    rrt = RRT()
    rospy.spin()


if __name__ == '__main__':
    main()
