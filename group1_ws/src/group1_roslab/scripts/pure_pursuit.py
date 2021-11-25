#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan

# TODO: import ROS msg types and libraries
from ackermann_msgs.msg import AckermannDriveStamped
from tf.transformations import euler_from_quaternion
import numpy as np

class PurePursuit(object):
    global L = 0.2
    global index_last_waypoint = 0
    # Import waypoints.csv into a list (path_points)
    dirname = os.path.dirname(__file__)
    filename = os.path.join(dirname, ’../waypoints/levine-waypoints.csv’)
    with open(filename) as f:
        path_points = [tuple(line) for line in csv.reader(f)]
    # Turn path_points into a list of floats to eliminate the need for casts in the code below.
    path_points = [(float(point[0]), float(point[1]), float(point[2])) for point in path_points]
    """
    The class that handles pure pursuit.
    """
    def __init__(self):
        #Topic for Simulation
        self.pose_sub = rospy.Subscriber('/pf/viz/inferred_pose', PoseStamped, self.pose_callback)
        #Topic for real world
        #self.pose_sub = rospy.Subscriber('/pf/inferred_pose', PoseStamped, self.pose_callback)
        self.drive_pub = rospy.Publisher('vesc/high_level/ackermann_cmd_mux/input/nav_0', AckermannDriveStamped, queue_size = 10)

    def pose_callback(self, pose_msg):
        eulerAngles = euler_from_quaternion(pose_msg.pose.orientation)
    
        # TODO: find the current waypoint to track using methods mentioned in lecture
        for i in range(index_last_waypoint,path_points.size - 1):
            nearest = np.inf
            nearest_dist = np.inf
            point_dist = abs(# Abstand car pos zu Ursprung -np.sqrt(path_points[i][0]**2 + path_points[i][1]**2))
            if point_dist > L and point_dist < nearest_dist:
                nearest = path_points[i]
                nearest_dist = point_dist
        

        


        # TODO: transform goal point to vehicle frame of reference

        # TODO: calculate curvature/steering angle

        # TODO: publish drive message, don't forget to limit the steering angle between -0.4189 and 0.4189 radians


def main():
    rospy.init_node('pure_pursuit_node')
    pp = PurePursuit()
    rospy.spin()
if __name__ == '__main__':
    main()
