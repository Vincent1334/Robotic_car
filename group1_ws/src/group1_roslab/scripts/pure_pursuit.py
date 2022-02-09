#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan

# import ROS msg types and libraries
import os
import csv
from ackermann_msgs.msg import AckermannDriveStamped
from tf.transformations import euler_from_quaternion
import numpy as np

L = 1
index_last_waypoint = 300

class PurePursuit(object):
    
    # Import waypoints.csv into a list (path_points)
    dirname = os.path.dirname(__file__)
    filename = os.path.join(dirname, '../waypoints/levine-waypoints.csv')
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
        global L
        global index_last_waypoint
        
        eulerAngles = euler_from_quaternion((pose_msg.pose.orientation.x,pose_msg.pose.orientation.y,pose_msg.pose.orientation.z,pose_msg.pose.orientation.w))
        
        x_pos = pose_msg.pose.position.x
        y_pos = pose_msg.pose.position.y
        
        nearest_index = index_last_waypoint
        nearest_dist = np.inf
        
        # Find the current waypoint to track using methods mentioned in lecture
        if index_last_waypoint >= len(self.path_points) - 2:
           print ('goal!')
        else:
            for i in range(index_last_waypoint,len(self.path_points) - 1):
            
                point_dist = np.sqrt((x_pos - self.path_points[i][0])**2 + (y_pos - self.path_points[i][1])**2)
                if point_dist > L and point_dist < nearest_dist:
                    nearest_index = i
                    nearest_dist = point_dist
                    break
        
            index_last_waypoint = nearest_index
            #print(abs((np.sqrt(x_pos**2 + y_pos**2) -np.sqrt(self.path_points[nearest_index][0]**2 + self.path_points[nearest_index][1]**2))))
            print (nearest_index)
        

        # transform goal point to vehicle frame of reference
        x = self.path_points[nearest_index][0] - x_pos
        y = self.path_points[nearest_index][1] - y_pos
        #x_dist = x*np.cos(eulerAngles[2])-y*np.sin(eulerAngles[2])   
        y_dist = y*np.cos(-eulerAngles[2])+x*np.sin(-eulerAngles[2]) 
        
        #print("-----")
        #print(x_dist)
        #print(y_dist)

        # calculate curvature/steering angle
        curvature = 2*y_dist/(L**2)
        #print(curvature)
        # publish drive message, don't forget to limit the steering angle between -0.4189 and 0.4189 radians
        if abs(curvature) > 0.4189:
            curvature = 0.4189 * np.sign(curvature)
        
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.drive.steering_angle = curvature
        drive_msg.drive.speed = 1
        self.drive_pub.publish(drive_msg)

def main():
    rospy.init_node('pure_pursuit_node')
    pp = PurePursuit()
    rospy.spin()
if __name__ == '__main__':
    main()
