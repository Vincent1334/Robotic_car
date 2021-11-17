#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

#PID CONTROL PARAMS
kp = 0.8
kd = 0.25
ki = 1
servo_offset = 0.0
prev_error = 0.0 
error = 0.0
integral = 0.0


#WALL FOLLOW PARAMS
ANGLE_RANGE = 270 # Hokuyo 10LX has 270 degrees scan
DESIRED_DISTANCE_RIGHT = 0.9 # meters
DESIRED_DISTANCE_LEFT = 0.55
VELOCITY = 0.3 # meters per second
CAR_LENGTH = 0.50 # Traxxas Rally is 20 inches or 0.5 meters

class WallFollow:
    """ Implement Wall Following on the car
    """
    def __init__(self):
        #Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/nav'

        self.lidar_sub = rospy.Subscriber('scan', LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher('vesc/high_level/ackermann_cmd_mux/input/nav_0', AckermannDrive, queue_size = 10)

    def getRange(self, data, angle):
        # data: single message from topic /scan
        # angle: between -45 to 225 degrees, where 0 degrees is directly to the right
        # Outputs length in meters to object with angle in lidar scan field of view
        #make sure to take care of nans etc.        
        if(angle < -45 or angle > 225):
            return 0.0
        return data.ranges[angle*4+45*4+data.angle_min*4]


    def pid_control(self, error, velocity):
        global integral
        global prev_error
        global kp
        global ki
        global kd        
        
        angle = kp * error + kd * (error-prev_error) #+ ki * np.(error)
        prev_error = error
        v = (1/(abs(angle)*20)) * velocity
        if v > velocity:
            v = velocity
        
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = -angle # - by simulation
        drive_msg.drive.speed = v
        self.drive_pub.publish(drive_msg)

    def followRight(self, data, rightDist):
        #Follow left wall as per the algorithm 
        L = 0.5
        theta = 50
        a = getRange(theta)
        b = getRange(0)
        alpha = np.arctan((a * np.cos(theta) - b)/(a * np.cos(theta) ))
        dist = b * np.cos(alpha)
        dist_future = dist + L * np.sin(alpha)
        error = dist_future - rightDist

        return error 
    
     def followLeft(self, data, leftDist):
        #Follow left wall as per the algorithm 
        L = 0.5
        theta = 180 - 50
        a = getRange(theta)
        b = getRange(180)
        alpha = np.arctan((a * np.cos(theta) - b)/(a * np.cos(theta) ))
        dist = b * np.cos(alpha)
        dist_future = dist + L * np.sin(alpha)
        error = dist_future - leftDist

        return error 

    def lidar_callback(self, data):
       
        errorRight = followRight(self, data , DESIRED_DISTANCE_RIGHT)
        errorLeft = followLeft(self, data , DESIRED_DISTANCE_LEFT)
        
        self.pid_control(errorRight, VELOCITY)
        #if errorRight > errorLeft:
        #    self.pid_control(errorLeft, VELOCITY)
        #else:
        #    self.pid_control(errorRight, VELOCITY)

def main(args):
    rospy.init_node("WallFollow_node", anonymous=True)
    wf = WallFollow()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)