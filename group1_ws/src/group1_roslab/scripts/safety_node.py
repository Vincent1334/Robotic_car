#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDrive
import numpy as np


class Safety(object):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        
        """
        One publisher should publish to the /brake topic with a AckermannDriveStamped brake message.

        One publisher should publish to the /brake_bool topic with a Bool message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /odom topic to get the current speed of the vehicle.
        

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """
        
        self.speed = 0
        # ROS subscribers and publishers.
        pub_ackerman= rospy.Publisher('ackermanDrive', AckermannDrive, queue_size = 10)
        rospy.Subscriber('odom', Odometry, odom_callback)
        rospy.Subscriber('scan', LaserScan, scan_callback)

    def odom_callback(self, odom_msg):
        # update current speed
        linear = odom_msg.twist.linear
        self.speed = np.sqrt(np.pow(linear.x)+np.pow(linear.y))

    def scan_callback(self, scan_msg):
        # TODO: calculate TTC
        minTTC = np.inf
        angle = scan_msg.angle_min
        for value in scan_msg.ranges:  
            TTC = value / (self.speed * np.cos(angle)) 
            angle += scan_msg.angle_increment
            if minTTC > TTC:
                minTTC = TTC
        if minTTC > :
            return
        # publish brake message and publish controller bool
        break_msg =  AckermannDrive()
        break_msg.speed = 0
        pub_ackerman.publish(break_msg)


def main():
    rospy.init_node('safety_node')
    sn = Safety()
    rospy.spin()
if __name__ == '__main__':
    main()