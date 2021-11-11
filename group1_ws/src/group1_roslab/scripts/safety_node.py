#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np

pub_ackerman= rospy.Publisher('vesc/low_level/ackermann_cmd_mux/input/brake', AckermannDriveStamped, queue_size = 10)


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
        
        rospy.Subscriber('vesc/odom', Odometry, self.odom_callback)
        rospy.Subscriber('scan', LaserScan, self.scan_callback)

    def odom_callback(self, odom_msg):
        # update current speed
        self.speed = odom_msg.twist.twist.linear.x
        #self.speed = np.sqrt(odom_msg.twist.covariance.linear.x ** 2 + odom_msg.twist.covariance.linear.y ** 2)        

    def scan_callback(self, scan_msg):        
        if self.speed < 0:
            return
        TTCs= np.ones(1081)-1000
        #leftside
        for i in range(540,int(scan_msg.angle_max)*4-400):
            value = scan_msg.ranges[i]
            if np.isnan(value) or np.isinf(value) or self.speed == 0:
                continue
            TTC = -(value / (self.speed * abs(np.cos(np.deg2rad(i/4)))))
            TTCs[i] = TTC
                
        #rightside
        for i in range(int(scan_msg.angle_min)*4+400,540):
            value = scan_msg.ranges[i]
            if np.isnan(value) or np.isinf(value) or self.speed == 0:
                continue
            TTC = -(value / (self.speed * abs(np.cos(np.deg2rad(i/4)))))
            TTCs[i] = TTC

        if np.ndarray.max(TTCs)  > -2:        
            # publish brake message and publish controller bool            
            break_msg =  AckermannDriveStamped()
            break_msg.drive.speed = 0
            pub_ackerman.publish(break_msg)
            
        print("maxTTC", np.ndarray.max(TTCs))
       


def main():
    
    rospy.init_node('safety_node')
    sn = Safety()
    rospy.spin()
if __name__ == '__main__':
    main()