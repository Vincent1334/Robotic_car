#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
import numpy as np
    
pub_max = rospy.Publisher('farthest_point', Float64, queue_size = 10)
pub_min = rospy.Publisher('closest_point', Float64, queue_size = 10)

    
def listener():      
    rospy.init_node('lidar', anonymous = True)
    rospy.Subscriber('scan', LaserScan, get_values)
    rospy.spin()
        
def get_values(data):
    
    max_v = 0
    min_v = np.inf
    
    for d in data.ranges:
        if np.isnan(d) or np.isinf(d):
            continue
        if d > max_v:
            max_v = d
        elif d < min_v:
            min_v = d
                
    print("maxvalue:", max_v)
    print("minvalue:", min_v)
    
    pub_max.publish(max_v)
    pub_min.publish(min_v)
        
if __name__ == '__main__':
    try:
        listener().run()
    except rospy.ROSInterruptException:
        pass
    