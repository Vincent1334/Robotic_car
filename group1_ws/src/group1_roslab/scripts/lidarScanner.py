#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from group1_roslab.msg import scan_range
from std_msgs.msg import Float64
import numpy as np

pub_scanRange = rospy.Publisher('scanRange', scan_range, queue_size = 10)
#pub_max = rospy.Publisher('farthest_point', Float64, queue_size = 10)
#pub_min = rospy.Publisher('closest_point', Float64, queue_size = 10)

    
def listener():      
    rospy.init_node('lidarScanner', anonymous = True)
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
	
	msg = scan_range()
	msg.max_v = max_v
	msg.min_v = min_v
	msg.header.stamp = rospy.Time.now();

	pub_scanRange.pub(msg)
	

    
    #pub_max.publish(max_v)
    #pub_min.publish(min_v)

        
if __name__ == '__main__':
    try:
        listener().run()
    except rospy.ROSInterruptException:
        pass
    
