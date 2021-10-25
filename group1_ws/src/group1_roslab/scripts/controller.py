#!/usr/bin/env python
import rospy
from std_msgs.msg import String # import std message
from race.msg import drive_param # import the custom message

def control_callback(data):
	key = data.data
	print key	
	if key == "up":
		forward = 1
	elif key == "down":
		forward = -1
	else:
		forward = 0

	if key == "left":
		left = 0.5
		forward = 0.5
	elif key == "right":
		left = -0.5
		forward = 0.5
	else:
		left = 0

	msg = drive_param()	
	msg.velocity = forward
	msg.angle = left
	pub.publish(msg)

if __name__ == ’__main__’:	
	pub = rospy.Publisher('driverParameters', drive_param, queue_size = 10)
	rospy.Subscriber('keyboardTalker',String, control_callback)
	# rospy.Subscriber("scan", LaserScan, scan_callback)
	rospy.spin()
