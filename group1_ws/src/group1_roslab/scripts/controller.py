#!/usr/bin/env python
import rospy
from std_msgs.msg import String # import std message
from ackermann_msgs.msg import AckermannDriveStamped # import the custom message

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
		left = 0.4189
		forward = 0.5
	elif key == "right":
		left = -0.4189
		forward = 0.5
	else:
		left = 0
	
	drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = forward
        drive_msg.drive.steering_angle = left
	pub.publish(drive_msg)

if __name__ == "__main__":	
	rospy.init_node("controller",anonymous = True)
	pub = rospy.Publisher("vesc/low_level/ackermann_cmd_mux/input/teleop", AckermannDriveStamped, queue_size = 10)
	rospy.Subscriber("keyboardTalker",String, control_callback)
	# rospy.Subscriber("scan", LaserScan, scan_callback)
	rospy.spin()
