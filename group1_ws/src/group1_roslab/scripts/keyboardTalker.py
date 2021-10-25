#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import curses

stdscr = curses.initscr()
curses.cbreak()
stdscr.keypad(1)
stdscr.refresh()

def main():
	key = ’’
	while key != ord(’q’):
		key = stdscr.getch()
		stdscr.refresh()
		print key

	if key == curses.KEY_UP: msg = "up"
	elif key == curses.KEY_DOWN: msg = "down"
	elif key == curses.KEY_LEFT: msg = "left"
	elif key == curses.KEY_RIGHT: msg = "right"
	else:msg = ""

	pub.publish(msg)
	curses.endwin()

# Boilerplate code to start this ROS node.
if __name__ == ’__main__’:
	rospy.init_node(’keyboardTalker’,anonymous = True)
	pub = rospy.Publisher(’direction’, String, queue_size = 10)
	main()
	rospy.spin()
