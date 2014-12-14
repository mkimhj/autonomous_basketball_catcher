#! /usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import Point
from math import atan2, sqrt, pi

class Navigation:

	def __init__(self):
		rospy.init_node('navigation')
		listener = tf.TransformListener()
		publisher = rospy.Publisher('/catcher/robot', Point, queue_size=1)
		while not rospy.is_shutdown():
			now = rospy.Time.now()
			try:
				listener.waitForTransform('goal', 'robot', now, rospy.Duration(2.0))
				position, orientation = listener.lookupTransform('goal', 'robot', now)
				publisher.publish(*position)
			except tf.Exception as e:
				print e

if __name__ == '__main__':
	Navigation()