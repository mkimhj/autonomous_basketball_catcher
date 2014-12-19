#! /usr/bin/env python

import rospy
from std_msgs.msg import String
import tf
from tf.transformations import vector_norm

class Control:

	def __init__(self):
		rospy.init_node('control')
		publisher = rospy.Publisher('/catcher/control', String, latch=True, queue_size=1)
		self.tf = tf.TransformListener()
		self.state = 'waiting'
		while not rospy.is_shutdown():
			publisher.publish(self.state)
			getattr(self, 'state_' + self.state)()

	def state_waiting(self):
		raw_input()
		self.state = 'spinning'

	def state_spinning(self):
		raw_input()
		self.state = 'waiting'

if __name__ == '__main__':
	Control()