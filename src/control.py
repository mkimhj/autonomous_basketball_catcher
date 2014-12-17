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
		self.state = 'sampling'

	def state_sampling(self):
		raw_input()
		self.state = 'driving'

	def state_driving(self):
		try:
			now = rospy.Time.now()
			self.tf.waitForTransform('goal', 'robot', now, rospy.Duration(2.0))
			position, orientation = listener.lookupTransform('goal', 'robot', now)
			distance = vector_norm(position)
			if distance < 0.1:
				self.state = 'waiting'
		except tf.Exception:
			pass

if __name__ == '__main__':
	Control()