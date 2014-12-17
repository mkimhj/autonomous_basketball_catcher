#! /usr/bin/env python

import rospy
from std_msgs.msg import String
import tf
import numpy as np
import warnings
warnings.filterwarnings('ignore')

class Trajectory:

	def __init__(self):
		rospy.init_node('trajectory')
		self.control_state = 'waiting'
		rospy.Subscriber('/catcher/control', String, self.received_control)
		listener = tf.TransformListener()
		broadcaster = tf.TransformBroadcaster()
		while not rospy.is_shutdown():
			now = rospy.Time.now()
			if self.control_state == 'waiting':
				self.x = []
				self.y = []
				self.z = []
			if self.control_state == 'sampling':
				try:
					listener.waitForTransform('trash', 'robot', now, rospy.Duration(0.2))
					position, orientation = listener.lookupTransform('trash', 'robot', now)
					self.x.append(position[0])
					self.y.append(position[1])
					self.z.append(position[2])
					print 'position =', position
				except tf.Exception as e:
					pass
			if self.control_state == 'driving':
				if len(self.x) >= 3:
					x, y = self.get_projected_coordinates_from_line()
					print 'x, y =', x, y
					if x and y:
						broadcaster.sendTransform((x, y, 0), (0, 0, 0, 1), now, 'goal', 'robot')

	def received_control(self, msg):
		self.control_state = msg.data

	def compute_polynomials(self):
		# calculate polynomials
		x_z = np.polyfit(self.x, self.z, 2)
		y_z = np.polyfit(self.y, self.z, 2)

		self.x_z_poly = np.poly1d(x_z)
		self.y_z_poly = np.poly1d(y_z)

		z_x = np.polyfit(self.z, self.x, 2)
		z_y = np.polyfit(self.z, self.y, 2)

		self.z_x_poly = np.poly1d(x_z)
		self.z_y_poly = np.poly1d(y_z)

	def show_polynomials(self):
		x_new = np.linspace(self.x[0], self.x[-1], 50)
		y_new = np.linspace(self.y[0], self.y[-1], 50)

		z_x_new = self.x_z_poly(x_new)
		z_y_new = self.y_z_poly(y_new)

		f, (ax1, ax2) = plt.subplots(1, 2, sharey=True)

		ax1.set_title('X-Z Curve')
		ax1.plot(self.x,self.z,'o', x_new, z_x_new)

		ax2.set_title('Y-Z Curve')
		ax2.plot(self.y,self.z,'o', y_new, z_y_new)

		plt.xlim([self.x[0]-1, self.x[-1] + 1 ])
		plt.show()

	def get_projected_coordinates(self):
		self.compute_polynomials()
		# returns projected x,y coordinates based on calculated parabolas

		print(self.z_x_poly.r)
		print(self.z_y_poly.r)

		x_coord = self.z_x_poly.r[0]
		y_coord = self.z_y_poly.r[0]

		if np.iscomplex(x_coord) or np.iscomplex(y_coord):
			return (None, None)

		return (x_coord, y_coord)

	def compute_line(self):
		x_y = np.polyfit(self.x, self.y, 1)
		self.x_y_line = np.poly1d(x_y)

	def get_projected_coordinates_from_line(self):
		self.compute_line()

		#determine direction
		begin_coord = (self.x[0], self.y[0])
		final_coord = (self.x[len(self.x) - 1], self.y[len(self.y) -1])
		direction = begin_coord[0] - final_coord[0]

		x_coord = begin_coord[0]

		if direction < 0:
			x_coord += 1
		else:
			x_coord += -1

		y_coord = self.x_y_line(x_coord)

		return [x_coord, y_coord]

if __name__ == '__main__':
	Trajectory()