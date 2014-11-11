#! /usr/bin/env python

import numpy as np
from scipy.optimize import curve_fit
import freenect
import cv2
import matplotlib.pyplot as plt

class Find:

	def __init__(self):
		self.samples = []

		cv2.namedWindow('controls', cv2.CV_WINDOW_AUTOSIZE)
		def nothing(one): pass
		cv2.createTrackbar('range_min', 'controls', 0, 255, nothing)
		cv2.createTrackbar('range_max', 'controls', 255, 255, nothing)
		while True:
			depth, timestamp = freenect.sync_get_depth()
			self.process_image(depth)

	def process_image(self, depth):
		depth >>= 2
		depth = depth.astype(np.uint8)
		range_min = cv2.getTrackbarPos('range_min', 'controls')
		range_max = cv2.getTrackbarPos('range_max', 'controls')
		obj = cv2.inRange(depth, range_min, range_max)
		cv2.imshow('depth', depth)
		cv2.imshow('obj', obj)
		cv2.waitKey(10)

	def adjust_trajectory(self, sample):
		pass

class Trajectory:

	def __init__(self, data_points):
		self.data_points = data_points

		# get x, y, z vectors
		self.x = self.data_points[:,0]
		self.y = self.data_points[:,1]
		self.z = self.data_points[:,2]

		self.compute_polynomials();

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

	def get_projected_coordinates(self, z):
		# returns x,y coordinates for input z based on calculated parabolas
		x_coord = self.z_x_poly(z)
		y_coord = self.z_y_poly(z)

		return [x_coord, y_coord]

if __name__ == '__main__':
	Find()
	sample_data_points = np.array([(1, 1, 1), (2, 2, 2), (3, 3, 3), (4.5, 4, 4), (5, 5, 4.5), (6, 6, 4), (7, 7, 3), (8, 8, 2), (9, 9, 1)])
	trajectory = Trajectory(sample_data_points)
	print(trajectory.get_projected_coordinates(2.3))
	trajectory.show_polynomials()
