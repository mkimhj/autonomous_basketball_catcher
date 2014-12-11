#! /usr/bin/env python

import numpy as np
from scipy.optimize import curve_fit
#import freenect
#import cv2
import matplotlib.pyplot as plt
from math import pi, atan

DEGREES_TO_RADIANS = pi / 180
K_VERTICAL_FOV = 43 * DEGREES_TO_RADIANS
K_HORIZONTAL_FOV = 57 * DEGREES_TO_RADIANS
K_HALF_WIDTH = 640 / 2
K_HALF_HEIGHT = 480 / 2
K_VERTICAL_DEPTH = K_HALF_HEIGHT / atan(K_VERTICAL_FOV / 2)
K_HORIZONTAL_DEPTH = K_HALF_WIDTH / atan(K_HORIZONTAL_FOV / 2)
K_MAX_DEPTH_MM = 10000

DEBUG = True

class Find:

	def __init__(self):
		self.samples = []

		cv2.namedWindow('depth', cv2.CV_WINDOW_AUTOSIZE)
		cv2.namedWindow('processed', cv2.CV_WINDOW_AUTOSIZE)
		self.last_depth = None
		cv2.setMouseCallback('depth', self.on_mouse_click)
		while True:
			depth, timestamp = freenect.sync_get_depth(format=freenect.DEPTH_MM)
			self.last_depth = depth
			self.process_image(depth)

	def on_mouse_click(self, event, x, y, flag, param):
		if(event == cv2.EVENT_LBUTTONUP):
			z = int(self.last_depth[y, x])
			print 'x: %d y: %d z: %d'%self.project(x, y, z)

	def process_image(self, depth):
		depth = depth.astype(float) / K_MAX_DEPTH_MM
		depth[depth == 0] = 1
		ranged = cv2.inRange(depth, 0, 0.3)
		contours, _ = cv2.findContours(ranged.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
		processed = np.zeros((480, 640, 3), np.uint8)
		processed[:, :, 0] = ranged
		obj = None
		for contour in contours:
			area = cv2.contourArea(contour)
			if 100 < area < 1500:
				obj = contour
				rect = cv2.boundingRect(obj)
				x, y, w, h = rect
				x += w / 2
				y += h / 2
				self.samples.append((x, y))
				break
		cv2.drawContours(processed, np.array([obj]), -1, (0, 255, 0), 3)
		for sample in self.samples:
			cv2.line(processed, sample, sample, (0, 0, 255), 3)
		cv2.imshow('depth', depth)
		cv2.imshow('processed', processed)
		if cv2.waitKey(10) == 13:
			self.samples = []

	def project(self, u, v, z):
		u -= K_HALF_WIDTH
		v -= K_HALF_HEIGHT
		x = (u / K_HORIZONTAL_DEPTH) * z
		y = (v / K_VERTICAL_DEPTH) * z
		return x, -y, -z

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

	def get_projected_coordinates(self):
		# returns projected x,y coordinates based on calculated parabolas

		log(self.z_x_poly.r)
		log(self.z_y_poly.r)

		x_coord = self.z_x_poly.r[0]
		y_coord = self.z_y_poly.r[0]

		return [x_coord, y_coord]

class Navigation:

	def __init__(self, current_coord, current_angle, projected_coord):
		# TODO: returns current x, y coordinates and angle of hercules robot trash can
		self.current_coord = current_coord
		self.current_angle = current_angle
		self.projected_coord = projected_coord

	def calc_destination_angle(self):
		x_dist = self.projected_coord[0] - self.current_coord[0]
		y_dist = self.projected_coord[1] - self.current_coord[1]

		print x_dist
		print y_dist
		
		raw_angle = np.tan([y_dist/x_dist])
		print raw_angle
		adjusted_angle = self.current_angle - raw_angle

		return adjusted_angle

class Communication:

	def __init__(self):
		pass

# Utility Methods
def log(msg):
	if DEBUG:
		print "catcher.py: " + str(msg);

if __name__ == '__main__':
	#Find()
	sample_data_points = np.array([(1, 1, 1), (2, 2, 2), (3, 3, 3), (4.5, 4, 4), (5, 5, 1.5), (6, 6, 0.75), (7, 7, 0.25), (8, 8, .2), (9, 9, 0.1)])
	trajectory = Trajectory(sample_data_points)

	projected_coord = trajectory.get_projected_coordinates()
	log(projected_coord);

	navigation = Navigation([0,0], 90, projected_coord);
	log(navigation.calc_destination_angle());

	trajectory.show_polynomials()

