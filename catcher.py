#! /usr/bin/env python

import numpy as np
from scipy.optimize import curve_fit
import freenect
import cv2
from math import pi, atan

DEGREES_TO_RADIANS = pi / 180
K_VERTICAL_FOV = 43 * DEGREES_TO_RADIANS
K_HORIZONTAL_FOV = 57 * DEGREES_TO_RADIANS
K_HALF_WIDTH = 640 / 2
K_HALF_HEIGHT = 480 / 2
K_VERTICAL_DEPTH = K_HALF_HEIGHT / atan(K_VERTICAL_FOV / 2)
K_HORIZONTAL_DEPTH = K_HALF_WIDTH / atan(K_HORIZONTAL_FOV / 2)
K_MAX_DEPTH_MM = 10000

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
		cv2.drawContours(processed, contours, -1, (0, 255, 0), 3)
		cv2.imshow('depth', depth)
		cv2.imshow('processed', processed)
		cv2.waitKey(10)

	def project(self, u, v, z):
		u -= K_HALF_WIDTH
		v -= K_HALF_HEIGHT
		x = (u / K_HORIZONTAL_DEPTH) * z
		y = (v / K_VERTICAL_DEPTH) * z
		return x, -y, -z

if __name__ == '__main__':
	Find()
