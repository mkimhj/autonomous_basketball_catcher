#! /usr/bin/env python

import numpy as np
from scipy.optimize import curve_fit
import freenect
import cv2

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

if __name__ == '__main__':
	Find()
