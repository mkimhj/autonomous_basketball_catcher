#! /usr/bin/env python

import numpy as np
from geometry_msgs.msg import PointStamped
import freenect
import cv

class Find:

	def __init__(self):
		cv.NamedWindow('Video')
		while True:
			self.process_image(freenect.sync_get_depth()[0])

	def process_image(self, depth):
		np.clip(depth, 0, 2**10 - 1, depth)
		depth = depth.astype(np.uint8)
		image = cv.CreateImageHeader((depth.shape[1], depth.shape[0]), cv.IPL_DEPTH_8U, 1)
		cv.SetData(image, depth.tostring(), depth.dtype.itemsize * depth.shape[1])
		cv.ShowImage('Video', image)
		cv.WaitKey(10)

if __name__ == '__main__':
	Find()
