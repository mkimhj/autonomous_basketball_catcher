#! /usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import tf
from math import pi, atan, isnan, sqrt

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
		rospy.init_node('catcher_find')
		rospy.Subscriber('/camera/depth/image_raw', Image, self.process_image)
		self.tf = tf.TransformBroadcaster()
		self.cv_bridge = CvBridge()
		rospy.spin()

	def process_image(self, msg):
		depth = self.cv_bridge.imgmsg_to_cv2(msg)
		depth = depth.astype(float) / K_MAX_DEPTH_MM
		depth[depth == 0] = 1
		ranged = cv2.inRange(depth, 0, 0.2)
		contours, _ = cv2.findContours(ranged.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
		processed = np.zeros((480, 640, 3), np.uint8)
		processed[:, :, 0] = ranged
		for contour in contours:
			area = cv2.contourArea(contour)
			radius = sqrt(area / pi)
			x, y, w, h = cv2.boundingRect(contour)
			x, y = x + w / 2, y + h / 2
			z = depth[y, x] * 10
			aspect = float(w) / h
			if 15 < radius * z < 35 and 500 < area < 5000 and (1.0 / 1.6) < aspect < 1.6:
				cv2.drawContours(processed, np.array([contour]), -1, (0, 255, 0), 3)
				if not isnan(z):
					self.tf.sendTransform(self.project(x, y, z), (0, 0, 0, 1),
						msg.header.stamp, "trash", "camera_depth_optical_frame")
				break
		cv2.imshow('depth', processed)
		cv2.waitKey(10)

	def project(self, u, v, z):
		u -= K_HALF_WIDTH
		v -= K_HALF_HEIGHT
		x = (u / K_HORIZONTAL_DEPTH) * z
		y = (v / K_VERTICAL_DEPTH) * z
		return x, y, z

if __name__ == '__main__':
	Find()
