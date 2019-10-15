#!/usr/bin/env python

import roslib

import sys
import rospy
import cv2

# import the necessary packages
from collections import deque
from imutils.video import VideoStream
import numpy as np
import argparse
import imutils
import time
import geometry_msgs
import sensor_msgs.point_cloud2 as pc2

from kortex_demos.srv import GetCoordinates
from std_msgs.msg import String
from sensor_msgs.msg import Image, PointCloud2, PointField
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point, Vector3

class finding_z:

	def __init__(self):

		self.bridge = CvBridge()
		self.x_pixel = 0
		self.y_pixel = 0
		self.z_metre = 0

		rospy.wait_for_service("get_xy_coordinates")
		self.srv_get_xy_coordinates = rospy.ServiceProxy('get_xy_coordinates', GetCoordinates)

		self.srv_get_xyz_coordinates = rospy.Service("get_xyz_coordinates",GetCoordinates, self.get_xyz_coordinates_callback)
		
		self.depthcam = rospy.Subscriber("/camera/depth_registered/points",PointCloud2,self.get_z)

	def get_z(self,cloud):

		point = self.srv_get_xy_coordinates().point

		x_cam = point.x
		y_cam = point.y

		matrix_pixel = []
		for i in range(0,10):
			for j in range(0,10):
				n = i - 5
				m = j - 5
				d = int(x_cam - n)
				f = int(y_cam - m)
				matrix_pixel.append([d, f])

		cloud_points = pc2.read_points(cloud, skip_nans=True, field_names = ("z"), uvs=matrix_pixel)

		z_list = []

		for z in cloud_points:
			z_list.append(z[0])

		try :
			z_mean = sum(z_list)/ len(z_list)
		except :
			z_mean = 0

		self.x_pixel = float(x_cam)
		self.y_pixel = float(y_cam)
		self.z_metre = float(z_mean)
		
		# print("x : ", self.x_pixel, end=" - ")
		# print("y : ", self.y_pixel, end=" - ")
		# print("z : ", self.z_metre)
	
	def get_xyz_coordinates_callback(self,request):
		try:
			return Vector3(self.x_pixel,self.y_pixel,self.z_metre)

		except:
			print("get_xyz_coordinates_callback")
	
def main(args):
	
	rospy.init_node('cube_z_processing', anonymous=True)
	ic = finding_z()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
		cv2.destroyAllWindows()

if __name__ == '__main__':
	# construct the argument parse and parse the arguments
	ap = argparse.ArgumentParser()
	ap.add_argument("-v", "--video",
		help="path to the (optional) video file")
	ap.add_argument("-b", "--buffer", type=int, default=64,
		help="max buffer size")
	args = vars(ap.parse_args(rospy.myargv()[1:]))
	main(args)
