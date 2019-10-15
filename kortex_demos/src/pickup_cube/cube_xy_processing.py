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

from std_msgs.msg import String, Float64, Int32, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point

from kortex_demos.srv import GetCoordinates, GetCubeWidth

class ImageConverter:

    def __init__(self):
        
        self.color_lower = ( 40, 80, 140 )
        self.color_upper = ( 65, 255, 255 )
        self.x_center = 0
        self.y_center = 0
        self.width = 0
        self.is_cube_found = Bool()

        self.bridge = CvBridge()

        self.srv_get_xy_coordinates = rospy.Service("get_xy_coordinates", GetCoordinates, self.get_xy_coordinates_callback)
        self.srv_get_width = rospy.Service("get_cube_width", GetCubeWidth, self.get_cube_width_callback)

        self.pub_cube_found = rospy.Publisher("is_cube_found", Bool, queue_size=1)

        self.sub_image = rospy.Subscriber("/camera/color/image_raw",Image,self.callback)
        #self.target_color = rospy.Subscriber("target_color",target_color,self.fct_target_color)

    # Find X and Y
    def callback(self,data):

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        except CvBridgeError as e:
            print(e)
        x = 0
        y = 0
        w = 0
        # grab the current frame
        frame = cv_image.view()

        # resize the frame, blur it, and convert it to the HSV
        # color space
        frame = imutils.resize(frame, height=480)
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # construct a mask for the color "green", then perform
        # a series of dilations and erosions to remove any small
        # blobs left in the mask
        mask = cv2.inRange(hsv, self.color_lower, self.color_upper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None

        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            x, y, w, h = cv2.boundingRect(c)
            # only proceed if the radius meets a minimum size
            if h > 25:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 255), 2)
                cv2.circle(frame, center, 2, (0, 0, 255), -1)
                self.x_center = x + w/2
                self.y_center = y + h/2
                self.width = w
                self.is_cube_found.data = True
            else :
                self.is_cube_found.data = False
        else :
                self.is_cube_found.data = False

        # update the points queue
        pts.appendleft(center)

        # loop over the set of tracked points
        for i in range(1, len(pts)):
            # if either of the tracked points are None, ignore them
            if pts[i - 1] is None or pts[i] is None:
                continue

        # publish if cube was found or not
        self.pub_cube_found.publish(self.is_cube_found)
                
        # show the frame to our screen
        cv2.imshow("Frame", frame)
        cv2.waitKey(3)

    # def fct_target_color(self,data):
    #     self.color_lower = ( data.h_low, data.s_low, data.v_low )
    #     self.color_upper = ( data.h_high, data.s_high, data.v_high )
    #     print("color update")

    def get_xy_coordinates_callback(self, request):
        # print("x : ", self.x_center, end=" - ")
        # print("y : ", self.y_center, end=" - ")
        # print("z : ", 0, end=" - ")
        # print("width : ", self.width)
        return (geometry_msgs.msg.Vector3(self.x_center,self.y_center,0))

    def get_cube_width_callback(self, request):
        width = int(self.width)
        return (width)

####################################################

def main(args):

    rospy.init_node('cube_xy_processing', anonymous=True)
    ic = ImageConverter()
    rospy.loginfo('Node is running')
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    # construct the argument parse and parse the arguments
    ap = argparse.ArgumentParser()
    ap.add_argument("-b", "--buffer", type=int, default=64,
        help="max buffer size")

    args = vars(ap.parse_args(rospy.myargv()[1:]))
    pts = deque(maxlen=args["buffer"])

    main(args)
