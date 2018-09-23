#!/usr/bin/env python
# coding: utf-8

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

def ImageCallback(data):
  print("=== ImageCallback ===")
  try:
    # bgr
    bgr_image = bridge.imgmsg_to_cv2(data, "bgr8")
    cv2.rectangle(bgr_image, (0, 0), (640, 360), (0, 0, 0), cv2.FILLED)
    cv2.rectangle(bgr_image, (0, 360), (200, 480), (0, 0, 0), cv2.FILLED)
    cv2.rectangle(bgr_image, (440, 360), (640, 480), (0, 0, 0), cv2.FILLED)
    cv2.namedWindow("bgr", cv2.WINDOW_NORMAL)
    cv2.imshow("bgr", bgr_image)
    # hsv
    hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
    cv2.namedWindow("hsv", cv2.WINDOW_NORMAL)
    cv2.imshow("hsv", hsv_image)
    # hsv mask
    lower_white = np.array([0, 0, 70])
    upper_white = np.array([180, 30, 100])
    mask_white = cv2.inRange(hsv_image, lower_white, upper_white)
    cv2.namedWindow("hsv_white", cv2.WINDOW_NORMAL)
    cv2.imshow("hsv_white", mask_white)
    # gaussian blur
    gauss_img = cv2.GaussianBlur(mask_white, (25, 25), 1)
    cv2.namedWindow("gauss", cv2.WINDOW_NORMAL)
    cv2.imshow("gauss", gauss_img)
    # hough
    edges = cv2.Canny(gauss_img, 50, 100, apertureSize = 3)
    minLineLength = 100
    maxLineGap = 10
    lines = cv2.HoughLinesP(edges, 1, np.pi/180.0, 100, minLineLength, maxLineGap)
    _image = bgr_image
    if lines is not None :
      for x1, y1, x2, y2 in lines[0]:
        if(abs(y1 - y2) < 1e-6):
          cv2.line(_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
    cv2.namedWindow("hough", cv2.WINDOW_NORMAL)
    cv2.imshow("hough", _image)

    cv2.waitKey(3)

  except CvBridgeError as e:
    print(e)

def process():
  img_sub = rospy.Subscriber('/usb_cam/image_raw', Image, ImageCallback)


  r = rospy.Rate(10)

  while not rospy.is_shutdown():
    print "=== white_line_detector ==="

    r.sleep()

if __name__ == '__main__':
  rospy.init_node('white_line_detector')
  try:
    process()
  except rospy.ROSInterruptException: pass
