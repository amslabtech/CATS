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
    cv2.namedWindow("bgr", cv2.WINDOW_NORMAL)
    cv2.imshow("bgr", bgr_image)
    # hsv
    hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
    cv2.namedWindow("hsv", cv2.WINDOW_NORMAL)
    cv2.imshow("hsv", hsv_image)
    # hsv mask
    lower_white = np.array([0, 0, 100])
    upper_white = np.array([180, 50, 130])
    mask_white = cv2.inRange(hsv_image, lower_white, upper_white)
    res_white = cv2.bitwise_and(bgr_image, bgr_image, mask= mask_white)
    cv2.namedWindow("mask", cv2.WINDOW_NORMAL)
    cv2.imshow("mask", res_white)
    # 2値化
    #https://www.blog.umentu.work/python-opencv3%E3%81%A7%E7%94%BB%E5%83%8F%E3%81%AE%E7%94%BB%E7%B4%A0%E5%80%A4%E3%82%92%E4%BA%8C%E5%80%A4%E5%8C%96%E3%81%97%E3%81%A6%E5%87%BA%E5%8A%9B/
    gray_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2GRAY)
    thresh = 180
    max_pixel = 255
    ret, img_dst = cv2.threshold(gray_image, thresh, max_pixel, cv2.THRESH_BINARY)
    cv2.namedWindow("img_dst", cv2.WINDOW_NORMAL)
    cv2.imshow("img_dst", img_dst)
    cv2.waitKey(3)
    # hough
    edges = cv2.Canny(res_white, 50, 100, apertureSize = 3)
    minLineLength = 100
    maxLineGap = 10
    lines = cv2.HoughLinesP(edges, 1, np.pi/180.0, 100, minLineLength, maxLineGap)
    _image = bgr_image
    if lines is not None :
      for x1, y1, x2, y2 in lines[0]:
        cv2.line(_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
    cv2.namedWindow("hough", cv2.WINDOW_NORMAL)
    cv2.imshow("hough", _image)

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
