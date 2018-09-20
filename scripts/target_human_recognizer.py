#!/usr/bin/env python
# coding: utf-8

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

def ImageCallback(data):
  # print("=== ImageCallback ===")
  try:
    # bgr
    bgr_image = bridge.imgmsg_to_cv2(data, "bgr8")
    #cv2.rectangle(bgr_image, (0, 240), (640, 480), (0, 0, 0), cv2.FILLED)
    cv2.namedWindow("bgr", cv2.WINDOW_NORMAL)
    cv2.imshow("bgr", bgr_image)
    # hsv
    hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
    cv2.namedWindow("hsv", cv2.WINDOW_NORMAL)
    cv2.imshow("hsv", hsv_image)
    # hsv mask:[b, g, r]
    lower_orange = np.array([0, 200, 0])
    upper_orange = np.array([20, 255, 255])
    mask_orange = cv2.inRange(hsv_image, lower_orange, upper_orange)
    orange = cv2.bitwise_and(bgr_image, bgr_image, mask= mask_orange)
    cv2.namedWindow("orange", cv2.WINDOW_NORMAL)
    cv2.imshow("orange", orange)
    cv2.waitKey(3)

  except CvBridgeError as e:
    print(e)

def process():
  print "=== target_human_recognizer ==="
  img_sub = rospy.Subscriber('/usb_cam/image_raw', Image, ImageCallback)

  r = rospy.Rate(10)

  while not rospy.is_shutdown():

    r.sleep()

if __name__ == '__main__':
  rospy.init_node('target_human_recognizer')
  try:
    process()
  except rospy.ROSInterruptException: pass
