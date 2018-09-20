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
    cv2.rectangle(bgr_image, (0, 240), (640, 480), (0, 0, 0), cv2.FILLED)
    cv2.namedWindow("bgr", cv2.WINDOW_NORMAL)
    cv2.imshow("bgr", bgr_image)
    # hsv
    hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
    cv2.namedWindow("hsv", cv2.WINDOW_NORMAL)
    cv2.imshow("hsv", hsv_image)
    # hsv mask:[b, g, r]
    lower_red = np.array([0, 150, 50])
    upper_red = np.array([10, 255, 255])
    mask_red = cv2.inRange(hsv_image, lower_red, upper_red)
    red = cv2.bitwise_and(bgr_image, bgr_image, mask= mask_red)
    avg_red = red.mean(0).mean(0)
    cv2.putText(red, str(avg_red), (0, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0))
    cv2.namedWindow("red", cv2.WINDOW_NORMAL)
    cv2.imshow("red", red)
    lower_green = np.array([30, 100, 50])
    upper_green = np.array([90, 255, 255])
    mask_green = cv2.inRange(hsv_image, lower_green, upper_green)
    green = cv2.bitwise_and(bgr_image, bgr_image, mask= mask_green)
    avg_green = green.mean(0).mean(0)
    cv2.putText(green, str(avg_green), (0, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0))
    cv2.namedWindow("green", cv2.WINDOW_NORMAL)
    cv2.imshow("green", green)

    if (np.argmax(avg_red) != 2) and (np.argmax(avg_green) == 1):
      if avg_green[1] > 0.011:
        print "green"
    elif (np.argmax(avg_red) == 2) and (np.argmax(avg_green) != 1):
      if avg_red[2] > 0.011:
        print "red"
    else:
      print "no signal"

    cv2.waitKey(3)

  except CvBridgeError as e:
    print(e)

def process():
  print "=== trafic_light_discriminator ==="
  img_sub = rospy.Subscriber('/usb_cam/image_raw', Image, ImageCallback)

  r = rospy.Rate(10)

  while not rospy.is_shutdown():

    r.sleep()

if __name__ == '__main__':
  rospy.init_node('trafic_light_discriminator')
  try:
    process()
  except rospy.ROSInterruptException: pass
