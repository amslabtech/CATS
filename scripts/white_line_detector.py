#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

def ImageCallback(data):
  print("=== ImageCallback ===")
  try:
    bgr_image = bridge.imgmsg_to_cv2(data, "bgr8")
    hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
    cv2.imshow("bgr", bgr_image)
    #cv2.imshow("hsv", hsv_image)
    gray_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2GRAY)

    thresh = 128
    max_pixel = 255
    ret, img_dst = cv2.threshold(gray_image, thresh, max_pixel, cv2.THRESH_BINARY)
    cv2.imshow("img_dst", img_dst)
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
