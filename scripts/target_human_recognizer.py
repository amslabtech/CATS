#!/usr/bin/env python
# coding: utf-8

import rospy
import cv2
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes

bridge = CvBridge()
bgr_image = np.empty(1)
image_flag = False
target_angle = 0
target_flag = False

def ImageCallback(data):
  # print("=== ImageCallback ===")
  try:
    # bgr
    global bgr_image
    bgr_image = bridge.imgmsg_to_cv2(data, "bgr8")
    #cv2.namedWindow("bgr", cv2.WINDOW_NORMAL)
    #cv2.imshow("bgr", bgr_image)
    # hsv
    hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
    #cv2.namedWindow("hsv", cv2.WINDOW_NORMAL)
    #cv2.imshow("hsv", hsv_image)
    lower_budapest_red = np.array([140, 50, 30])
    upper_budapest_red = np.array([200, 255, 150])
    mask_budapest_red = cv2.inRange(hsv_image, lower_budapest_red, upper_budapest_red)
    budapest_red = cv2.bitwise_and(bgr_image, bgr_image, mask=mask_budapest_red)
    #cv2.namedWindow("budapest_red", cv2.WINDOW_NORMAL)
    #cv2.imshow("budapest_red", budapest_red)
    lower_budapest_yellow = np.array([20, 100, 0])
    upper_budapest_yellow = np.array([40, 255, 255])
    mask_budapest_yellow = cv2.inRange(hsv_image, lower_budapest_yellow, upper_budapest_yellow)
    budapest_yellow = cv2.bitwise_and(bgr_image, bgr_image, mask=mask_budapest_yellow)
    #cv2.namedWindow("budapest_yellow", cv2.WINDOW_NORMAL)
    #cv2.imshow("budapest_yellow", budapest_yellow)
    #cv2.waitKey(3)
    global image_flag
    image_flag = True

  except CvBridgeError as e:
    print(e)

def is_orange(avr):
  return 0.5 < avr[0] and avr[0] < avr[1] and avr[1] < avr[2]

def is_budapest_red(avr):
  return avr[1] < avr[0] and avr[0] < avr[2] and avr[2] > 3.0

def is_budapest_yellow(avr):
  return 1

def BBCallback(data):
  if image_flag:
    try:
      count = 0
      for bb in data.bounding_boxes:
        if bb.Class == 'person':
          small_image = bgr_image[bb.ymin:bb.ymax, bb.xmin:bb.xmax]
          hsv_image = cv2.cvtColor(small_image, cv2.COLOR_BGR2HSV)
          # orange detection
          lower_orange = np.array([0, 200, 0])
          upper_orange = np.array([20, 255, 255])
          mask_orange = cv2.inRange(hsv_image, lower_orange, upper_orange)
          orange = cv2.bitwise_and(small_image, small_image, mask=mask_orange)
          avr_orange = orange.mean(0).mean(0)
          # budapest detection
          lower_budapest_red = np.array([140, 50, 30])
          upper_budapest_red = np.array([200, 255, 150])
          mask_budapest_red = cv2.inRange(hsv_image, lower_budapest_red, upper_budapest_red)
          budapest_red = cv2.bitwise_and(small_image, small_image, mask=mask_budapest_red)
          avr_budapest_red = budapest_red.mean(0).mean(0)
          #print avr_budapest_red
          lower_budapest_yellow = np.array([20, 100, 0])
          upper_budapest_yellow = np.array([40, 255, 255])
          mask_budapest_yellow = cv2.inRange(hsv_image, lower_budapest_yellow, upper_budapest_yellow)
          budapest_yellow = cv2.bitwise_and(small_image, small_image, mask=mask_budapest_yellow)
          avr_budapest_yellow = budapest_yellow.mean(0).mean(0)
          #print avr_budapest_yellow

          if is_orange(avr_orange) :
          #if is_budapest_red(avr_budapest_red) :
            cv2.namedWindow(bb.Class+str(count), cv2.WINDOW_NORMAL)
            #cv2.imshow(bb.Class, orange)
            cv2.imshow(bb.Class+str(count), orange)
            # c920r:77[deg]
            # image:480*640
            x_center = (bb.xmin + bb.xmax) * 0.5
            y_center = (bb.ymin + bb.ymax) * 0.5
            #:w
            print x_center, y_center
            #print bgr_image.shape
            angle = -(x_center / 640.0 * 77.0 - 38.5)
            global target_angle
            target_angle = angle
            global target_flag
            target_flag = True
            print str(angle) + "[deg]"
          count += 1
      cv2.waitKey(3)
    except CvBridgeError as e:
      print(e)

def process():
  print "=== target_human_recognizer ==="
  img_sub = rospy.Subscriber('/usb_cam/image_raw', Image, ImageCallback)
  yolo_sub = rospy.Subscriber('/human_rec/darknet_ros/bounding_boxes', BoundingBoxes, BBCallback)
  angle_pub = rospy.Publisher('/target/angle', Float32, queue_size=10)

  r = rospy.Rate(10)

  while not rospy.is_shutdown():
    if target_flag:
      angle = Float32()
      angle.data = target_angle
      angle_pub.publish(angle)
    r.sleep()

if __name__ == '__main__':
  rospy.init_node('target_human_recognizer')
  try:
    process()
  except rospy.ROSInterruptException: pass
