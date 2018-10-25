#!/usr/bin/env python
# coding: utf-8

import rospy
import os
import numpy as np
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseStamped
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *


class WaypointMaker():
  def __init__(self):
    rospy.init_node('waypoint_maker')
    self.waypoints = PoseArray()
    self.wp_pub = rospy.Publisher('/waypoints', PoseArray, queue_size=10)
    self.file_name = "kakunin_waypoints.txt"
    self.num = 0
    self.server = None
    self.menu_handler = MenuHandler()

  def pose_callback(self, data):
    print "waypoint received"
    print "[" + str(self.num) + "]" + "(" + str(data.pose.position.x) + ", " + str(data.pose.position.y) + ")"
    self.waypoints.poses.append(data.pose)
    self.num += 1
    self.wp_pub.publish(self.waypoints)

  def interact_callback(self, data):
    print data

  def process(self):
    print "=== waypoint_maker ==="

    rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.pose_callback)

    self.waypoints.header.frame_id = "map"
    #p0 = Pose()
    #p0.orientation.w = 1
    #waypoints.poses.append(p0)

    r = rospy.Rate(10)

    self.server = InteractiveMarkerServer("control")
    self.menu_handler.insert("entry", callback=self.interact_callback)
    self.server.applyChanges()

    print "press n to exit"

    self.wp_pub.publish(self.waypoints)

    while not rospy.is_shutdown():
      s = raw_input()
      if((s is "n") or (s is "c")):
        wp_list = ""
        for p in self.waypoints.poses:
          wp_list += str(p.position.x) + " " + str(p.position.y) + "\n"
        with open(os.path.dirname(__file__) + "/../param/" + self.file_name, mode="w") as f:
          f.write(wp_list)
          print self.file_name + " generated!"
        exit()
      r.sleep()

if __name__ == '__main__':
  wm = WaypointMaker()
  try:
    wm.process()
  except rospy.ROSInterruptException: pass
