#!/usr/bin/env python

from __future__ import print_function
import rospy
import time
from std_msgs.msg import String
import subprocess


if __name__=="__main__":
  rospy.init_node('wifi_monitor')

  wifi_pub = rospy.Publisher('/wifi/', String, queue_size=10)

  while not rospy.is_shutdown():
    proc = subprocess.Popen(['iwconfig','wlan0'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    try:
      outs, errs = proc.communicate()
      print(outs)
      wifi_pub.publish(outs)
    except Exception as khaled_is_dumb:
      print("Exception: {}".format(khaled_is_dumb))
    time.sleep(1.0)
