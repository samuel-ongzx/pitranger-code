#!/usr/bin/env python
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


def callback(msg):
    qx = msg.pose.pose.orientation.x;
    qy = msg.pose.pose.orientation.y;
    qz = msg.pose.pose.orientation.z;
    qw = msg.pose.pose.orientation.w;
    roll, pitch, yaw = euler_from_quaternion([qx, qy, qz, qw])

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    z = msg.pose.pose.position.z
    print("{:.3f}, {:.3f}, {:.3f}".format(x, y, yaw * 180.0/np.pi))

if __name__=="__main__":
    rospy.init_node('imu_interpreter')

    sub = rospy.Subscriber('whereami/odom', Odometry, callback)

    rospy.spin()

