#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def talker():
    pub = rospy.Publisher('/pitranger/in/twist_cmd', Twist, queue_size=10)
    rospy.init_node('pr_control_node', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    body_vel = 0.05
    yaw_rate = 0.0

    while not rospy.is_shutdown():
        msg = Twist()
        msg.linear.x = body_vel
        msg.angular.z = yaw_rate        
        
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass