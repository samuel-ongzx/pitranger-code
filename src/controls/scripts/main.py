#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import sys

body_vel = 0.0
yaw_rate = 0.0
publish_rate = 1

max_body_vel = 0.05
max_yaw_rate = 0.05
max_publish_rate = 10

def talker():
    pub = rospy.Publisher('/pitranger/in/twist_cmd', Twist, queue_size=10)
    rospy.init_node('pr_control_node', anonymous=True)
    rate = rospy.Rate(publish_rate) # 10hz

    while not rospy.is_shutdown():
        msg = Twist()
        msg.linear.x = body_vel
        msg.angular.z = yaw_rate
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    expected_args = 3
    received_args = len(sys.argv)-1
    if(not expected_args == received_args):
        print("You passed in {} arugments but expected {}".format(received_args, expected_args))
    else: 
        body_vel = float(sys.argv[1])
        yaw_rate = float(sys.argv[2])
        publish_rate = int(sys.argv[3])
        
        mag_body_vel = abs(body_vel)
        mag_yaw_rate = abs(yaw_rate)

        proceed = True
        if(mag_body_vel > max_body_vel):
            print("Magnitude of body velocity received ({}) exceeds limit {}".format(mag_body_vel, max_body_vel))
            proceed = False
        if(mag_yaw_rate > max_yaw_rate):
            print("Magnitude of yaw rate received ({}) exceeds limit {}".format(mag_yaw_rate, max_yaw_rate))
            proceed = False
        if(not publish_rate in range(1, max_publish_rate+1)):
            print("Publish rate received ({}) is out of range [1,{}]".format(publish_rate, max_publish_rate))
            proceed = False
        if(proceed):
            print("Sending drive command!")
            try:
                talker()
            except rospy.ROSInterruptException:
                pass
        else:
            print
            print("Usage example:")
            print("rosrun controls main2.py -0.02 0.03 7")
            print("issues a \"move backwards at 2cm/second with 0.03 yaw rate\" command 7 times per second")
