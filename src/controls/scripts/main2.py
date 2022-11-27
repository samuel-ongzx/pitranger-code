#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import sys

def talker(body_vel, publish_rate):
    pub = rospy.Publisher('/pitranger/in/twist_cmd', Twist, queue_size=10)
    rospy.init_node('pr_control_node', anonymous=True)
    rate = rospy.Rate(publish_rate) # 10hz
    yaw_rate = body_vel / (-1.5)

    while not rospy.is_shutdown():
        msg = Twist()
        msg.linear.x = body_vel
        msg.angular.z = yaw_rate
        pub.publish(msg)
        rate.sleep()

max_body_vel = 0.05
max_publish_rate = 10
if __name__ == '__main__':
    expected_args = 2
    received_args = len(sys.argv)-1
    if(not expected_args == received_args):
        print("You passed in {} arugments but expected {}".format(received_args, expected_args))
    else: 
        body_vel = float(sys.argv[1])
        publish_rate = int(sys.argv[2])
        
        mag_body_vel = abs(body_vel)

        proceed = True
        if(mag_body_vel > max_body_vel):
            print("Magnitude of body velocity received ({}) exceeds limit {}".format(mag_body_vel, max_body_vel))
            proceed = False
        if(not publish_rate in range(1, max_publish_rate+1)):
            print("Publish rate received ({}) is out of range [1,{}]".format(publish_rate, max_publish_rate))
            proceed = False
        if(proceed):
            print("Sending drive command!")
            try:
                talker(body_vel, publish_rate)
            except rospy.ROSInterruptException:
                pass
        else:
            print
            print("Usage example:")
            print("rosrun controls main2.py -0.02 7")
            print("issues a \"move backwards at 2cm/second\" command 7 times per second")
