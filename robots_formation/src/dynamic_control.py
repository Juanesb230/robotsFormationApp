#!/usr/bin/env python 
import rospy
import numpy as np
from collections import deque
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

uref = 0.0
wref = 0.0
x1 = 0.0
y1 = 0.0
x2 = 0.0
y2 = 0.0
x3 = 0.0
y3 = 0.0

def velRef(msg):
    global uref
    global wref
    uref=msg.linear.x
    wref=msg.angular.z
    print(uref)
    print(wref)

if __name__ == '__main__':
        #NODE DEFINITION
        rospy.init_node('dynamic_control')
        sub_vel = rospy.Subscriber('/cmd_vel',Twist,velRef)
        diff_vel1 = rospy.Publisher('/cmd_vel1',Twist,queue_size=1)
        diff_vel2 = rospy.Publisher('/cmd_vel2',Twist,queue_size=1)
        diff_vel3 = rospy.Publisher('/cmd_vel3',Twist,queue_size=1)
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            cmd = Twist()
            cmd.linear.x = uref
            cmd.angular.z = wref
            diff_vel1.publish(cmd)
            diff_vel2.publish(cmd)
            diff_vel3.publish(cmd)
            rate.sleep