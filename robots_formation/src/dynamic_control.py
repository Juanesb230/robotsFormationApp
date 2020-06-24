#!/usr/bin/env python
import rospy
import numpy as np
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry

uref = 0.0
wref = 0.0
x1 = 0.0
y1 = 0.0
theta1 = 0.0
x2 = 0.0
y2 = 0.0
theta2 = 0.0
x3 = 0.0
y3 = 0.0
theta3 = 0.0

def velRef(msg):
    global uref
    global wref
    uref=msg.linear.x
    wref=msg.angular.z
    print(uref)
    print(wref)

#ODOMETRY DATA Robot 1
def Odom1(msg):
    global x1
    global y1
    global theta1

    x1=msg.pose.pose.position.x + 1.0
    y1=msg.pose.pose.position.y
    rot_q=msg.pose.pose.orientation
    angles=euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    theta1=angles[2]

#ODOMETRY DATA Robot 2
def Odom2(msg):
    global x2
    global y2
    global theta2

    x2=msg.pose.pose.position.x - 1.0
    y2=msg.pose.pose.position.y - 1.0
    rot_q=msg.pose.pose.orientation
    angles=euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    theta2=angles[2]

#ODOMETRY DATA Robot 3
def odom3(msg):
    global x3
    global y3
    global theta3

    x3=msg.pose.pose.position.x - 1.0
    y3=msg.pose.pose.position.y + 1.0
    rot_q=msg.pose.pose.orientation
    angles=euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    theta3=angles[2]

def variablesFormation(x_1,y_1,x_2,y_2,x_3,y_3):
    d1 = math.sqrt((x_1 - x_3)**2 + (y_1 - y_3)**2)
    d2 = math.sqrt((x_1 - x_2)**2 + (y_1 - y_2)**2)
    d3 = math.sqrt((x_2 - x_3)**2 + (y_2 - y_3)**2)
    beta = math.acos((d_1**2 + d_2**2 - d_3**2) / (2 * d_1 * d_2))
    xc = (x_1 + x_2 + x_3) / 3
    yc = (y_1 + y_2 + y_3) / 3
    theta = math.atan2(2/3 * x_1 - 1/3 * (x_2 + x_3), 2/3 * y_1 - 1/3 * (y_2 + y_3))
    return d1,d2,beta,xc,yc,theta

if __name__ == '__main__':
        #NODE DEFINITION
        rospy.init_node('dynamic_control')
        sub_vel = rospy.Subscriber('/cmd_vel',Twist,velRef)
        sub_odom1 = rospy.Subscriber('/odom1',Odometry,Odom1)
        sub_odom2 = rospy.Subscriber('/odom2',Odometry,Odom2)
        sub_odom3 = rospy.Subscriber('/odom3',Odometry,Odom3)
        diff_vel1 = rospy.Publisher('/cmd_vel1',Twist,queue_size=1)
        diff_vel2 = rospy.Publisher('/cmd_vel2',Twist,queue_size=1)
        diff_vel3 = rospy.Publisher('/cmd_vel3',Twist,queue_size=1)
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            #d1,d2,beta,xc,yc,theta = variablesFormation(x1,y1,x2,y2,x3,y3)
            #var = np.array([[d1],[d2],[beta],[xc],[yc],[theta]])
            print (1)
            cmd = Twist()
            cmd.linear.x = uref
            cmd.angular.z = wref
            diff_vel1.publish(cmd)
            diff_vel2.publish(cmd)
            diff_vel3.publish(cmd)
            rate.sleep