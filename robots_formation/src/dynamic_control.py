#!/usr/bin/env python
import rospy
import numpy as np
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

uref = 0.0
wref = 0.0
x1 = 1.0
y1 = 0.0
theta1 = 0.0
x2 = -1.0
y2 = -1.0
theta2 = 0.0
x3 = -1.0001
y3 = 1.0001
theta3 = 0.0
setpoint = np.array([[2],[2],[60*math.pi/180],[0],[0],[0]])
setpoint_ant = setpoint

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

    x1=msg.pose.pose.position.x
    y1=msg.pose.pose.position.y
    rot_q=msg.pose.pose.orientation
    angles=euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    theta1=angles[2]

#ODOMETRY DATA Robot 2
def Odom2(msg):
    global x2
    global y2
    global theta2

    x2=msg.pose.pose.position.x
    y2=msg.pose.pose.position.y
    rot_q=msg.pose.pose.orientation
    angles=euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    theta2=angles[2]

#ODOMETRY DATA Robot 3
def Odom3(msg):
    global x3
    global y3
    global theta3

    x3=msg.pose.pose.position.x
    y3=msg.pose.pose.position.y
    rot_q=msg.pose.pose.orientation
    angles=euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    theta3=angles[2]

def references(setpoint_):
    setpoint_[0,0] = rospy.get_param('d1r',setpoint_[0,0])
    setpoint_[1,0] = rospy.get_param('d2r',setpoint_[1,0])
    setpoint_[2,0] = rospy.get_param('betar',setpoint_[2,0])
    setpoint_[3,0] = rospy.get_param('xcr',setpoint_[3,0])
    setpoint_[4,0] = rospy.get_param('ycr',setpoint_[4,0])
    setpoint_[5,0] = rospy.get_param('thetar',setpoint_[5,0])
    return setpoint_

def controlFormation(x_1,y_1,x_2,y_2,x_3,y_3,theta_1,theta_2,theta_3, setpoint_, setpoint_ant_):
    d_1 = math.sqrt((x_1 - x_3)**2 + (y_1 - y_3)**2)
    d_2 = math.sqrt((x_1 - x_2)**2 + (y_1 - y_2)**2)
    d_3 = math.sqrt((x_2 - x_3)**2 + (y_2 - y_3)**2)
    beta_ = math.acos((d_1**2 + d_2**2 - d_3**2) / (2 * d_1 * d_2))
    xc_ = (x_1 + x_2 + x_3) / 3
    yc_ = (y_1 + y_2 + y_3) / 3
    theta_ = math.atan2(2/3 * x_1 - 1/3 * (x_2 + x_3), 2/3 * y_1 - 1/3 * (y_2 + y_3))
    q = np.array([[d_1],[d_2],[beta_],[xc_],[yc_],[theta_]])
    q_e = setpoint_ - q
    q_d = (setpoint_ - setpoint_ant_) / 0.1
    Jr = np.array([[math.cos(theta_1), -0.1 * math.sin(theta_1), 0.0 , 0.0 , 0.0 , 0.0], 
                    [math.sin(theta_1), 0.1 * math.cos(theta_1), 0.0 , 0.0 , 0.0 , 0.0], 
                    [ 0.0 , 0.0 , math.cos(theta_2), -0.1 * math.sin(theta_2), 0.0 , 0.0], 
                    [ 0.0 , 0.0 , math.sin(theta_2), 0.1 * math.cos(theta_2), 0.0 , 0.0], 
                    [ 0.0 , 0.0 , 0.0 , 0.0 , math.cos(theta_3), -0.1 * math.sin(theta_3)], 
                    [ 0.0 , 0.0 , 0.0 , 0.0 , math.sin(theta_3), 0.1 * math.cos(theta_3)]])
    J = np.array([[(2*x_1 - 2*x_3)/(2*((x_1 - x_3)**2 + (y_1 - y_3)**2)**(1/2)), (2*y_1 - 2*y_3)/(2*((x_1 - x_3)**2 + (y_1 - y_3)**2)**(1/2)), 0, 0, -(2*x_1 - 2*x_3)/(2*((x_1 - x_3)**2 + (y_1 - y_3)**2)**(1/2)), -(2*y_1 - 2*y_3)/(2*((x_1 - x_3)**2 + (y_1 - y_3)**2)**(1/2))],
                  [(2*x_1 - 2*x_2)/(2*((x_1 - x_2)**2 + (y_1 - y_2)**2)**(1/2)), (2*y_1 - 2*y_2)/(2*((x_1 - x_2)**2 + (y_1 - y_2)**2)**(1/2)), -(2*x_1 - 2*x_2)/(2*((x_1 - x_2)**2 + (y_1 - y_2)**2)**(1/2)), -(2*y_1 - 2*y_2)/(2*((x_1 - x_2)**2 + (y_1 - y_2)**2)**(1/2)), 0, 0],
                  [((2*x_2 - 4*x_1 + 2*x_3)/(2*((x_1 - x_2)**2 + (y_1 - y_2)**2)**(1/2)*((x_1 - x_3)**2 + (y_1 - y_3)**2)**(1/2)) + ((2*x_1 - 2*x_2)*((x_1 - x_2)**2 + (x_1 - x_3)**2 - (x_2 - x_3)**2 + (y_1 - y_2)**2 + (y_1 - y_3)**2 - (y_2 - y_3)**2))/(4*((x_1 - x_2)**2 + (y_1 - y_2)**2)**(3/2)*((x_1 - x_3)**2 + (y_1 - y_3)**2)**(1/2)) + ((2*x_1 - 2*x_3)*((x_1 - x_2)**2 + (x_1 - x_3)**2 - (x_2 - x_3)**2 + (y_1 - y_2)**2 + (y_1 - y_3)**2 - (y_2 - y_3)**2))/(4*((x_1 - x_2)**2 + (y_1 - y_2)**2)**(1/2)*((x_1 - x_3)**2 + (y_1 - y_3)**2)**(3/2)))/(1 - ((x_1 - x_2)**2 + (x_1 - x_3)**2 - (x_2 - x_3)**2 + (y_1 - y_2)**2 + (y_1 - y_3)**2 - (y_2 - y_3)**2)**2/(4*((x_1 - x_2)**2 + (y_1 - y_2)**2)*((x_1 - x_3)**2 + (y_1 - y_3)**2)))**(1/2), ((2*y_2 - 4*y_1 + 2*y_3)/(2*((x_1 - x_2)**2 + (y_1 - y_2)**2)**(1/2)*((x_1 - x_3)**2 + (y_1 - y_3)**2)**(1/2)) + ((2*y_1 - 2*y_2)*((x_1 - x_2)**2 + (x_1 - x_3)**2 - (x_2 - x_3)**2 + (y_1 - y_2)**2 + (y_1 - y_3)**2 - (y_2 - y_3)**2))/(4*((x_1 - x_2)**2 + (y_1 - y_2)**2)**(3/2)*((x_1 - x_3)**2 + (y_1 - y_3)**2)**(1/2)) + ((2*y_1 - 2*y_3)*((x_1 - x_2)**2 + (x_1 - x_3)**2 - (x_2 - x_3)**2 + (y_1 - y_2)**2 + (y_1 - y_3)**2 - (y_2 - y_3)**2))/(4*((x_1 - x_2)**2 + (y_1 - y_2)**2)**(1/2)*((x_1 - x_3)**2 + (y_1 - y_3)**2)**(3/2)))/(1 - ((x_1 - x_2)**2 + (x_1 - x_3)**2 - (x_2 - x_3)**2 + (y_1 - y_2)**2 + (y_1 - y_3)**2 - (y_2 - y_3)**2)**2/(4*((x_1 - x_2)**2 + (y_1 - y_2)**2)*((x_1 - x_3)**2 + (y_1 - y_3)**2)))**(1/2), ((2*x_1 - 2*x_3)/(2*((x_1 - x_2)**2 + (y_1 - y_2)**2)**(1/2)*((x_1 - x_3)**2 + (y_1 - y_3)**2)**(1/2)) - ((2*x_1 - 2*x_2)*((x_1 - x_2)**2 + (x_1 - x_3)**2 - (x_2 - x_3)**2 + (y_1 - y_2)**2 + (y_1 - y_3)**2 - (y_2 - y_3)**2))/(4*((x_1 - x_2)**2 + (y_1 - y_2)**2)**(3/2)*((x_1 - x_3)**2 + (y_1 - y_3)**2)**(1/2)))/(1 - ((x_1 - x_2)**2 + (x_1 - x_3)**2 - (x_2 - x_3)**2 + (y_1 - y_2)**2 + (y_1 - y_3)**2 - (y_2 - y_3)**2)**2/(4*((x_1 - x_2)**2 + (y_1 - y_2)**2)*((x_1 - x_3)**2 + (y_1 - y_3)**2)))**(1/2), ((2*y_1 - 2*y_3)/(2*((x_1 - x_2)**2 + (y_1 - y_2)**2)**(1/2)*((x_1 - x_3)**2 + (y_1 - y_3)**2)**(1/2)) - ((2*y_1 - 2*y_2)*((x_1 - x_2)**2 + (x_1 - x_3)**2 - (x_2 - x_3)**2 + (y_1 - y_2)**2 + (y_1 - y_3)**2 - (y_2 - y_3)**2))/(4*((x_1 - x_2)**2 + (y_1 - y_2)**2)**(3/2)*((x_1 - x_3)**2 + (y_1 - y_3)**2)**(1/2)))/(1 - ((x_1 - x_2)**2 + (x_1 - x_3)**2 - (x_2 - x_3)**2 + (y_1 - y_2)**2 + (y_1 - y_3)**2 - (y_2 - y_3)**2)**2/(4*((x_1 - x_2)**2 + (y_1 - y_2)**2)*((x_1 - x_3)**2 + (y_1 - y_3)**2)))**(1/2), ((2*x_1 - 2*x_2)/(2*((x_1 - x_2)**2 + (y_1 - y_2)**2)**(1/2)*((x_1 - x_3)**2 + (y_1 - y_3)**2)**(1/2)) - ((2*x_1 - 2*x_3)*((x_1 - x_2)**2 + (x_1 - x_3)**2 - (x_2 - x_3)**2 + (y_1 - y_2)**2 + (y_1 - y_3)**2 - (y_2 - y_3)**2))/(4*((x_1 - x_2)**2 + (y_1 - y_2)**2)**(1/2)*((x_1 - x_3)**2 + (y_1 - y_3)**2)**(3/2)))/(1 - ((x_1 - x_2)**2 + (x_1 - x_3)**2 - (x_2 - x_3)**2 + (y_1 - y_2)**2 + (y_1 - y_3)**2 - (y_2 - y_3)**2)**2/(4*((x_1 - x_2)**2 + (y_1 - y_2)**2)*((x_1 - x_3)**2 + (y_1 - y_3)**2)))**(1/2), ((2*y_1 - 2*y_2)/(2*((x_1 - x_2)**2 + (y_1 - y_2)**2)**(1/2)*((x_1 - x_3)**2 + (y_1 - y_3)**2)**(1/2)) - ((2*y_1 - 2*y_3)*((x_1 - x_2)**2 + (x_1 - x_3)**2 - (x_2 - x_3)**2 + (y_1 - y_2)**2 + (y_1 - y_3)**2 - (y_2 - y_3)**2))/(4*((x_1 - x_2)**2 + (y_1 - y_2)**2)**(1/2)*((x_1 - x_3)**2 + (y_1 - y_3)**2)**(3/2)))/(1 - ((x_1 - x_2)**2 + (x_1 - x_3)**2 - (x_2 - x_3)**2 + (y_1 - y_2)**2 + (y_1 - y_3)**2 - (y_2 - y_3)**2)**2/(4*((x_1 - x_2)**2 + (y_1 - y_2)**2)*((x_1 - x_3)**2 + (y_1 - y_3)**2)))**(1/2)],
                  [1.0/3.0, 0, 1.0/3.0, 0, 1.0/3.0, 0],
                  [0, 1.0/3.0, 0, 1.0/3.0, 0, 1.0/3.0],
                  [-2/(3*((x_2/3 - (2*x_1)/3 + x_3/3)**2/(y_2/3 - (2*y_1)/3 + y_3/3)**2 + 1)*(y_2/3 - (2*y_1)/3 + y_3/3)), (2*(x_2/3 - (2*x_1)/3 + x_3/3))/(3*((x_2/3 - (2*x_1)/3 + x_3/3)**2/(y_2/3 - (2*y_1)/3 + y_3/3)**2 + 1)*(y_2/3 - (2*y_1)/3 + y_3/3)**2), 1/(3*((x_2/3 - (2*x_1)/3 + x_3/3)**2/(y_2/3 - (2*y_1)/3 + y_3/3)**2 + 1)*(y_2/3 - (2*y_1)/3 + y_3/3)), -(x_2/3 - (2*x_1)/3 + x_3/3)/(3*((x_2/3 - (2*x_1)/3 + x_3/3)**2/(y_2/3 - (2*y_1)/3 + y_3/3)**2 + 1)*(y_2/3 - (2*y_1)/3 + y_3/3)**2), 1/(3*((x_2/3 - (2*x_1)/3 + x_3/3)**2/(y_2/3 - (2*y_1)/3 + y_3/3)**2 + 1)*(y_2/3 - (2*y_1)/3 + y_3/3)), -(x_2/3 - (2*x_1)/3 + x_3/3)/(3*((x_2/3 - (2*x_1)/3 + x_3/3)**2/(y_2/3 - (2*y_1)/3 + y_3/3)**2 + 1)*(y_2/3 - (2*y_1)/3 + y_3/3)**2)]
                  ])
    control_ = np.dot(np.linalg.inv(np.dot(J,Jr)),q_d + 0.6 * q_e)
    return control_, setpoint_

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
            setpoint = references(setpoint)
            control, setpoint_ant = controlFormation(x1,y1,x2,y2,x3,y3,theta1,theta2,theta3,setpoint,setpoint_ant)            
            cmd1 = Twist()
            cmd1.linear.x = control[0,0]
            cmd1.angular.z = control[1,0]
            cmd2 = Twist()
            cmd2.linear.x = control[2,0]
            cmd2.angular.z = control[3,0]
            cmd3 = Twist()
            cmd3.linear.x = control[4,0]
            cmd3.angular.z = control[5,0]
            diff_vel1.publish(cmd1)
            diff_vel2.publish(cmd2)
            diff_vel3.publish(cmd3)
            rate.sleep