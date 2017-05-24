#!/usr/bin/env python
import rospy
from geometry_msgs.msg import  Twist,Point,Quaternion
from math import radians,pi,copysign,sqrt,pow
import tf
from nav_msgs.msg import Odometry 

def go_straight(dist):
    move_cmd=Twist()
    move_cmd.linear.x=linear_speed
    for i in range(dist/linear_speed):
        pub.publish(move_cmd)
        rate.sleep()
    move_cmd=Twist()
    pub.publish(move_cmd)

def turn(deg):
    deg=radians(deg)
    move_cmd=Twist()
    move_cmd.angular.z=deg
    for i in range  (deg/angular_speed):
        pub.publish(move_cmd)
        rate.sleep()
    move_cmd=Twist()
    pub.publish(move_cmd)

rospy.init_node('move_square',anonymous=False)

linear_speed=0.2
angular_speed=pi
square_side=1.0
square_angle=radians(90)

pub=rospy.Publisher('/cmd_vel_mux/input/teleop',Twist,queue_size=10)

rate=rospy.Rate(20)

for i in range(4):
    go_straight(square_side)
    turn(pi/2)
     