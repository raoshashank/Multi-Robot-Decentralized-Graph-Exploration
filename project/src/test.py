#!/usr/bin/env python
import rospy
from math import pi,atan2
import numpy as np
from math import pi,asin,sin
from matrix_op import matrix_op
from nav_msgs.msg import Odometry

def callback(msg):
    q=[0,0,0,0]
    q[0]=msg.pose.pose.orientation.w
    q[1]=msg.pose.pose.orientation.x
    q[2]=msg.pose.pose.orientation.y
    q[3]=msg.pose.pose.orientation.z  
    heading=atan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]*q[2]+q[3]*q[3]))
    rospy.loginfo(heading)

rospy.init_node("test")

sub=rospy.Subscriber('/bot_0/odom',Odometry,callback)
rospy.spin()