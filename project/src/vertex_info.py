#!/usr/bin/env python
import rospy
from project.msg import vertex_info,vertices
import numpy as np
from rospy.numpy_msg import numpy_msg
import pickle
from math import atan2,pi
from nav_msgs.msg import Odometry
def find_heading():
    global odom_feedback
    q=[0,0,0,0]
    q[0]=odom_feedback.pose.pose.orientation.w
    q[1]=odom_feedback.pose.pose.orientation.x
    q[2]=odom_feedback.pose.pose.orientation.y
    q[3]=odom_feedback.pose.pose.orientation.z  
    heading=atan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]*q[2]+q[3]*q[3]))
    return heading

def odom_callback(msg):
    global odom_feedback
    odom_feedback=msg
    heading=find_heading()
    rospy.loginfo(heading)
def callback(msg):
    msg=msg.v
    for i in msg:
        I=pickle.loads(i.I)
        rospy.loginfo(i.tag+str(I[:,1:I.shape[1]]))
        rospy.loginfo("#################################")

rospy.init_node('tester')
sub_o=rospy.Subscriber('/bot_0/odom',Odometry,odom_callback)
rospy.spin()










