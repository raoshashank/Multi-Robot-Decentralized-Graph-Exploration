#! /usr/bin/env python

import rospy
import numpy as np 
from std_msgs.msg import Int32
import matplotlib.pyplot as plt
from gazebo_msgs.msg import LinkStates
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from math import sin,cos,atan2
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped
from project.msg import vertex_info

def callback(msg):
  #rospy.loginfo("X:"+str(msg.pose.spose.position.x)+"    Y:"+str(msg.pose.pose.position.y)+"    Z:"+str(msg.pose.pose.position.z))
  odom_feedback=msg
  rospy.loginfo("X: "+str(msg.pose.pose.position.x)+" Y: "+str(msg.pose.pose.position.y))

"""  
  q=[0,0,0,0]
  q[0]=odom_feedback.pose.pose.orientation.w
  q[1]=odom_feedback.pose.pose.orientation.x
  q[2]=odom_feedback.pose.pose.orientation.y
  q[3]=odom_feedback.pose.pose.orientation.z  
  heading=atan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]*q[2]+q[3]*q[3]))
"""
  
  

rospy.init_node('topic_subscriber')
bot_no=0
topic_name="/bot_"+str(bot_no)
rate=rospy.Rate(20)
sub=rospy.Subscriber('/bot_0/odom',Odometry,callback)
rospy.spin()

 