#! /usr/bin/env python

import rospy
import numpy as np 
from std_msgs.msg import Int32
import matplotlib.pyplot as plt
from gazebo_msgs.msg import LinkStates
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from math import sin,cos
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped
from project.msg import test,vertex_info

def callback(msg):
  #rospy.loginfo("X:"+str(msg.pose.spose.position.x)+"    Y:"+str(msg.pose.pose.position.y)+"    Z:"+str(msg.pose.pose.position.z))
  rospy.loginfo(msg.v[1])

rospy.init_node('topic_subscriber')
bot_no=0
topic_name="/bot_"+str(bot_no)
rate=rospy.Rate(20)
#sub=rospy.Subscriber('/robot_pose_ekf/odom_combined',PoseWithCovarianceStamped,callback)
sub=rospy.Subscriber('/counter',test,callback)
rospy.spin()

 