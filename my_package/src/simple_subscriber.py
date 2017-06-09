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
def callback(msg):
  rospy.loginfo(str(msg.twist.twist.linear.y))

rospy.init_node('topic_subscriber')
rate=rospy.Rate(20)
sub=rospy.Subscriber('/bot_0/odom',Odometry,callback)
rospy.spin()

 