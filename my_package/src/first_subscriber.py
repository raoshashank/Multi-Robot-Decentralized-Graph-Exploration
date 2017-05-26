#! /usr/bin/env python

import rospy
import numpy as np 
from std_msgs.msg import Int32
import matplotlib.pyplot as plt
from gazebo_msgs.msg import LinkStates
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
def callback(msg):
  #rate.sleep()
  """
  plt.plot (np.arange(-135,135,0.375),msg.ranges)
  plt.pause(0.001)
  plt.draw()
  """
  euler=euler_from_quaternion(msg.pose.pose)
  print euler
rospy.init_node('topic_subscriber')
rate=rospy.Rate(20)
#msg=LaserScan()
#msg.ranges=np.arange(-135,135,0.375)
#plt.show(block=False)  
sub=rospy.Subscriber('/odometry/filtered',Odometry,callback)
rospy.spin()

 