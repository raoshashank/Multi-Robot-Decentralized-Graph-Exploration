#! /usr/bin/env python

import rospy
import numpy as np 
from std_msgs.msg import Int32
import matplotlib.pyplot as plt
from gazebo_msgs.msg import LinkStates
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
def callback(msg):
  rate.sleep()
  plt.plot (np.arange(0,720),msg.ranges)
  plt.pause(0.001)
  plt.draw()
  
  #rospy.loginfo(len(msg.ranges))
rospy.init_node('topic_subscriber')
rate=rospy.Rate(20)
msg=LaserScan()
msg.ranges=np.arange(0,720,)
plt.show(block=False)  
sub=rospy.Subscriber('/bot_0/laser/scan',LaserScan,callback)
rospy.spin()

 