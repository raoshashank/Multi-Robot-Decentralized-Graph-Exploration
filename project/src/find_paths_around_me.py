#! /usr/bin/env python
import rospy
import numpy as np 
from std_msgs.msg import Int32
import matplotlib.pyplot as plt
from gazebo_msgs.msg import LinkStates
from sensor_msgs.msg import LaserScan
def callback(msg):
  #rate.sleep()
  plt.plot (np.arange(-135,135,0.375),msg.ranges)
  plt.pause(0.001)
  plt.draw()

rospy.init_node('topic_subscriber')
rate=rospy.Rate(100)
msg=LaserScan()
msg.ranges=np.arange(-135,135,0.375)
plt.show(block=False)  
sub=rospy.Subscriber('/scan',LaserScan,callback)

rospy.spin()

 