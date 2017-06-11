#! /usr/bin/env python
from project.msg import vertices,vertex_info
import rospy
from std_msgs.msg import Int32
import numpy as np
rospy.init_node('simple_publisher')
pub = rospy.Publisher('/vertices', vertices, queue_size=1)
rate = rospy.Rate(2)
empty=[]
temp=vertices()
temp.v=empty
while not rospy.is_shutdown(): 
  pub.publish(temp)
  rate.sleep()