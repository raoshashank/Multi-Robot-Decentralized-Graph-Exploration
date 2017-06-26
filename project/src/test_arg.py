import numpy as np
import rospy
from sys import argv
from project.msg import vertices,vertex_info

def callback(msg):
    rospy.loginfo(msg)


rospy.init_node('sub')
rospy.Subscriber("/vertices", vertices, callback)
rospy.spin()