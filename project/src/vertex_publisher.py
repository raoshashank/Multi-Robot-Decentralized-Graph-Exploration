#! /usr/bin/env python
import rospy
from project.msg import vertices,vertex_info
from std_msgs.msg import Int32
from matrix_op import matrix_op
import numpy as np
import pickle

#def callback(msg):
#    rospy.loginfo(len(msg.v))

  

rospy.init_node('vertex_publisher')

if __name__=='__main__': 
  rate=rospy.Rate(20)
  pub=rospy.Publisher( '/vertices',vertices,latch=True,queue_size=10)
 # sub=rospy.Subscriber('/vertices',vertices,callback)
  rospy.loginfo("log info")
  rospy.logdebug("log debug ")
 # rospy.spin()   






"""
temp.v=[]
v1=vertex_info()
v1.x=-4.5
v1.y=-5.2
v1.tag='v1'
v1.inci.I=[0,0,0,0]
v1.inci.tags_array=[]
temp.v.append(v1)
v1=vertex_info()
v1.x=-1.6
v1.y=-5.2
v1.tag='v2'
v1.inci.I=[0,0,0,0]
v1.inci.tags_array=[]
temp.v.append(v1)
v1=vertex_info()
v1.x=3.5
v1.y=-5.2
v1.tag='v3'
v1.inci.I=[0,0,0,0]
v1.inci.tags_array=[]
temp.v.append(v1)
v1=vertex_info()

v1.x=-1.6
v1.y=1
v1.tag='v4'
v1.inci.I=[0,0,0,0]
v1.inci.tags_array=[]
temp.v.append(v1)
v1=vertex_info()

v1.x=-1.6
v1.y=5.2
v1.tag='v5'
v1.inci.I=[0,0,0,0]
v1.inci.tags_array=[]
temp.v.append(v1)
v1=vertex_info()

v1.x=-7.6
v1.y=1
v1.tag='v6'
v1.inci.I=[0,0,0,0]
v1.inci.tags_array=[]
temp.v.append(v1)
v1=vertex_info()

v1.x=-7.6
v1.y=5.2
v1.tag='v7'
v1.inci.I=[0,0,0,0]
v1.inci.tags_array=[]
temp.v.append(v1)
v1=vertex_info()

v1.x=-7.6
v1.y=-3.5
v1.tag='v8'
v1.inci.I=[0,0,0,0]
v1.inci.tags_array=[]
temp.v.append(v1)
v1=vertex_info()

v1.x=7.6
v1.y=1
v1.tag='v9'
v1.inci.I=[0,0,0,0]
v1.inci.tags_array=[]
temp.v.append(v1)
v1=vertex_info()

v1.x=7.6
v1.y=5.2
v1.tag='v10'
v1.inci.I=[0,0,0,0]
v1.inci.tags_array=[]
temp.v.append(v1)
v1=vertex_info()

v1.x=7.6
v1.y=-5.4
v1.tag='v11'
v1.inci.I=[0,0,0,0]
v1.inci.tags_array=[]
temp.v.append(v1)
v1=vertex_info()

"""
