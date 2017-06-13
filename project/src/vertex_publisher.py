#! /usr/bin/env python
from project.msg import vertices,vertex_info
import rospy
from std_msgs.msg import Int32
import numpy as np
rospy.init_node('vertex_publisher')
pub = rospy.Publisher('/vertices', vertices, queue_size=1)
rate = rospy.Rate(2)
temp=vertices()
temp.v=[]
v1=vertex_info()
v1.x=-4.5
v1.y=-5.2
v1.tag='v1'
v1.I=[]
v1.pos_in_I=0
temp.v.append(v1)
v1=vertex_info()
v1.x=-1.6
v1.y=-5.2
v1.tag='v2'
v1.I=[]
v1.pos_in_I=1
temp.v.append(v1)
v1=vertex_info()
v1.x=3.5
v1.y=-5.2
v1.tag='v3'
v1.I=[]
v1.pos_in_I=-1
temp.v.append(v1)
v1=vertex_info()

v1.x=-1.6
v1.y=1
v1.tag='v4'
v1.I=[]
v1.pos_in_I=2
temp.v.append(v1)
v1=vertex_info()

v1.x=-1.6
v1.y=5.2
v1.tag='v5'
v1.I=[]
v1.pos_in_I=3
temp.v.append(v1)
v1=vertex_info()

v1.x=-7.6
v1.y=1
v1.tag='v6'
v1.I=[]
v1.pos_in_I=4
temp.v.append(v1)
v1=vertex_info()

v1.x=-7.6
v1.y=5.2
v1.tag='v7'
v1.I=[]
v1.pos_in_I=5
temp.v.append(v1)
v1=vertex_info()

v1.x=-7.6
v1.y=-3.5
v1.tag='v8'
v1.I=[]
v1.pos_in_I=6
temp.v.append(v1)
v1=vertex_info()

v1.x=7.6
v1.y=1
v1.tag='v9'
v1.I=[]
v1.pos_in_I=-1
temp.v.append(v1)
v1=vertex_info()

v1.x=7.6
v1.y=5.2
v1.tag='v10'
v1.I=[]
v1.pos_in_I=-1
temp.v.append(v1)
v1=vertex_info()

v1.x=7.6
v1.y=-5.4
v1.tag='v11'
v1.I=[]
v1.pos_in_I=-1
temp.v.append(v1)
v1=vertex_info()


while not rospy.is_shutdown(): 
  pub.publish(temp)
  rate.sleep()