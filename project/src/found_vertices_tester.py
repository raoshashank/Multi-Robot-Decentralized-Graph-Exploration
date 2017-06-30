#!/usr/bin/env python
import rospy
from project.msg import vertices,vertex_info

rospy.init_node("vertices_tester")
rate=rospy.Rate(20)
temp=vertices()
v1=vertex_info()
v1.tag="x1y1"
v2=vertex_info()
v2.tag="x2y2"
v3=vertex_info()
v3.tag="x3y3"
v4=vertex_info()
v4.tag="x4y4"
v5=vertex_info()
v5.tag="x5y5"

pub=rospy.Publisher("/vertices",vertices,latch=True,queue_size=10)
temp.v.append(v1)
temp.v.append(v2)
pub.publish(temp)

