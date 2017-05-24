#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import LinkStates
from vertex import vertex
from robot import robot
from math import sqrt
"""
def edge_traverse_function():
    while !Exploration_done
        if robot position  == node poisition
            if node_position != beacon_position 
                add node position to beacon position list(coordinates)
                call constructor to initialse beacon matrix
            Merge_Matrices (Algorithm 1)
            Order_Matrix   (Algorithm 2)
            Identify edges connected to vertex
            Select edge to traverse (ALgorithm 4)
            edge_traverse_function()
        else keep going in same direction without orientation change    

"""

##find an acceptable value
distance_to_vertex=0

def dst_function(v,x2,y2):
    return sqrt((v.x-x2)^2+(v.y-y2)^2)

def check_if_at_vertex(x,y):
    flag=0
    result=[]
    for v in vertexes:
        if dst_function(v,x,y)<distance_to_vertex:
          flag=1
          return [True,v]
    if not flag==1:
        return [false,0]
    


v1=vertex(3.88,-3.80)
v2=vertex(1.0277,-3.80)
v3=vertex(-2.63,-3.84)
v4=vertex(4.87,-1.701)
v5=vertex(4.87,0.622)
v6=vertex(4.86,2.47)
v7=vertex(1.03,0.655)
v8=vertex(-2.33,0.64)
v9=vertex(-2.38,-1.67)
v10=vertex(-2.37,2.69)
v11=vertex(-5.26,0.64)
v12=vertex(-5.29,-1.69)
v13=vertex(-5.27,2.53)
vertexes=[v1,v2,v3,v4,v5,v6,v7,v8,v9,v10,v11,v12,v13]













