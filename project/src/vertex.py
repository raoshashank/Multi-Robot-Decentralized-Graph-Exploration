#!/usr/bin/env python
import numpy
import rospy
class vertex:
   vertex_tag=str()
   x = 0
   y = 0
   In=[]
   In_1=[]
   beacon=False
   n=0

   def __init__(self,x,y,vertex_tag,In,n):
       self.n=n
       self.x=x
       self.y=y
       self.vertex_tag=vertex_tag
       self.In_1=self.In
       self.In=In
       
    
     



