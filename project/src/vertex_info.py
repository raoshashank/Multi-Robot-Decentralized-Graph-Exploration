#!/usr/bin/env python
import rospy
from project.msg import vertex_info,vertices
import numpy as np
from rospy.numpy_msg import numpy_msg
import pickle
rospy.init_node('test2')
v1=vertex_info()
v1.x=20
v1.y=20
I=np.array([[v1,0,0,1],[v1,0,0,2],[v1,1,1,1]])
##use pickle to store and publish multi dimensional array
s=pickle.dumps(I)
v1.I=s
##to retrieve I,use I= pickle.loads(v.I)
v_ar=vertices()
v_ar.v=[]
v_ar.v.append(v1)
pub=rospy.Publisher('/vertices',vertices,queue_size=10)
rate=rospy.Rate(20)
rate.sleep()
rate.sleep()
pub.publish(v_ar)








