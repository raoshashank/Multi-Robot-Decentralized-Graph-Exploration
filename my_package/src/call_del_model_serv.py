#!/usr/bin/env python
import rospy
from gazebo_msgs.srv import DeleteModel,DeleteModelRequest
import sys
import rospkg
rospack=rospkg.RosPack()
traj=rospack.get
rospy.init_node('service_client')
rospy.wait_for_service('/gazebo/delete_model')
delete_model_service=rospy.ServiceProxy('/gazebo/delete_model',DeleteModel)
kk=DeleteModelRequest()
kk.model_name="unit_cylinder_1"
result=delete_model_service(kk)
print result