#!/usr/bin/env python
import rospy
from my_custom_srv_msg.srv import custom_srv_msg,custom_srv_msgRequest,custom_srv_msgResponse
rospy.init_node('custom_service_client')
rospy.wait_for_service('/custom_service')
servcaller=rospy.ServiceProxy('/custom_service',custom_srv_msg)
params=custom_srv_msgRequest()
params.radius=5
params.repetitions=10
result=servcaller(params)
print result.success
