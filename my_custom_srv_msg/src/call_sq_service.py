#!/usr/bin/env python
import rospy
from std_srvs.srv import EmptyRequest,Empty
rospy.init_node("service_client", anonymous=True)
rospy.wait_for_service('/Square_service')
serv=rospy.ServiceProxy('/Square_service',Empty)
kk=EmptyRequest()
result=serv(kk)
