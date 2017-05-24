#!/usr/bin/env python
import rospy
from std_srvs.srv import Empty,EmptyResponse,EmptyRequest
from geometry_msgs.msg import 

def callback(request):
    val=Twist()
    val.linear.x=0.5
    

    print "Done!"
    return EmptyResponse()

rospy.init_node('service_client')
my_service=rospy.Service('/my_service',Empty,callback)
rospy.spin()