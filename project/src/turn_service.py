#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from math import pi
from project.srv import dirturn,dirturnRequest,dirturnResponse
def callback(request):
    dire=request.dir
    cmd=Twist()
    if dire==1: ##left turn
        cmd.angular.z=angular_velocity_z
    else:
        cmd.angular.z=-1*angular_velocity_z
    rospy.loginfo("Turn commence")
    for i in range(25):
        pub.publish(cmd)
        rate.sleep()
    cmd=Twist()
    pub.publish(cmd)
    rospy.loginfo("Turn end")
    response=dirturnResponse()
    response.success=True
    return response


rospy.init_node('turn_service')
angular_velocity_z=pi/4
rate=rospy.Rate(10)
pub=rospy.Publisher('/cmd_vel',Twist,queue_size=1)
while not rospy.is_shutdown():
    rospy.Service("/turn_service_server",dirturn,callback)
    rospy.spin()