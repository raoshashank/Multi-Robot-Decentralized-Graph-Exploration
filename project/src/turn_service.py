#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from math import pi
from project.srv import dirturn,dirturnRequest,dirturnResponse

###Need to accurately turn by 90deg
def go_forward():
    cmd=Twist()
    cmd.linear.x=linear_velocity_x
    pub.publish(cmd)
    rate.sleep()
    cmd=Twist()
    pub.publish(cmd)

def callback(request):
    dire=request.dir
    cmd=Twist()
    response=dirturnResponse()
    ##Turn around
    if dire==2:
        cmd.angular.z=angular_velocity_z
        for i in range(50):
             pub.publish(cmd)
             rate.sleep()
        cmd=Twist()
        pub.publish(cmd)
        response=dirturnResponse()
        response.success=True
        return response
    
    elif dire==1: ##left turn
        cmd.angular.z=angular_velocity_z
    elif dire==0:##Right turn
        cmd.angular.z=-1*angular_velocity_z
    rospy.loginfo("Turn commence")
    for i in range(13):
        pub.publish(cmd)
        rate.sleep()
    cmd=Twist()
    pub.publish(cmd)
    rospy.loginfo("Turn end")
    for i in range(15):
        go_forward()
    response=dirturnResponse()
    response.success=True
    return response


rospy.init_node('turn_service_node')
angular_velocity_z=pi/4
linear_velocity_x=0.4
rate=rospy.Rate(5)
pub=rospy.Publisher('/cmd_vel',Twist,queue_size=1)
while not rospy.is_shutdown():
    rospy.Service("/turn_service_server",dirturn,callback)
    rospy.spin()