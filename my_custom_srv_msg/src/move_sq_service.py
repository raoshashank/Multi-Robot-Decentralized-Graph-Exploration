#!/usr/bin/env python
import rospy
from std_srvs.srv import Empty,EmptyRequest,EmptyResponse
from geometry_msgs.msg import  Twist,Point,Quaternion
from math import radians,pi
from nav_msgs.msg import Odometry 

def go_straight(dist):
    move_cmd=Twist()
    move_cmd.linear.x=linear_speed
    rospy.loginfo("Going Straight")
    for i in range(10):
        pub.publish(move_cmd)
        rate.sleep()
    move_cmd=Twist()
    pub.publish(move_cmd)

def turn(deg):
    deg=radians(deg)
    move_cmd=Twist()
    move_cmd.angular.z=deg
    rospy.loginfo("Turning")
    for i in range(10):
        pub.publish(move_cmd)
        rate.sleep()
    move_cmd=Twist()
    pub.publish(move_cmd)



def callback(request):
    while not rospy.is_shutdown():
        for i in range(4):
            go_straight(square_side)
            turn(45)  
    print "done"
    return EmptyResponse()



rospy.init_node('service',anonymous=False)
linear_speed=0.2
angular_speed=pi/4
square_side=1.0
square_angle=radians(90)
rate=rospy.Rate(5)
pub=rospy.Publisher('/cmd_vel_mux/input/teleop',Twist,queue_size=10)
my_service=rospy.Service('/Square_service',Empty,callback)
rospy.spin()
