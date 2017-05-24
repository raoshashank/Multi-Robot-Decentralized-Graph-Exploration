#!/usr/bin/env python
import rospy
from my_custom_srv_msg.srv import custom_srv_msg,custom_srv_msgResponse
from geometry_msgs.msg import  Twist,Point,Quaternion
from math import radians,pi
from nav_msgs.msg import Odometry 

def go_straight(dist):
    linear_speed=dist/2
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
    print 'request recieved: radius:'+str(request.radius)+'reps:'+str(request.repetitions)
    try:
        for i in range(4*request.repetitions):
                go_straight(request.radius)
                turn(45)
        custom_srv_msgResponse=True
    except rospy.ROSException():
        custom_srv_msgResponse=False    
    return custom_srv_msgResponse


rospy.init_node('service',anonymous=False)
angular_speed=pi/4
square_side=1.0
square_angle=radians(90)
rate=rospy.Rate(5)
pub=rospy.Publisher('/cmd_vel_mux/input/teleop',Twist,queue_size=10)
while not rospy.is_shutdown():
    rospy.Service('/custom_service',custom_srv_msg,callback)
    rospy.spin()
