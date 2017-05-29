#!/usr/bin/env python
import rospy
from project.srv import dirturn,dirturnRequest,dirturnResponse
from math import pi,atan2
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import pi,asin,sin


def turn():
    global flag,feedback,heading_cmd,heading,q,cmd ,done,angle,initial_heading,heading_error
    q=[0,0,0,0]
    cmd=Twist()
    q[0]=feedback.pose.pose.orientation.w
    q[1]=feedback.pose.pose.orientation.x
    q[2]=feedback.pose.pose.orientation.y
    q[3]=feedback.pose.pose.orientation.z  
    heading=atan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]*q[2]+q[3]*q[3]))
    
    if flag==0:
        flag=1
        initial_heading=heading
        heading_cmd=initial_heading+angle
    
    
    heading_error=heading_cmd-heading
    rospy.loginfo(heading_error)

    if heading_error>pi:
        heading_error=heading_error-2*pi
    if heading_error<=-pi:
        heading_error=heading_error+2*pi
    
    
    if abs(heading_error) < 0.001:
        rospy.loginfo("Turn done!")
        done=1
    else:
        cmd.angular.z=-0.8*heading_error
        pub.publish(cmd)
        rospy.loginfo(heading_error)

def callback(msg):
    global feedback
    feedback=msg
    
def service_callback(request):
    global angle,done,flag
    
    angle=request.angle

    while  done==0:
        turn()
        
    done=0     
    flag=0
    return dirturnResponse()

flag=0
heading_cmd=0
q=[]
heading=0
heading_error=0
feedback=Odometry()
initial_heading=0
done=0
angle=0
rospy.init_node('service_client')
rate=rospy.Rate(20)
pub=rospy.Publisher('/bot_0/cmd_vel',Twist,queue_size=1)
sub=rospy.Subscriber('/bot_0/odom',Odometry,callback)
while not rospy.is_shutdown():
    my_service=rospy.Service('/turn_service_server',dirturn,service_callback)
    rospy.spin()