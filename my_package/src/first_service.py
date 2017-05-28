#!/usr/bin/env python
import rospy
from std_srvs.srv import Empty,EmptyResponse,EmptyRequest
from math import pi,atan2
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import pi,asin,sin
flag=0
heading_cmd=0
q=[]
heading=0
heading_error=0
feedback=Odometry()
done=0
angle=0
def turn():
    global flag,feedback,heading_cmd,heading,q,cmd ,done
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
    
    
    if heading_error < 0.001:
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
    global angle
    request.angle=angle

    while  done==0:
        turn()
        
         
        
    return EmptyResponse()

rospy.init_node('service_client')
rate=rospy.Rate(20)
pub=rospy.Publisher('/bot_0/cmd_vel',Twist,queue_size=1)
heading_error=0
sub=rospy.Subscriber('/bot_0/odom',Odometry,callback)
my_service=rospy.Service('/my_service',Empty,service_callback)
rospy.spin()