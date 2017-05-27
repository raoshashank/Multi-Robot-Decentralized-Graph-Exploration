#!/usr/bin/env python
import rospy
from math import pi,atan2
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import pi,asin,sin
from std_srvs.srv import EmptyRequest,Empty,EmptyResponse

##try closed loop control by checking odometry information 
flag=0
heading_cmd=0
q=[]
heading=0
heading_error=0
feedback=Odometry()

###using quaternion info:
def turn():
    global flag,feedback,heading_cmd,heading,q,cmd 
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
        heading_cmd=initial_heading+pi
    
    
    heading_error=heading_cmd-heading
    if heading_error>pi:
        heading_error=heading_error-2*pi
    if heading_error<=-pi:
        heading_error=heading_error+2*pi
    
    cmd.angular.z=-0.8*heading_error
    pub.publish(cmd)

    if heading_error < 0.001:
        rospy.loginfo("Turn done!")


def callback(msg):
    global feedback
    feedback=msg
    
        

def service_callback(request):
    global heading_error
    global cmd,pub
    turn()
    if heading_error<0.001:
        cmd.linear.x=0
        cmd.angular.z=0
        pub.publish(cmd)
        return EmptyResponse


rospy.init_node('test_node')
rate=rospy.Rate(20)
pub=rospy.Publisher('/bot_0/cmd_vel',Twist,queue_size=1)
heading_error=0
sub=rospy.Subscriber('/bot_0/odom',Odometry,callback)
while not rospy.is_shutdown():  
    rospy.Service('/test_turn_service',Empty,service_callback)
    rospy.spin()


