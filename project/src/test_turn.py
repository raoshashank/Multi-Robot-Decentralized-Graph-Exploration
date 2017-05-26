#!/usr/bin/env python
import rospy
from math import pi,atan2
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import pi,asin,sin
from std_srvs.srv import EmptyRequest,Empty,EmptyResponse

##try closed loop control by checking odometry information 
flag=0
#initial_angle =0
heading_cmd=0
q=[]
heading=0
#final_angle_lower=0
#final_angle_upper=0
"""
def callback(msg):
    global flag
    global final_angle_lower
    global final_angle_upper
    global initial_angle
    if flag==0:
        ##set initial position
        initial_angle=2*asin(msg.pose.pose.orientation.z)
        rospy.loginfo("initial pose:"+str(initial_angle))
        flag=1
        final_angle_lower=initial_angle+(pi/2)+deltarate
        rospy.loginfo(final_angle_lower)
       # final_orientation_lower=sin(final_angle_lower/2)
        final_angle_upper=initial_angle+(pi/2)-delta
        rospy.loginfo(final_angle_upper)
        #final_orientation_upper=(final_angle_upper/2)
    ori=2*asin(msg.pose.pose.orientation.z)
        
    if ori>final_angle_lower or ori<final_angle_upper:
        cmd.angular.z=0
        pub.publish(cmd)
        rospy.loginfo("done!")
        return 0
    else:
        rospy.loginfo(ori)
        pub.publish(cmd)
        rate.sleep()
        
    #rospy.loginfo("orientation now:"+str(2*asin(msg.pose.pose.orientation.z)))
 """
###using quaternion info:
def callback(msg):
    global flag
    global heading_cmd
    global heading
    global q
    global cmd
    q=[0,0,0,0]
    cmd=Twist()
    q[0]=msg.pose.pose.orientation.w
    q[1]=msg.pose.pose.orientation.x
    q[2]=msg.pose.pose.orientation.y
    q[3]=msg.pose.pose.orientation.z  
    heading=atan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]*q[2]+q[3]*q[3]))
    
    if flag==0:
        flag=1
        initial_heading=heading
        heading_cmd=initial_heading-(pi/2)
    
    
    heading_error=heading_cmd-heading
    if heading_error>pi:
        heading_error=heading_error-2*pi
    if heading_error<=-pi:
        heading_error=heading_error+2*pi
    
    cmd.angular.z=-0.8*heading_error
    pub.publish(cmd)

    if heading_error < 0.001:
        rospy.loginfo("Turn done!")
        return EmptyResponse



rospy.init_node('test_node')
rate=rospy.Rate(20)
pub=rospy.Publisher('/bot_0/cmd_vel',Twist,queue_size=1)
#cmd=Twist()
#cmd.angular.z=pi/10
#delta=0.002
sub=rospy.Subscriber('/bot_0/odom',Odometry,callback)
rospy.spin()


