#!/usr/bin/env python
import rospy
from math import pi
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
##try closed loop control by checking odometry information 
def callback(msg):
    current_angle=msg.pose.pose.orientation.z
    while current_angle



rospy.init_node('test_node')
rate=rospy.Rate(20)
pub=rospy.Publisher('/cmd_vel',Twist,queue_size=1)
sub=rospy.Subscriber('/odometry/filtered',Odometry,callback)
cmd=Twist()
cmd.angular.z=pi/5

for i in range(50):
    pub.publish(cmd)
    rospy.loginfo("Turning")
    rate.sleep()
    
cmd=Twist()
pub.publish(cmd)
