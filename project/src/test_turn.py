#!/usr/bin/env python
import rospy
from math import pi
from geometry_msgs.msg import Twist
rospy.init_node('test_node')
rate=rospy.Rate(20)
pub=rospy.Publisher('/cmd_vel',Twist,queue_size=1)
cmd=Twist()
cmd.angular.z=pi/5
for i in range(50):
    pub.publish(cmd)
    rospy.loginfo("Turning")
    rate.sleep()
    
cmd=Twist()
pub.publish(cmd)
