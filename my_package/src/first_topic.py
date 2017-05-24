#! /usr/bin/env python

import rospy                               
from std_msgs.msg import Int32    
from geometry_msgs.msg import Twist        

rospy.init_node('topic_publisher')
pub=rospy.Publisher('cmd_vel_mux/input/teleop',Twist,queue_size=10)                     
var = Twist()
var.linear.x=0.0  
var.linear.y=0.0
var.linear.z=0.0
var.angular.x=0.0
var.angular.y=0.0
var.angular.z=1.5708
rate=rospy.Rate(2)                                             
while not rospy.is_shutdown():
    pub.publish(var)    
    rate.sleep()
                                           
                                        
