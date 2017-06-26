#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from math import pi,atan2,cos,sqrt,pow
from random import randint
import numpy as np
from nav_msgs.msg import Odometry
from matrix_op import matrix_op
from project.msg import vertex_info,vertices,incidence
from collections import deque
import networkx as nx   
import pickle


        
def orient_to_heading(dir):
    global flag,odom_feedback,heading_cmd,heading,q,cmd ,done,angle,initial_heading,heading_error
    q=[0,0,0,0]
    done=0
    heading_cmd=dir
    while done!=1 and not rospy.is_shutdown():
        cmd=Twist()
        q[0]=odom_feedback.pose.pose.orientation.w
        q[1]=odom_feedback.pose.pose.orientation.x
        q[2]=odom_feedback.pose.pose.orientation.y
        q[3]=odom_feedback.pose.pose.orientation.z  
        rospy.loginfo(q)
        heading=atan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]*q[2]+q[3]*q[3]))
        heading_error=heading_cmd-heading
        rospy.loginfo(heading)

        if heading_error>pi:
            heading_error=heading_error-2*pi
        if heading_error<=-pi:
            heading_error=heading_error+2*pi
    
    
        if abs(heading_error) < 0.001:
            done=1
        else:
            cmd.angular.z=-0.8*heading_error
            pub.publish(cmd)


def odom_callback(msg):
    global odom_feedback
    odom_feedback=msg

def vertices_callback(msg):
    global vertex_array
    vertex_array=msg.v



def laser_callback(msg):
    global check,mid_avg,data
    data=msg.ranges
    
    ###check in 3 directions for the free space
    ###slice the ranges array into 3 regions for 3 directions:;left,forward and right
    
    #left_slice=np.asarray(data[667:720])
    #left_avg=left_slice.sum()/len(left_slice)
    
    
    #mid_slice=np.asarray(data[355:365])
    #mid_avg=mid_slice.sum()/len(mid_slice)
    
    #right_slice=np.asarray(data[0:53])
    #right_avg=right_slice.sum()/len(right_slice)
    
    #check=[left_avg,mid_avg,right_avg]
    check=[data[719],data[360],data[0]]
    #rospy.loginfo("Values i found are:"+str(check))    
    



def main():
    orient_to_heading(2*pi)


        
if  __name__ == "__main__":
    rospy.init_node('random_mover',anonymous=False) 
    ###########Global Variables############
    bot_no=0
    q=[0,0,0,0]
    flag=0
    node_found=0
    initial_heading=0.0 
    heading_error=0.0
    heading=0.0  
    check=[]
    interval_for_angle_measurement=10
    linear_velocity_x=0.1
    
    angular_velocity_z=0
    
    rate=rospy.Rate(5)
    

    previous_vertex=vertex_info()
    next_vertex=vertex_info()
    current_v=vertex_info()
    traverse_q=deque()
    
    odom=Odometry()
    data=LaserScan()
    odom_feedback=Odometry()
    cmd=Twist()
    op=matrix_op()
    traverse_q=deque()

    mid_avg=0
    range_thresh=2
    lane_width=2
    vertex_array=[]
    I_R=np.array([[]])
    E1cap=0
    E2cap=0
    Vcap=0
    
    ###########Global Variables############
   
    sub_odom=rospy.Subscriber('/bot_0/odom',Odometry,odom_callback)
    pub=rospy.Publisher('/bot_0/cmd_vel',Twist,queue_size=1)
    
    sub=rospy.Subscriber('/bot_0/laser/scan',LaserScan,laser_callback)
    
    sub_vertex=rospy.Subscriber('/vertices',vertices,vertices_callback)
    pub_vertices=rospy.Publisher('/vertices',vertices,queue_size=1)

    main()
    rospy.spin()
