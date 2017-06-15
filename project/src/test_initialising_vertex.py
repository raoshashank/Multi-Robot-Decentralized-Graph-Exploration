#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from math import pi,atan2,cos,sqrt,pow
from random import randint
import numpy as np
from nav_msgs.msg import Odometry
from project.srv import direction,directionRequest,directionResponse,dirturn,dirturnRequest,dirturnResponse,new_direction,new_directionRequest,new_directionResponse 
from matrix_op import matrix_op
from project.msg import vertex_info,vertices
from collections import deque
    

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
        heading=atan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]*q[2]+q[3]*q[3]))
        heading_error=heading_cmd-heading

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

def forward_by_half_lane_width():
    global data,check,odom_feedback,lane_width,pub

    x_start=odom_feedback.pose.pose.position.x
    y_start=odom_feedback.pose.pose.position.y
    delta=0.05
    error=0
    goal_dst=lane_width/2+delta
    dst=0
    while dst<=goal_dst:
       go_forward()
       dst=sqrt((odom_feedback.pose.pose.position.x-x_start)**2+(odom_feedback.pose.pose.position.y-y_start)**2)

    cmd=Twist()
    pub.publish(cmd)
    rospy.loginfo("Escaped")

    



def go_forward():
    global q,cmd,odom_feedback,flag,rate,heading,initial_heading,heading_error,angular_velocity_z,linear_velocity_x
    q[0]=odom_feedback.pose.pose.orientation.w
    q[1]=odom_feedback.pose.pose.orientation.x
    q[2]=odom_feedback.pose.pose.orientation.y
    q[3]=odom_feedback.pose.pose.orientation.z  
    heading=atan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]*q[2]+q[3]*q[3]))
    if flag==0:
        initial_heading=heading
        flag=1
    heading_error=initial_heading-heading
    if heading_error>pi:
        heading_error=heading_error-2*pi
    if heading_error<=-pi:
        heading_error=heading_error+2*pi
    angular_velocity_z=-0.9*heading_error
    cmd=Twist()
    cmd.linear.x=linear_velocity_x
    cmd.angular.z=angular_velocity_z
    pub.publish(cmd)



def odom_callback(msg):
    global odom_feedback
    odom_feedback=msg

# def vertices_callback(msg):
#     global vertex_array
#     vertex_array=msg.v
def laser_callback(msg):
    global check,mid_avg,data
    data=msg.ranges
    check=[data[719],data[360],data[0]]
    




def initialize_vertex_I():
    global heading,range_thresh
    I=[]
    err=0.1
    orient_to_heading(pi/2)
    if data[0]>range_thresh:
        I.append(2*pi)
    if data[360]>range_thresh:
        I.append(pi/2)
    if data[719]>range_thresh:
        I.append(pi)
    orient_to_heading(0)
    if data[0]>range_thresh:
        I.append(3*pi/2)
    orient_to_heading(pi/2)
    rospy.loginfo(I)
    
    return I


def main():
    global cmd,data,flag,servcaller,servcaller2,node_found,check,mid_avg,params,params2,heading,odom_feedback,vertex_array
    global E1cap,E2cap,Vcap
    #global vertices
    while not rospy.is_shutdown():
        if check!=[]:
            count=0

            for i in check:
                if i>range_thresh:
                    count+=1

            if count>=2 or (count==0 and data[360]<lane_width/2):
                if count>=2:
                    forward_by_half_lane_width()
                rospy.loginfo("I'M AT NODE!")        
                v_found=vertex_info()
                v_x=odom_feedback.pose.pose.position.x
                v_y=odom_feedback.pose.pose.position.y
                rospy.loginfo("Found new node!!")
                v_found.tag='x'+str(v_x)+'y'+str(v_y)
                v_found.x=v_x
                v_found.y=v_y
                v_found.inci.I=initialize_vertex_I() 
                rospy.sleep(20)
            else:
                go_forward()
                
      
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
    
    params=directionRequest()
    params2=dirturnRequest()
    
    rate=rospy.Rate(5)
    
    odom=Odometry()
    data=LaserScan()
    odom_feedback=Odometry()
    cmd=Twist()
    op=matrix_op()
    
    mid_avg=0
    range_thresh=2
    lane_width=2
    vertex_array=[]
    I=[]
    E1cap=0
    E2cap=0
    Vcap=0
    ###########Global Variables############

    ##Service1 for deciding direction
    # rospy.wait_for_service('/direction_service_server')
    # servcaller=rospy.ServiceProxy('/direction_service_server',new_direction)        
    
    #Service2 for turning in the decided direction
    #rospy.wait_for_service('/turn_service_server')
    #servcaller2=rospy.ServiceProxy('/turn_service_server',dirturn)
    
    sub_odom=rospy.Subscriber('/bot_0/odom',Odometry,odom_callback)
    pub=rospy.Publisher('/bot_0/cmd_vel',Twist,queue_size=1)
    
    sub=rospy.Subscriber('/bot_0/laser/scan',LaserScan,laser_callback)
    
    # sub_vertex=rospy.Subscriber('/vertices',vertices,vertices_callback)
    # pub_vertices=rospy.Publisher('/vertices',vertices,queue_size=1)

    main()
    rospy.spin()
