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
#from project import vertex
from project.msg import vertex_info,vertices
from collections import deque
    
"""
Steps:
2.@node,check if coordinates correspond to vertex position of any vertex in the array
    2a.if yes then extract the vertex and go to 3
    2b.else find edges connected to vertex(basically angles of edges connected) 
       initialise a vertex with current coordinates and go to 3

3.run merge_matrix algorithm
4.run Order_matrix algorithm
5.Select last edge of In() as next edge for traversal


""" 
def second_step_on_vertex_visit(I_R):
    global E1cap,E2cap
    c_q=deque()
    if op.non_zero_element_count(I_R[:,E1cap])


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
    

def check_for_vertex_in_array(v_x,v_y):
    global vertex_array
    v_found=vertex_info()
    v_found.tag=""
    tolerance=0.1
    distance=0.5    
    for v in vertex_array:
        err=sqrt((v_x-v.x)**2+(v_y-v.y)**2)
        rospy.loginfo(err)
        if  err<distance:
            v_found=v
    rospy.loginfo("found with coordinates :"+v_found.tag)
    return v_found

def check_for_vertex_in_array_tag(tag):
    global vertex_array
    v_found=vertex_info()
    v_found.tag=""
    for v in vertex_array:
        if v.tag==tag:
            v_found=v
    rospy.loginfo("found with tag:"+v_found.tag)
    return v_found




##TO DO
def initialize_vertex_I():
    global heading
    I=[]
    err=0.1
    ##Angles to be checked 0(=2pi),pi/2,-pi(=pi),-pi/2(3pi/2) in order
    ##Orient in pi/2 direction and check left and right
    ##Assume order of checking doesnt matter
    ##HOW THE F  DO I DO THIS SHIT!!
        
    I.append(heading)
    orient_to_heading(pi/2)

    

    return [0,0,0,0]

#TO DO
def get_vertex_tag(tag_arr,i):
    return return tag_arr[i]

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
                orient_to_heading(pi/2)
                rospy.loginfo("I'M AT NODE!")        
                v_found=vertex_info()
                v_x=odom_feedback.pose.pose.position.x
                v_y=odom_feedback.pose.pose.position.y
                v_found=check_for_vertex_in_array(v_x,v_y)
                if v_found.tag=="":
                    rospy.loginfo("Found new node!!")
                    v_found.tag='x'+str(v_x)+'y'+str(v_y)
                    v_found.x=v_x
                    v_found.y=v_y
                    v_found.inci.I=initialize_vertex_I()
                    rospy.loginfo("Vertex with tag "+v_found.tag+"Found")
                else:
                    rospy.loginfo("I'm at node with tag:"+v_found.tag)
                ##Node identify and initialize done
                ##Do first step at vertex visit
                #do second step at vertex visit
                #Doubt:Is I' by any chance the last row of In-1(Rk)
                E1cap=I.shape[1]
                E2cap=v_found.inci.I.shape[1]
                Vcap=I.shape[0]+v_found.inci.I.shape[0]
                I=op.first_step_on_vertex_visit(I,v_found.inci.I,I[:,I.shape[1]-1])
                v_found.inci.I=I
                #Do second step on vertex visit
                second_step_on_vertex_visit(I)    
                
            
            
            else:
                go_forward()
                
"""           

                  exists=0
                v_x=vertex_x_from_ekf
                v_y=vertex_y_from_ekf
                
                ##Check vertex has beacon or not
                for i in vertices:
                    if i.x>v_x+delta and i.x<v_x-delta and i.y>v_y+delta and i.y<v_y-delta:
                        exists=1
                        v=i
                        break
                

                    

                if exists!=1:
                    #create new vertex
                    tag = generate_random_tag()
                    In=initialize() #Initialize incidence matrix
                    v = vertex(v_x,v_y,tag,In,len(vertices)+1)
                
                ###First Step on vertex visit
                I_dash=matrix_operations.completed(In)
                I_2_dash=matrix_operations.merge_matrix(I_dash,I)
                I_temp=matrix_operations.merge_matrix(I_2_dash,v.In)
                I_temp=matrix_operations.order_matrix(I_temp)
                I=I_temp
                v.I=I_temp
"""



        
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
    rospy.wait_for_service('/direction_service_server')
    servcaller=rospy.ServiceProxy('/direction_service_server',new_direction)        
    
    #Service2 for turning in the decided direction
    rospy.wait_for_service('/turn_service_server')
    servcaller2=rospy.ServiceProxy('/turn_service_server',dirturn)
    
    sub_odom=rospy.Subscriber('/bot_0/odom',Odometry,odom_callback)
    pub=rospy.Publisher('/bot_0/cmd_vel',Twist,queue_size=1)
    
    sub=rospy.Subscriber('/bot_0/laser/scan',LaserScan,laser_callback)
    
    sub_vertex=rospy.Subscriber('/vertices',vertices,vertices_callback)
    pub_vertices=rospy.Publisher('/vertices',vertices,queue_size=1)

    main()
    rospy.spin()
