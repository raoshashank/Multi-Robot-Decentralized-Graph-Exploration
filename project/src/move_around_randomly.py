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
"""
def escape_turn(initial):
    global data,check,feedback
    ##find first local minimas in + & - directions from 0
    for i range(1,360):
        if data[i]>data[i-1] and data[i]>data[i+1]:
            right_minima=data[i]
            ir=i
    for i range(360,720):
        if data[i]>data[i-1] and data[i]>data[i+1]:
            left_minima=data[i]
            il=i
    
    dist=left_minima*cos(il*0.00655)
    
    while feedback.pose.pose.position.x>(initial+dist):
            go_forward()
        
    if data[0]>data[719]:
        x=
        
        
    else:
    
    
"""      
"""
Problems:
3.Path to next vertex


Steps:
1.Subscribe to vertex topic that stores info about all the identified vertices as an array of vertex class objects
2.@node,check if coordinates correspond to vertex position of any vertex in the array
    Checking coordinates is by using ekf filter to get coordinates from odometry data of robot
    2a.if yes then extract the vertex and go to 3
    2b.else find edges connected to vertex(basically angles of edges connected) 
       initialise a vertex with current coordinates and go to 3

3.run merge_matrix algorithm
4.run Order_matrix algorithm
5.Select last edge of In() as next edge for traversal
6.Use "adjacency matrix" to get next path to traverse to next edge
7.Traverse to The next Edgemy

""" 



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
    for v in vertex_array:
        if v.x+tolerance < v_x < v.x-tolerance and v.y+tolerance < v_y < v.y-tolerance:
            rospy.loginfo("Arrived at node_tag:"+str(v.tag))
            v_found=v
    
    return v_found

##TO DO
def initialize_vertex_I():
    return []

#TO DO
def get_vertex_tag():
    return ""

def main():
    global cmd,data,flag,servcaller,servcaller2,node_found,check,mid_avg,params,params2,heading,odom_feedback,vertex_array
    #global vertices
    while not rospy.is_shutdown():
        if check!=[]:
            count=0

            for i in check:
                if i>range_thresh:
                    count+=1

            ###Check for node position###
            #TO DO : include end node here

            if count>=2 :
                rospy.loginfo("I'M AT NODE!")        
                forward_by_half_lane_width()
                #params.check=check
                #decision=servcaller(params).response
                #rospy.loginfo(decision)
                #Below commented code goes here.
                v_found=vertex_info()
                v_x=odom_feedback.pose.pose.position.x
                v_y=odom_feedback.pose.pose.position.y
                v_found=check_for_vertex_in_array(v_x,v_y)
                if v_found.tag=="":
                    rospy.loginfo("Found new node!!")
                    str=get_vertex_tag()
                    v_found.tag=str
                    v_found.x=v_x
                    v_found.y=v_y
                    v_found.I=initialize_vertex_I()
                    rospy.loginfo("Vertex with tag "+v_found.tag+"Found")
                
                    
                 #TO DO  
                I_dash=[] ##Completed Column ??

                ##First Step
                I_updated=op.first_step_on_vertex_visit(v_found.I,I_R,completed_column)
                I_R=I_updated
                v_found=I_updated
                vertex_array.append(v_found)
                pub_vertices.publish(vertex_array)
                
                ##Call direction_service.py for second step on vertex visit

                params.I=I_updated
                ##output will be array of directions    
                path=servcaller(params).response
                
                

                if decision=="L" :
                    #call turn_service_caller with param 0
                    rospy.loginfo("Turning Left")
                    params2.angle=pi/2
                    servcaller2(params2)
                    flag=0
                    forward_by_half_lane_width()
                elif decision=="R" :
                    #call turn_service_caller with param 1
                    rospy.loginfo("Turning Right")
                    params2.angle=-pi/2
                    servcaller2(params2)
                    flag=0
                    forward_by_half_lane_width()
                
                else:
                    rospy.loginfo("Going Forward")
                    forward_by_half_lane_width()
              
            
                    
                
            if count==0 and data[360]<lane_width/2:#mid_avg<range_thresh:
                rospy.loginfo("I'm at end node!")
                ###Call turn_service_caller with param 2
                params2.angle=pi
                servcaller2(params2)
                flag=0
            
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
    I_R=[]
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
    
    sub_vertex=rospy,Subscriber('/vertices',vertices,vertices_callback)
    pub_vertices=rospy.Publisher('/vertices',vertices,queue_size=1)

    main()
    rospy.spin()
