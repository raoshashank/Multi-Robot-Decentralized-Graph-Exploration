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
from project.msg import vertex_info,vertices,incidence
from collections import deque
import networkx as nx   
"""
1.Generate I' matrix
2.even when Dijsktra is running , on visiting edges,the nodes have to identified , merging etc to be done but new edge not to be changed unless its gets traversed
3.vertex_tags management
4.Second step Algo Q management

"""
"""
main() will only identify current vertices,intialise  vertices and execute First_Step
and Second Step.
When bo vertex is found,main() drives robot in forward direction
Second Step algo will initialise Queue containing the vertices for trajectory
Second step should direct robot so it should give direction to turn etc.

"""
#Give direction to turn for reaching next_turn
def turn_to_next_vertex(current_v,next_v):
    global odom_feedback,cmd,pub
    err=0.2
    if   next_v.x>current_v.x and abs(next_v.y-current_v.y)<err:
        return -pi/2
    elif next_v.x<current_v.x and abs(next_v.y-current_v.y)<err:
        return pi/2
    elif next_v.y>current_v.y and abs(next_v.x-current_v.x)<err:
        return 0
    elif next_v.y<current_v.y and abs(next_v.x-current_v.x)<err:
        return pi



def second_step_on_vertex_visit(current_v):
     global traverse_q,E1cap,E2cap,Vcap,tags_array_R,I_R,Ec
     global current_v,previous_vertex,next_vertex
    
     if op.non_zero_element_count(I_R[:,E1cap+E2cap-1])==2:
        break
     else:
         if len(traverse_q)==0 or op.non_zero_element_count(traverse_q[len(traverse_q)-1])==2:
            next_edge=I_R[:,E1cap+E2cap-1]
        
            ##identify index of known edge on next vertex for passing to dijkstra
         
            for i in next_edge:
                if i!=0:
                    next_v_tag=tags_array_R[i]
                    break
      
            #index of current edge for dijsktra
            for j in I_R:
                if tags_array_R[j]==current_v.inci.tag:
                    break

            next_v=check_for_vertex_in_array_tag(next_v_tag)
            adj=op.inci_to_adj(I_R)
            G=nx.from_numpy_matrix(adj,create_using=nx.DiGraph()) 
            path=nx.dijkstra(G,i,j)
            ##Generated path has indices of vertices in adjacency matrix which same as in incidence matrix
            del path[0]       #First vertex is current vertex always in result  
            ret_path=[]
            for p in path:
                ##vertex_at_pos(p) to be defined
                temp=vertex_at_pos(p)
                ret_path.append(temp)
        
            ##path containing vertex objects generated      

            for r in ret_path:
                 traverse_q.append(r)
            
            ##Queue updated




         next_vertex=traverse_q[0]
         if op.non_zero_element_sign(I_R[:,E1cap+E2cap-1])>1:
            I_R[:,E1cap+E2cap-1]=-I_R[:,E1cap+E2cap-1]


         #circular shifting   
         b=I_R[:,Ec:E1cap+E2cap-1]
         b=np.roll(b,-1*(b.shape[1]-1))
         I_R=I_R[:,0:Ec]
         I_R=np.column_stack((I_R,b))
         
        
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


def initialize_vertex_I():  
    global heading,range_thresh
    I=[]
    err=0.1
    ##Angles to be checked 0(=2pi),pi/2,-pi(=pi),-pi/2(3pi/2) in order
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
    global E1cap,E2cap,Vcap,I_R,tags_array_R,Ec
    global current_v,previous_vertex

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
                ##New Vertex discovery
                if v_found.tag=="":
                    rospy.loginfo("Found new node!!")
                    v_found.tag='x'+str(v_x)+'y'+str(v_y)
                    v_found.x=v_x
                    v_found.y=v_y
                    v_found.inci.I=initialize_vertex_I()
                    v_found.inci.tags_array.append(v_found.tag)
                    rospy.loginfo("Vertex with tag "+v_found.tag+"Found")
                else:
                    rospy.loginfo("I'm at node with tag:"+v_found.tag)
                
                current_v=v_found
                ##First Step
                I_double_dash=[]
                I_dash=np.zeros((2,1))
                I_dash=[]
                Ec=0
                [I_double_dash,Vcap,E1cap,E2cap]=op.merge_matrices(I_dash,I_R)
                [I_R,Vcap,E1cap,E2cap]=op.merge_matrices(I_double_dash,v_found.inci.I)
                [Ec,I_R]=op.Order_Matrix(I_R,E1cap,E2cap,Vcap)            
                v_found.inci.I=I_R
                #v_found.inci.tags_array=tags_array_R
                ##UPDATE tags_array for Rk
                #Do second step on vertex visit
                second_step_on_vertex_visit(v_found)   
                v_found.inci.I=I_R   
                v_found.inci.tags_array=tags_array_R
                
                #publish updated vertex info to /vertices topic
                for count,v in enumerate(vertex_array):
                    if v_found.tag==v.tag:
                        vertex_array[count]=v_found

                pub_vertices.publish(vertex_array)             
            
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
    

    previous_vertex=vertex_info()
    next_vertex=vertex_info()
    current_v=vertex_info()

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
    Inci_R=incidence()
    I_R=Inci_R.I
    tags_array_R=Inci_R.tags_array
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
