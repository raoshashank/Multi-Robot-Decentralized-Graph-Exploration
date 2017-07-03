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

def shift( matrix):
        ''' shift given 2D matrix in-place the given number of rows or columns
            in the specified (UP, DOWN, LEFT, RIGHT) direction and return it
        '''
        n =  -(1 % len(matrix[0]))
        temp = zip(*matrix)
        h = temp[:n]
        del temp[:n]
        temp.extend(h)
        matrix[:] = map(list, zip(*temp))
        return matrix
   

def turn_to_next_vertex(current_v,next_v):
    global odom_feedback,cmd,pub
    err=0.2
    if   next_v.x>current_v.x and abs(next_v.y-current_v.y)<err:
        return 0
    elif next_v.x<current_v.x and abs(next_v.y-current_v.y)<err:
        return pi
    elif next_v.y>current_v.y and abs(next_v.x-current_v.x)<err:
        return pi/2
    elif next_v.y<current_v.y and abs(next_v.x-current_v.x)<err:
        return 3*pi/2
    else:
        return find_heading()

def edge_from_v(v_1,v_2,I_R):
    for i in range(I_R.shape[0]):
     if I_R[i,0].tag==v_1.tag:
       break
    for j in range(I_R.shape[0]):           
     if I_R[j,0].tag==v_2.tag:
       break
    
    for e in range(1,I_R.shape[1]):
     if I_R[i][e]!=0 and I_R[j][e]!=0:
       break

    #index of edge
    return e


def second_step_on_vertex_visit():
     global traverse_q,E1cap,E2cap,Vcap,Ec,I_R,ret_path,traverse_q
     global current_v,previous_vertex,next_vertex,turn_at_dest
     
     if op.non_zero_element_count(I_R[:,E1cap+E2cap])==2:
        #rospy.loginfo("Exploration Complete")
        return
        
     else:
     
         if len(traverse_q)>0: 
            rospy.loginfo("Traverse queue length:"+str(len(traverse_q)))
            last_edge=traverse_q[len(traverse_q)-1]
             # i will be index of non-zero element in last_edge
            for i in range(len(last_edge[:,0])):
                 if last_edge[i,1]!=0:
                    break
            #v_in_e will be vertex corresponding to non-zero value in last_edge   
            v_in_e=last_edge[i,0]               
            #index_of_v_in_I_R will be index of this vertex in I_R matrix
            index_of_v_in_I_R=[x for x in range(len(I_R[:,0])) if I_R[x,0].tag==v_in_e.tag]
            #now to match absolute values to find the edge in I_R
            for j in range(1,I_R.shape[1]):
                 if np.absolute(I_R[index_of_v_in_I_R,j])==np.absolute(last_edge[i,1]):
                    break

            last_edge_of_q_in_I_R=I_R[:,j]                          
        
         if len(traverse_q)==0 or op.non_zero_element_count(last_edge_of_q_in_I_R)==2:      
            next_edge=I_R[:,E1cap+E2cap]
            turn_at_dest=0
            #index of current edge for dijsktra
            for source in range(len(I_R[:,0])):
                if I_R[source,0].tag==current_v.tag:
                    break
            rospy.loginfo("Source: "+I_R[source,0].tag)
            rospy.loginfo("I_R"+str(I_R[:,1:I_R.shape[1]]))
            ##identify index of known edge on next vertex for passing to dijkstra
            for target in range(len(next_edge)):
                if next_edge[target]!=0:
                    turn_at_dest=np.absolute(next_edge[target])
                    break
            
            rospy.loginfo("target:"+I_R[target,0].tag)
            traverse_q=deque()
            adj=op.inci_to_adj(I_R[:,1:I_R.shape[1]])
            #rospy.loginfo("Adjacency:"+str(adj))
            G=nx.from_numpy_matrix(adj,create_using=nx.DiGraph()) 
            path=nx.dijkstra_path(G,source,target)
            rospy.loginfo("index of vertices in index path : " + str(path))
            #del path[0]       #First vertex is current vertex always in result  
            ret_path=[] 
            for p in path:
                ret_path.append(I_R[p,0])
            ret_path=deque(ret_path)
            rospy.loginfo("Que of vertices:"+str([v.tag for v in ret_path]))
            vert_col=I_R[:,0].transpose()
            for i in range(len(ret_path)-1):
                e=edge_from_v(ret_path[i],ret_path[i+1],I_R)
                rospy.loginfo("Found edge between "+ ret_path[i].tag+"and"+ret_path[i+1].tag)
                traverse_q.append(np.column_stack((vert_col,I_R[:,e])))
            del ret_path[0]
            traverse_q.append(np.column_stack((vert_col,next_edge)))
    
     rospy.loginfo("traverse_que:")
     for i in traverse_q:
         rospy.loginfo(i[:,1:i.shape[1]])
     next_edge=traverse_q.popleft()
     rospy.loginfo("Next Edge after popping from que:"+str(next_edge[:,1:next_edge.shape[1]]))
  
     try:
         next_vertex=ret_path.popleft() ##Immediate next vertex to reach
         rospy.loginfo("current vertex:"+current_v.tag)
         rospy.loginfo("next vertex:"+next_vertex.tag)
         orient_to_heading(turn_to_next_vertex(current_v,next_vertex))
     except IndexError:     
             rospy.loginfo("ret_path is empty;arrived at destination vertex")
             orient_to_heading(turn_at_dest)     
             if op.non_zero_element(I_R[:,E1cap+E2cap])>0:
    		 I_R[:,E1cap+E2cap]=-I_R[:,E1cap+E2cap]
             #circular shifting   
     	     b=I_R[:,Ec+1:]
             b=shift(b)
             I_R=I_R[:,0:Ec+1]
             I_R=np.column_stack((I_R,b))   
    
     previous_vertex=current_v
     
        
def orient_to_heading(dir):

    global flag,odom_feedback,heading_cmd,heading,q,cmd,done,angle,initial_heading,heading_error
    q=[0,0,0,0]
    done=0
    heading_cmd=dir
    while done!=1 and not rospy.is_shutdown():
        cmd=Twist()
        heading=find_heading()
        heading_error=heading_cmd-heading

        if heading_error>pi:
            heading_error=heading_error-2*pi
        if heading_error<=-pi:
            heading_error=heading_error+2*pi
    
    
        if abs(heading_error) < 0.001:
            done=1
        else:
            cmd.angular.z=-0.8*heading_error
            pub.publish(cmd)
    initial_heading=find_heading()

def forward_by_half_lane_width():
    global data,check,odom_feedback,lane_width,pub,flag

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


def find_heading():
    global odom_feedback
    q=[0,0,0,0]
    q[0]=odom_feedback.pose.pose.orientation.w
    q[1]=odom_feedback.pose.pose.orientation.x
    q[2]=odom_feedback.pose.pose.orientation.y
    q[3]=odom_feedback.pose.pose.orientation.z  
    heading=atan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]*q[2]+q[3]*q[3]))
    return heading

def go_forward():
    global q,cmd,odom_feedback,flag,rate,heading,initial_heading,heading_error,angular_velocity_z,linear_velocity_x
    heading=find_heading()
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
    check=[data[719],data[360],data[0]]    
    

def check_for_vertex_in_array(v_x,v_y):
    global vertex_array
    v_found=vertex_info()
    v_found.tag="empty"
    tolerance=0.1
    distance=0.5  
    for v in vertex_array:
        err=sqrt((v_x-v.x)**2+(v_y-v.y)**2)
        if  err<distance:
            v_found=v
            
    return v_found



def initialize_vertex_I():  
    global heading,range_thresh
    I=[]
    err=0.1
    #rospy.loginfo("Initializing new vertex!")
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
    I=np.array([I])
    ##rospy.loginfo(I)
    return I


def main():
    global cmd,data,flag,node_found,check,mid_avg,heading,odom_feedback,vertex_array,initial_heading
    global E1cap,E2cap,Vcap,I_R,Ec
    global current_v,previous_vertex
    Ec=0
    err=0.2
    initial_heading=find_heading()
    while not rospy.is_shutdown():
        if check!=[]:
            count=0
            for i in check:
                if i>range_thresh:
                    count+=1

            if count>=2 or (count==0 and data[360]<lane_width/2):               ##NODE condition including end node
                if count>=2:
                    forward_by_half_lane_width()
                
                #rospy.loginfo("######################################################")
                #I' initialization should be done first beacuse heading info will be lost after orienting to angle(pi/2)
                I_dash=[] 
                if previous_vertex.tag!='':
                 #rospy.loginfo("forming I' array") 
                 h=find_heading()
                 rospy.loginfo("heading:"+str(h)+"error:"+str(err))
                 if abs(h)<err:
                    I_dash.append(2*pi)
                    I_dash.append(pi)
                
                 elif abs(h-pi/2)<err:
                    I_dash.append(pi/2)
                    I_dash.append(3*pi/2)
                
                 elif abs(h-pi)<err:
                    I_dash.append(pi)
                    I_dash.append(2*pi)
                
                 elif abs(h+pi/2)<err:
                    I_dash.append(3*pi/2)
                    I_dash.append(pi/2)
                 I_dash=np.array([I_dash]).transpose()
                rospy.loginfo("I':"+str(I_dash))
                orient_to_heading(pi/2)
                current_v=vertex_info()
                current_v_I=[]
                ###temp for doing computation before storing in pickle
                v_x=odom_feedback.pose.pose.position.x
                v_y=odom_feedback.pose.pose.position.y
                current_v=check_for_vertex_in_array(v_x,v_y)
                ##New Vertex discovery
                if current_v.tag=="empty":
                    current_v.x=v_x
                    current_v.y=v_y
                    current_v_I=initialize_vertex_I()
                    temp=np.array([current_v])
                    current_v_I=np.column_stack((temp,current_v_I))
                    current_v.I=pickle.dumps(current_v_I)                    
                    current_v.tag="x"+str(v_x)+"y"+str(v_y)
                    #rospy.loginfo("Found new node!!@"+current_v.tag)

                else:
                    #rospy.loginfo("I'm at node :"+current_v.tag)
                    current_v_I=pickle.loads(current_v.I)
               
                I_double_dash=I_R            
                
                
                #Finding I'
                #rospy.loginfo("previous_vertex:"+previous_vertex.tag)
                if previous_vertex.tag!='':
                    vert_col_dash=np.array([[previous_vertex,current_v]]).transpose()
                    I_dash=np.column_stack((vert_col_dash,I_dash))
                    #rospy.loginfo("Before merging I' and I_R")
                    #rospy.loginfo("I':") 
                    #rospy.loginfo(I_dash[:,1:I_dash.shape[1]])
                    #rospy.loginfo("I_R:")
                    #rospy.loginfo(I_R[:,1:I_R.shape[1]])                
                    ##FIRST STEP ON VERTEX VISIT##

                    ##############################################################
                    [I_double_dash,Vcap,E1cap,E2cap]=op.merge_matrices(I_dash,I_R)
                    ##############################################################                                   
               
                    #rospy.loginfo("After merging I' and I_R")
                    #rospy.loginfo(I_double_dash[:,1:I_double_dash.shape[1]])
                    #rospy.loginfo("E1cap:"+str(E1cap)+"E2cap:"+str(E2cap)+"Vcap:"+str(Vcap))

                #rospy.loginfo("Before merging I'' and vertex_I")
                #rospy.loginfo("I'':")
                #rospy.loginfo(I_double_dash[:,1:I_double_dash.shape[1]])                
                #rospy.loginfo("vertex_I:")
                #rospy.loginfo(current_v_I[:,1:current_v_I.shape[1]])

                ###################################################################
                [I_R,Vcap,E1cap,E2cap]=op.merge_matrices(I_double_dash,current_v_I)
                ###################################################################

                #rospy.loginfo("E1cap:"+str(E1cap)+"E2cap:"+str(E2cap)+"Vcap:"+str(Vcap)) 
                #rospy.loginfo("After merging I'' and vertex_I")
                #rospy.loginfo(I_R[:,1:I_R.shape[1]]) 
                ##rospy.loginfo(I_R)
               
                ##################################################################################
                [Ec,I_R[:,1:I_R.shape[1]]]=op.Order_Matrix(I_R[:,1:I_R.shape[1]],E1cap,E2cap,Vcap)
                ##################################################################################
               
                #rospy.loginfo("After Ordering I_R")
                #rospy.loginfo(I_R[:,1:I_R.shape[1]]) 
                ##rospy.loginfo("Ec:"+str(Ec))
                ##rospy.loginfo("First step results: I_R:"+str(I_R[:,1:I_R.shape[1]])+" E1cap:"+str(E1cap)+" E2cap"+str(E2cap)+" Vcap:"+str(Vcap))
                ##rospy.loginfo(I_R)
                
                ##################################################################################
                current_v_I=I_R
                ##################################################################################
                
                #SECOND STEP ON VERTEX VISIT##
                #rospy.loginfo("E1cap:"+str(E1cap)+"E2cap:"+str(E2cap)+"Vcap:"+str(Vcap))
                
                ##################################################################################
                second_step_on_vertex_visit()   
                ##################################################################################
                
                
                #rospy.loginfo("Second Step Done!I_R:")
                #rospy.loginfo(I_R[:,1:I_R.shape[1]])
                
                ##################################################################################
                current_v_I=I_R   
                ##################################################################################
                rospy.loginfo(I_R[:,1:I_R.shape[1]])
                rospy.loginfo("-----")
                forward_by_half_lane_width()  
                #publish updated vertex info to /vertices topicx
                for count,v in enumerate(vertex_array):
                    if current_v.tag==v.tag:
                        del vertex_array[count]
                current_v.I=pickle.dumps(current_v_I)
                vertex_array.append(current_v)        
                pub_vertices.publish(vertex_array)    
                #rospy.loginfo("updated vertex_array with vertex:"+current_v.tag )         
            
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
    linear_velocity_x=0.15
    angular_velocity_z=0
    
    rate=rospy.Rate(5)
    
    ret_path=[]
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
    turn_at_dest=0

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
    pub_vertices=rospy.Publisher('/vertices',vertices,queue_size=1,latch=True)
    rate.sleep()
    pub_vertices.publish(vertices())
    rate.sleep()
    main()
    rospy.spin()
