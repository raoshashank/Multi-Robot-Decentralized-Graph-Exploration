#!/usr/bin/env python
#For any info contact: Shashank Rao M       e-mail: raoshashank73@gmail.com 
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
import sys

def shift( matrix):
        #This Functions implements right circular shifting of the given array by one column  
        n =  -(1 % len(matrix[0]))
        temp = zip(*matrix)
        h = temp[:n]
        del temp[:n]
        temp.extend(h)
        matrix[:] = map(list, zip(*temp))
        return matrix
   

def turn_to_next_vertex(current_v,next_v):
    #This function returns the heading to be turned by the robot so that it heads towards the next vertex
    #This is done by comparing the x,y values of the two vertices
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
        #If the current vertex and the next_vertex are the same,then it shouldnt turn to any heading
        return find_heading()

def edge_from_v(v_1,v_2,I_R):
    #This Function extracts the index of the edge in the array I_R containing 
    # the two vertices v_1 and v_2 
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
     #Executes Second_Step_On_vertex_visit 
     #Manages que of vertices to reach and que of edges to be traversed
     global traverse_q,E1cap,E2cap,Vcap,Ec,I_R,ret_path,traverse_q
     global current_v,previous_vertex,next_vertex,turn_at_dest
     
     if op.non_zero_element_count(I_R[:,E1cap+E2cap])==2:
        #If last column of I_R is completed then graph traversal is completed
        rospy.loginfo("Exploration Complete")
        return
        
     else:
     
         if len(traverse_q)>0: 
            #This part of the code finds the column corresponding to the last column in the traverse_q in the I_R to check i it completed or not.
            last_edge=traverse_q[len(traverse_q)-1]
            #i will be index of non-zero element in last_edge
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
            #This condition is to check if the robot has traversed the selected edge or has found that the selected edge has already been completed by another robot
            next_edge=I_R[:,E1cap+E2cap]    
            #turn_at_dest is the angle that the robot should turn to to traverse the unexplored or out edge on reaching the source vertex of the selected edge
            turn_at_dest=0
            #index of current edge for dijsktra
            for source in range(len(I_R[:,0])):
                if I_R[source,0].tag==current_v.tag:
                    break
            ##identify index of known edge on next vertex for passing to dijkstra
            for target in range(len(next_edge)):
                if next_edge[target]!=0:
                    turn_at_dest=np.absolute(next_edge[target])
                    break
            traverse_q=deque()

            adj=op.inci_to_adj(I_R[:,1:I_R.shape[1]])               # returns the adjacency matrix corresponding to the supplied incidence matrix
            G=nx.from_numpy_matrix(adj,create_using=nx.DiGraph())   #Converts numpy array to networkx type array 
            path=nx.dijkstra_path(G,source,target)                  # return dijkstra path containing the indices of the vertices in the adjacency matrix on the Graph G from source to target
            ret_path=[]                                             #ret_path is the que containing the vertex_info objects of the vertices corresponding to the indices in the array path
            for p in path:
                ret_path.append(I_R[p,0])                           #The indices in the adjacency and incidence matrix correspond to the same vertex
            ret_path=deque(ret_path)
            vert_col=I_R[:,0].transpose()
            for i in range(len(ret_path)-1):
                e=edge_from_v(ret_path[i],ret_path[i+1],I_R)        #find the edge containing corresponding vertices to find the continuous path to the target from the source
                traverse_q.append(np.column_stack((vert_col,I_R[:,e])))      #Add the found edge to the traverse_q
            del ret_path[0]                                                  #The first vertex in the path array is the current vertex
            traverse_q.append(np.column_stack((vert_col,next_edge)))         #Add unknown edge to traverse_q edge que
    

         next_edge=traverse_q.popleft()                                     # Immediate next edge to traverse
         rospy.loginfo("Next Edge after popping from que:"+str(next_edge[:,1:next_edge.shape[1]])) 
  
         try:
            next_vertex=ret_path.popleft()                                     ##Immediate next vertex to reach
            orient_to_heading(turn_to_next_vertex(current_v,next_vertex))      ##turn towards the next vertex
         except IndexError:     
             orient_to_heading(turn_at_dest)                                   #If popping from ret_path gives Index_error means the que is empy meaning that the robot has arrived at the source vertex of the unkniwn edge
             if op.non_zero_element(I_R[:,E1cap+E2cap])>0:                     #Next edge will be the selected edge so make it as OUT EDGE in the incidence matrix
                I_R[:,E1cap+E2cap]=-I_R[:,E1cap+E2cap]
         
             b=I_R[:,Ec+1:]                                                    #Circular shifting is done so that new unexplored edge can be selected on reaching the end of unexplored edge
             b=shift(b)
             I_R=I_R[:,0:Ec+1]
             I_R=np.column_stack((I_R,b))   
    
         previous_vertex=current_v                                              #Current vertex becomes previous vertex
     
        
def orient_to_heading(dir):
    #This function orients the robot towards the supplied heading
    global flag,odom_feedback,heading_cmd,heading,q,cmd,done,angle,initial_heading,heading_error
    q=[0,0,0,0]                                                                                     #array q will recieve the quaternion from the odometry callback
    done=0
    heading_cmd=dir
    while done!=1 and not rospy.is_shutdown():
        cmd=Twist()
        heading=find_heading()                      
        heading_error=heading_cmd-heading                                                           #heading_cmd is the desired heading to be achieved

        if heading_error>pi:
            heading_error=heading_error-2*pi
        if heading_error<=-pi:
            heading_error=heading_error+2*pi
    
    
        if abs(heading_error) < 0.001:                                                             #Limiting error of resulting heading to be 0.001
            done=1                      
        else:
            cmd.angular.z=-0.8*heading_error                                                        #P-Control to turn the bot to the desired heading
            pub.publish(cmd)
    initial_heading=find_heading()                                                                  #The initial heading will now be updated to the now achieved heading

def forward_by_half_lane_width():   

    # This function is used to prevent the robot from continuously detecting a node at the node position.
    #It moves the robot by a distance of exactly half the lane width to escape the node and enter the next corridoor

    global data,check,odom_feedback,lane_width,pub,flag

    x_start=odom_feedback.pose.pose.position.x
    y_start=odom_feedback.pose.pose.position.y
    delta=0.05
    error=0
    goal_dst=lane_width/2+delta
    dst=0
    while dst<=goal_dst:                                 #Keep going forward until the distance from the initial position is half lane width +/- error
       go_forward()
       dst=sqrt((odom_feedback.pose.pose.position.x-x_start)**2+(odom_feedback.pose.pose.position.y-y_start)**2)

    cmd=Twist()
    pub.publish(cmd)                                    #Stop the bot after this 


def find_heading():

    #This function converts the quaternion data obtained from odom_callback to a heading or yaw angle using the standard mathematical
    #transformation formula
    global odom_feedback
    q=[0,0,0,0]                                 #Quaternion array
    q[0]=odom_feedback.pose.pose.orientation.w
    q[1]=odom_feedback.pose.pose.orientation.x
    q[2]=odom_feedback.pose.pose.orientation.y
    q[3]=odom_feedback.pose.pose.orientation.z  
    heading=atan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]*q[2]+q[3]*q[3]))        #heading corresponding to the quaternion data
    return heading          

def go_forward():
    
    #This function uses P-Control to keep the robot moving along a straight line 
    
    global q,cmd,odom_feedback,flag,rate,heading,initial_heading,heading_error,angular_velocity_z,linear_velocity_x
    heading=find_heading()
    heading_error=initial_heading-heading
    if heading_error>pi:
        heading_error=heading_error-2*pi
    if heading_error<=-pi:
        heading_error=heading_error+2*pi
    angular_velocity_z=-0.9*heading_error               #P-Control with Kp=0.9
    cmd=Twist()
    cmd.linear.x=linear_velocity_x
    cmd.angular.z=angular_velocity_z
    pub.publish(cmd)                                    #publish the angular velocity command to reduce deviation from straight path



def odom_callback(msg):
    
    #This function is the callback for the /bot/odom topic
    
    global odom_feedback
    odom_feedback=msg

def vertices_callback(msg):

    #This function is the callback for the /vertices topic
    #this will return the vertex_array containing the information of all the nodes detected in the form of vertex_info objects

    global vertex_array
    vertex_array=msg.v



def laser_callback(msg):

    #This function is the callback for the /bot/laser/scan topic
    
    global check,mid_avg,data
    data=msg.ranges
    check=[data[719],data[360],data[0]]         #[Left,Middle,Right] directions . check array contains the ranges in the 3 directions mentioned
    

def check_for_vertex_in_array(v_x,v_y):
    
    #This function finds the vertex corresponding to the supplied x,y coordinates and returns a vertex_info object with tag "empty" if no match is found
    global vertex_array
    v_found=vertex_info()
    v_found.tag="empty"
    tolerance=0.1
    # an error of 0.1 wrt to the x,y coordinates is allowed
    distance=0.5  
    for v in vertex_array:
        err=sqrt((v_x-v.x)**2+(v_y-v.y)**2)
        if  err<distance:
            v_found=v
            
    return v_found



def initialize_vertex_I():  

    #This function ibnitilizes the Incidence matrix of a newly identified edge by finding the directions available for traversal around it.
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
    return I


def main():
    global cmd,data,flag,node_found,check,mid_avg,heading,odom_feedback,vertex_array,initial_heading
    global E1cap,E2cap,Vcap,I_R,Ec
    global current_v,previous_vertex
    Ec=0
    err=0.2
    initial_heading=find_heading()
    ##Lane width measurement
    while check==[]:
        rospy.loginfo("Loading")                #To prevent the exploration from starting before the laser range finder data is obtained
    lane_width=check[0]+check[2]                #lane width calculation
    while not rospy.is_shutdown():
        if check!=[]:
            count=0
            for i in check:                     #finding the number of directions available around the vertex.
                if i>range_thresh:
                    count+=1

            if count>=2 or (count==0 and data[360]<lane_width/2):               ##If more than 2 directions are available for traversal or the distance from the wall is less than half the lane width(end node),
                                                                                ##Then a node is found
                if count>=2:
                    forward_by_half_lane_width()                                ##Enter the node area which is in the middle of the node square because the node will be detected immediately on exiting a corridoor
                
                #I' initialization should be done first beacuse heading info will be lost after orienting to angle(pi/2)
                I_dash=[] 
                if previous_vertex.tag!='':                                        ##This part of the code finds the I' array corresponding to the recently completed edge
                 h=find_heading()
                 if abs(h)<err:
                    I_dash.append(2*pi)
                    I_dash.append(pi)
                
                 elif abs(h-pi/2)<err:
                    I_dash.append(pi/2)
                    I_dash.append(3*pi/2)
                
                 elif (h<0 and abs(h+pi)<err) or (h>0 and abs(h-pi)<err):
                    I_dash.append(pi)
                    I_dash.append(2*pi)
                
                 elif abs(h+pi/2)<err:
                    I_dash.append(3*pi/2)
                    I_dash.append(pi/2)
                I_dash=np.array([I_dash]).transpose()

                orient_to_heading(pi/2)                                             #Orient to pi/2 heading because turning towards the next vertex assumes that it has oriented itself to pi/2 heading
                current_v=vertex_info()
                current_v_I=[]
                ###temp for doing computation before storing in pickle
                v_x=odom_feedback.pose.pose.position.x
                v_y=odom_feedback.pose.pose.position.y
                current_v=check_for_vertex_in_array(v_x,v_y)
                ##New Vertex discovery
                if current_v.tag=="empty":                                          #Condition for detecting a new node
                    current_v.x=v_x
                    current_v.y=v_y
                    current_v_I=initialize_vertex_I()
                    temp=np.array([current_v])
                    current_v_I=np.column_stack((temp,current_v_I))
                    current_v.I=pickle.dumps(current_v_I)                           #since ros doesnt allow publishing raw 2d arrays or dynamic dimensions,
                    current_v.tag="x"+str(v_x)+"y"+str(v_y)                         #use pickle to save the array and give a string corresponding to the saved location

                else:
                    current_v_I=pickle.loads(current_v.I)                              #In case the vertex is already detected  
               
                I_double_dash=I_R                                                   #In case the first merging is not done
                #Finding I'
               
                if previous_vertex.tag!='':
                    vert_col_dash=np.array([[previous_vertex,current_v]]).transpose()
                    I_dash=np.column_stack((vert_col_dash,I_dash))
                            
                    ##FIRST STEP ON VERTEX VISIT##

                    ##############################################################
                    [I_double_dash,Vcap,E1cap,E2cap]=op.merge_matrices(I_dash,I_R)
                    ##############################################################                                   
               
                ###################################################################
                [I_R,Vcap,E1cap,E2cap]=op.merge_matrices(I_double_dash,current_v_I)
                ###################################################################
               
                ##################################################################################
                [Ec,I_R[:,1:I_R.shape[1]]]=op.Order_Matrix(I_R[:,1:I_R.shape[1]],E1cap,E2cap,Vcap)
                ##################################################################################
                
                ##################################################################################
                current_v_I=I_R
                ##################################################################################
                
                #SECOND STEP ON VERTEX VISIT##

                ##################################################################################
                second_step_on_vertex_visit()   
                ##################################################################################
                
                ##################################################################################
                current_v_I=I_R   
                ##################################################################################
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
   


    bot_num='bot_'+str(sys.argv[1])
    sub_odom=rospy.Subscriber('/'+bot_num+'/odom',Odometry,odom_callback)
    pub=rospy.Publisher('/'+bot_num+'/cmd_vel',Twist,queue_size=1)
    sub=rospy.Subscriber('/'+bot_num+'/laser/scan',LaserScan,laser_callback)
    sub_vertex=rospy.Subscriber('/vertices',vertices,vertices_callback)
    pub_vertices=rospy.Publisher('/vertices',vertices,queue_size=1,latch=True)
    rate.sleep()
    pub_vertices.publish(vertices())
    #To start publishing to the /vertices topic
    rate.sleep()
    main()
    rospy.spin()
