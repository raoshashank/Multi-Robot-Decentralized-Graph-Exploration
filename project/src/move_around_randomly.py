#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from math import pi,atan2,cos,sqrt,pow
from random import randint
import numpy as np
from nav_msgs.msg import Odometry
from project.srv import direction,directionRequest,directionResponse,dirturn,dirturnRequest,dirturnResponse 

#from project import vertex
#from project.msg import vertex_info
#from project.src import matrix_operations
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
def callback_vertex(msg):
    global vertices
    vertices=msg
"""        
"""
Problems:
1.Please Let me use Constant Lane Width!!Correction,I am using constant line width .Assume the bot starts at the middle of a lane
2.Incidence to Adjacency
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


#Constant lane width escape is just move forward by lane_width/2 from centre.
def forward_by_half_lane_width():
    
    global data,check,feedback,lane_width,pub

    x_start=feedback.pose.pose.position.x
    y_start=feedback.pose.pose.position.y
    delta=0.05
    error=0
    goal_dst=lane_width/2+delta
    dst=0
    while dst<=goal_dst:
       go_forward()
       dst=sqrt((feedback.pose.pose.position.x-x_start)**2+(feedback.pose.pose.position.y-y_start)**2)
       #rospy.loginfo(str(feedback.pose.pose.position.x)+" "+str(final_pos)+" "+str(initial_pos))

    cmd=Twist()
    pub.publish(cmd)
    rospy.loginfo("Escaped")

    



def go_forward():
    global q,cmd,feedback,flag,rate,heading,initial_heading,heading_error,angular_velocity_z,linear_velocity_x
    q[0]=feedback.pose.pose.orientation.w
    q[1]=feedback.pose.pose.orientation.x
    q[2]=feedback.pose.pose.orientation.y
    q[3]=feedback.pose.pose.orientation.z  
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
    #rate.sleep()
    pub.publish(cmd)
    #rospy.loginfo("Going Forward")



def callback2(msg):
    global feedback
    feedback=msg



def callback(msg):
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
    global cmd,data,flag,servcaller,servcaller2,node_found,check,mid_avg,params,params2,heading,feedback
    #global vertices
    while not rospy.is_shutdown():
        if check!=[]:
            count=0

            for i in check:
                if i>range_thresh:
                    count+=1

            ###Check for node position###
            if count>=2 :#and node_found==0:
               # node_found=1
                rospy.loginfo("I'M AT NODE!")        
                #cmd=Twist()
                #pub.publish(cmd)
                forward_by_half_lane_width()
                params.check=check
                decision=servcaller(params).response
                rospy.loginfo(decision)
                #rospy.loginfo(str(count)+"    "+str(node_found)+"     "+str(heading))
                #initial=feedback.pose.pose.position.x
#Below commented code goes here.

                if decision=="L" :
                    #call turn_service_caller with param 0
                    rospy.loginfo("Turning Left")
                    params2.angle=pi/2
                    servcaller2(params2)
                    flag=0
                    forward_by_half_lane_width()
                    #escape_turn(initial)
                elif decision=="R" :
                    #call turn_service_caller with param 1
                    rospy.loginfo("Turning Right")
                    params2.angle=-pi/2
                    servcaller2(params2)
                    flag=0
                    forward_by_half_lane_width()
                    #escape_turn(initial)
                
                else:
                    rospy.loginfo("Going Forward")
                    forward_by_half_lane_width()
                    #escape_turn(initial)
              
            
                    
                
            if count==0 and data[360]<lane_width/2:#mid_avg<range_thresh:
                rospy.loginfo("I'm at end node!")
                ###Call turn_service_caller with param 2
                params2.angle=pi
                servcaller2(params2)
                flag=0
                #node_found=1
            
            else:
                go_forward()
                
"""             exists=0
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
    feedback=Odometry()
    cmd=Twist()
    mid_avg=0
    range_thresh=2
    lane_width=2
    #vertices=[]
    #I=[]

    ##Service1 for deciding direction
    rospy.wait_for_service('/direction_service_server')
    servcaller=rospy.ServiceProxy('/direction_service_server',direction)        
    
    #Service2 for turning in the decided direction
    rospy.wait_for_service('/turn_service_server')
    servcaller2=rospy.ServiceProxy('/turn_service_server',dirturn)
    
    sub_odom=rospy.Subscriber('/bot_0/odom',Odometry,callback2)
    pub=rospy.Publisher('/bot_0/cmd_vel',Twist,queue_size=1)
    
    sub=rospy.Subscriber('/bot_0/laser/scan',LaserScan,callback)
    #sub_vertex=rospy,Subscriber('/vertex_info,vertex_msg,callback_vertex)
    main()
    rospy.spin()
