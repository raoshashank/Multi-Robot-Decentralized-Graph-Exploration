#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from math import pi,atan2,cos,sqrt,pow
from random import randint
import numpy as np
from nav_msgs.msg import Odometry
from project.srv import direction,directionRequest,directionResponse,dirturn,dirturnRequest,dirturnResponse 
from matrix_op import matrix_op
from project.msg import vertex_info,vertices
import networkx as nx

def path_to_next_edge(inci,current_v,next_v):
    global vertex_array,op
    adj=op.inci_to_adj(inci)
    rospy.loginfo(next_v.tag)
    i=current_v.pos_in_I
    j=next_v.pos_in_I   
    G=nx.from_numpy_matrix(adj,create_using=nx.DiGraph())    
    path=nx.dijkstra_path(G,i,j)
    rospy.loginfo(path)
    ret_path=[]
    for p in path:
        temp=check_for_vertex_in_array_index(p)
        ret_path.append(temp)
    del ret_path[0]
    rospy.loginfo(ret_path)
    return ret_path
def turn_to_next_vertex(current_v,next_v):
    global odom_feedback,cmd,pub
    err=0.2

    #rospy.loginfo("next_v.x: "+str(next_v.x)+" next_v.y: "+str(next_v.y))
    #rospy.loginfo("current_v.x: "+str(current_v.x)+" current_v.y: "+str(current_v.y))

    if   next_v.x>current_v.x and abs(next_v.y-current_v.y)<err:
        return -pi/2
    elif next_v.x<current_v.x and abs(next_v.y-current_v.y)<err:
        return pi/2
    elif next_v.y>current_v.y and abs(next_v.x-current_v.x)<err:
        return 0

    elif next_v.y<current_v.y and abs(next_v.x-current_v.x)<err:
        return pi

    
    


def check_for_vertex_in_array(v_x,v_y):
    global vertex_array
    v_found=vertex_info()
    v_found.tag=""
    tolerance=0.1
    distance=0.5    
    for v in vertex_array:
        err=sqrt((v_x-v.x)**2+(v_y-v.y)**2)
        rospy.loginfo(err)
        if  err<distance:#v.x-tolerance < v_x < v.x+tolerance and v.y-tolerance < v_y < v.y+tolerance:
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

def check_for_vertex_in_array_index(index):
    global vertex_array
    v_found=vertex_info()
    v_found.tag=""
    for v in vertex_array:
        if v.pos_in_I==index:
            v_found=v
    rospy.loginfo("found with index:"+v_found.tag)
    return v_found



def forward_by_half_lane_width():
    global data,check,odom_feedback,lane_width,pub

    x_start=odom_feedback.pose.pose.position.x
    y_start=odom_feedback.pose.pose.position.y
    delta=0.05
    error=0
    goal_dst=lane_width/2+delta
    dst=0
    while dst<=goal_dst and not rospy.is_shutdown():
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
    check=[data[719],data[360],data[0]]


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






def main():
    global cmd,data,flag,servcaller,servcaller2,node_found,check,mid_avg,params,params2,heading,odom_feedback,vertex_array,done,lane_width,range_thresh
    I=np.zeros((7,6))
    I[0][0]=1
    I[1][0]=1
    I[1][1]=1
    I[2][1]=1
    I[2][2]=1
    I[2][3]=1
    I[3][2]=1
    I[4][3]=1
    I[4][4]=1
    I[4][5]=1
    I[5][4]=1
    I[6][5]=1
    while not rospy.is_shutdown():
        if check!=[]:
            count=0
            for i in check:
                if i>range_thresh:
                    count+=1
            rospy.loginfo(check)
            if count>=2 or (count==0 and data[360]<lane_width/2):
              rospy.loginfo("I'M AT NODE!"+str(check))
              if count>=2:        
                forward_by_half_lane_width()

              orient_to_heading(pi/2)
              v_found=vertex_info()
              v_x=odom_feedback.pose.pose.position.x
              v_y=odom_feedback.pose.pose.position.y
              v_found=check_for_vertex_in_array(v_x,v_y)
              rospy.loginfo("I am at : "+v_found.tag)
              path=path_to_next_edge(I,v_found,vertex_array[7])
              rospy.loginfo(path)              
              for v_next in path:
                  next_turn=turn_to_next_vertex(v_found,v_next)
                  rospy.loginfo("Going to : " +v_next.tag)
                  rospy.loginfo(next_turn)
                  flag=0
                  if next_turn!=0:
                      params2.angle=next_turn
                      servcaller2(params2)
                      

                  forward_by_half_lane_width()

                  while True and not rospy.is_shutdown():
                      go_forward()
                      count_temp=0
                      for i in check:
                        if i>range_thresh:      
                            count_temp+=1
                      if count_temp>=2 or (count_temp==0 and data[360]<lane_width/2):
                          if count_temp>=2:
                             forward_by_half_lane_width()
                          orient_to_heading(pi/2)
                          v_found=vertex_info()
                          v_x=odom_feedback.pose.pose.position.x
                          v_y=odom_feedback.pose.pose.position.y
                          v_found=check_for_vertex_in_array(v_x,v_y)
                          rospy.loginfo("I am at : "+v_found.tag)
                          break
                        
                           
            else:
                go_forward()

        
if  __name__ == "__main__":
    rospy.init_node('follow_waypoint',anonymous=False)   
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
    rospy.wait_for_service('/turn_service_server')
    servcaller2=rospy.ServiceProxy('/turn_service_server',dirturn)
    
    sub_odom=rospy.Subscriber('/bot_0/odom',Odometry,odom_callback)
    pub=rospy.Publisher('/bot_0/cmd_vel',Twist,queue_size=1)
    
    sub=rospy.Subscriber('/bot_0/laser/scan',LaserScan,laser_callback)
    
    sub_vertex=rospy.Subscriber('/vertices',vertices,vertices_callback)
    pub_vertices=rospy.Publisher('/vertices',vertices,queue_size=1)

    main()
    rospy.spin()
