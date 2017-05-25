#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from math import pi
from random import randint
import numpy as np
from project.srv import direction,directionRequest,directionResponse
from project.srv import dirturn,dirturnRequest,dirturnResponse
rospy.init_node('random_mover',anonymous=False)
"""
do preliminary scan and move in the direction of maxima in laser scan data
currently assuming right angle turns
"""

###Lets make turning action also service so that node doesnt

def go_forward():
    cmd=Twist()
    cmd.linear.x=linear_velocity_x
    pub.publish(cmd)
    rate.sleep()
    cmd=Twist()
    pub.publish(cmd)
    
def callback(msg):
    data=msg.ranges
    ###find maximas:
    ###check in 3 directions for the free space
    ###slice the ranges array into 3 regions for 3 directions:;left,forward and right
    left_slice=np.asarray(data[115:125])
    left_avg=left_slice.sum()/len(left_slice)
    mid_slice=np.asarray(data[355:365])
    mid_avg=mid_slice.sum()/len(mid_slice)
    right_slice=np.asarray(data[595:605])
    right_avg=right_slice.sum()/len(right_slice)
    check=[left_avg,mid_avg,right_avg]
    rospy.loginfo("Values i found are:"+str(check))    
    count=0
    ##Service1 for deciding direction
    rospy.wait_for_service('/direction_service_server')
    servcaller=rospy.ServiceProxy('/direction_service_server',direction)
        
    #Service2 for turning in the decided direction
    rospy.wait_for_service('/turn_service_server')
    servcaller2=rospy.ServiceProxy('/turn_service_server',dirturn)
    ###Check for node position###
    for i in check:
        if i>3:
            count+=1
    
    if count>=2 :
        rospy.loginfo("I'M AT NODE!")        
        cmd=Twist()
        pub.publish(cmd)
        rate.sleep()        
        ##params for service1
        params=directionRequest()
        params.check=check
        
        #param for service2
        params2=dirturnRequest()
        deci=servcaller(params)
        decision=deci.response
        if decision=="R":
            ###call turn_service_caller with param 0
            params2.dir=0
            status=servcaller2(params2)
            rospy.loginfo(str(status))
        elif decision=="L":
            ###call turn_service_caller with param 1
            params2.dir=1
            status=servcaller2(params2)
            rospy.loginfo(str(status))
            ##keep going forward
        elif decision=="F":
            go_forward()
        
    elif count==0:
       rospy.loginfo("I'm at end node!")
       ###Call turn_service_caller with param 2
       params2=dirturnRequest()
       params2.dir=2
       status=servcaller2(params2)
       rospy.loginfo(str(status))
    else:
         go_forward()
"""
        while True:
            i=randint(0,2)
            if check[i]>3:
                break

        rospy.loginfo("Value of i is : "+str(i))
    
        if i==0:
            rospy.loginfo("Turning left")
            #turn(1)

        elif i==1:
            rospy.loginfo("going forward")
            go_forward()
        elif i==2:
            rospy.loginfo("Turning Right")
            #turn(0)
    else:
        go_forward()
"""
""" 
    if left_avg>3:
        ###turn left and move the bot
        rospy.loginfo("i will turn left!")
        turn(1)
    else:
         if right_avg>3:
             rospy.loginfo("I will turn right!")
             turn(0)

         else:
             if mid_avg>3:
                 rospy.loginfo("I need to go straight!")
                 go_forward()
             else :
                 rospy.loginfo("need to turn around")               
                 turn_around()
"""                   
"""
   The three angles for checking are : (-100:-80),(-10,10),(80,100)
   so array indexes for these angles in the case of husky robot for 
   a -135 to 135 angle coverage is :(100,150),(340,380),(520,560) 
 """
interval_for_angle_measurement=10
measurement_intervals=[(100,140),(340,380),(580,620)]
linear_velocity_x=0.4
rate=rospy.Rate(10)
data=LaserScan()
pub=rospy.Publisher('/cmd_vel',Twist,queue_size=1)
sub=rospy.Subscriber('/scan',LaserScan,callback)
rate.sleep()
rospy.spin()


