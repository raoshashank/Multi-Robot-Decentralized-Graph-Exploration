#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
def callback(msg):
    data=msg.ranges
    rospy.loginfo("Callback1")
    left_slice=np.asarray(data[0:10])
    left_avg=left_slice.sum()/len(left_slice)
    mid_slice=np.asarray(data[355:365])
    mid_avg=mid_slice.sum()/len(mid_slice)
    right_slice=np.asarray(data[710:720])
    right_avg=right_slice.sum()/len(right_slice)
    rospy.loginfo(right_avg)
    check=[left_avg,mid_avg,right_avg]
    #rospy.loginfo("Values i found are:"+str(check))    
    count=0
    ###Check for node position###
    for i in check:
        if i>3:
            count+=1
""" 
    if count>=2 :
        rospy.loginfo("I'M AT NODE!") 
    else:
        rospy.loginfo("I'm not") 
"""
 

rospy.init_node('test_node')
rate=rospy.Rate(20)
sub=rospy.Subscriber('/bot_0/laser/scan',LaserScan,callback)
rate.sleep()
rospy.spin()