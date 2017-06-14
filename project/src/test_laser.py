#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
def callback(msg):

    data=msg.ranges
    check=[data[719],data[360],data[0]]
    rospy.loginfo(check)
    if check[0]>2 or check[2]>2:
      rospy.sleep(1000)

rospy.init_node('test_node')
rate=rospy.Rate(20)
sub=rospy.Subscriber('/bot_0/laser/scan',LaserScan,callback)
rate.sleep()
rospy.spin()