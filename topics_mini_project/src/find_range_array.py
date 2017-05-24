#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan


def callback(msg):
    print msg.ranges

rospy.init_node('find_range_array')
sub=rospy.Subscriber('/scan',LaserScan,callback)
rospy.spin()
