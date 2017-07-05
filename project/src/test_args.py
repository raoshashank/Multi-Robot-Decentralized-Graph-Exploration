#!/usr/bin/env python
import rospy
import sys

rospy.init_node('test')
if __name__=='__main__':
    myargv=rospy.myargv(argv=sys.argv)
    