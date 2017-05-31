#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

def callback2(msg):
    global cmd_vel_cmd
    cmd_vel_cmd=msg


def stay_away():
    global feedback,left_distance,right_distance,pub,cmd_vel_cmd
    diff=left_distance-right_distance
    cmd_vel_cmd.angular.z+=0.001*diff
    pub.publish(cmd_vel_cmd)
    



def callback(msg):
    global feedback,left_distance,right_distance
    feedback=msg
    left_distance=msg.ranges[684]
    right_distance=msg.ranges[36]


def main():
    while not rospy.is_shutdown():
        stay_away()
    

if __name__ == '__main__':
    rospy.init_node('wall_dist')
    feedback=LaserScan()
    cmd_vel_cmd=Twist()
    left_distance=0
    right_distance=0
    threshold=0.5
    pub=rospy.Publisher("/bot_0/cmd_vel",Twist,queue_size=1)
    sub=rospy.Subscriber("/bot_0/laser/scan",LaserScan,callback)
    subc=rospy.Subscriber("/bot_0/cmd_vel",Twist,callback2)
    main()
    rospy.spin()