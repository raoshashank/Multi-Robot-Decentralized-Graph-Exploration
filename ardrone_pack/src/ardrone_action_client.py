#!/usr/bin/env python
import rospy
import time
import actionlib
from ardrone_as.msg import ArdroneAction,ArdroneGoal,ArdroneResult,ArdoneFeedback
from geometry_msgs.msg import Twist

nImage=1
def feedback_callback(feedback):
    global nImage
    print('[Feedback] image n.%d recieved'%nImage)
    nImage+=1

rospy.init_node('drone_action_client')
client=actionlib.SimpleActionClient('/ardone_action_server',ArdroneAction)
client.wait_for_server()

goal=ArdroneGoal()
goal.nseconds=10
client.send_goal(goal,feedback_cb=feedback_callback)

PENDING=0
ACTIVE=1
DONE=2
WARN=3
ERROR=4


rate=rospy.Rate(10)
pub=rospy.Publisher('/cmd_vel',Twist,queue_size=1)
val=Twist()
client_state=client.get_state()

val.linear.x=0.5
pub.publish(val)
rate.sleep()
val.linear.x=0
pub.publish(val)
while not client_state<=DONE :
    val.linear.x=0.5
    pub.publish(val)
    rate.sleep()
    val.linear.x=0
    val.linear.y=0.5
    pub.publish(val)
    rate.sleep()
    val.linear.x=-0.5
    val.linear.y=0
    pub.publish(val)
    rate.sleep()
    val.linear.x=0
    val.linear.y=-0.5
    pub.publish(val)
    rate.sleep()
    client_state=client.get_state()

if client_state==3 :
    print "Warning"
else:
    if client_state==4 :
        print "ERROR in Executing Action"
    else:
        if client_state==2 :
            print "Action Done!"