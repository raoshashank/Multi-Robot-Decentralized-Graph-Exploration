#!/usr/bin/env python
import rospy
from project.srv import direction,directionRequest,directionResponse
from random import randint

rospy.init_node("turn_service_client")


def callback(request):
    rospy.loginfo("Entered Service Callback")
    check=request.check
    response=directionResponse()
    
    while True:
            i=randint(0,2)
            if check[i]>3:
                break

    rospy.loginfo("Value of i is : "+str(i))
    
    if i==0:
       rospy.loginfo("Turning left")
       response.response="L"

    elif i==1:
        rospy.loginfo("going forward")
        response.response="F"
    elif i==2:
         rospy.loginfo("Turning Right")
         response.response="R"
         
    return response

######THIS COMMENT IS THE CHANGE!


rospy.loginfo("Entered Service")
while not rospy.is_shutdown():
    rospy.Service('/turn_service',direction,callback)
    rospy.spin()