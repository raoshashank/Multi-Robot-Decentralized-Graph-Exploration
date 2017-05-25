#!/usr/bin/env python
import rospy
from project.srv import direction,directionRequest,directionResponse
from project.srv import dirturn,dirturnRequest,dirturnResponse
from random import randint

rospy.init_node("direction_service_node")


def callback(request):
    rospy.loginfo("Entered Service Callback")
    check=request.check
    resp=directionResponse()
    
    ###Choosing which direction to go...Currently threshold of 3 metres
    while True:
            i=randint(0,2)
            if check[i]>range_threshold:
                break

    rospy.loginfo("Value of i is : "+str(i))
    
    if i==0:
       rospy.loginfo("Turning Left")
       resp.response="L"

    elif i==1:
        rospy.loginfo("going forward")
        resp.response="F"
    elif i==2:
         rospy.loginfo("Turning Right")
         resp.response="R"
         
    return resp



rospy.loginfo("Entered Service")
range_threshold=3
while not rospy.is_shutdown():
    rospy.Service('/direction_service_server',direction,callback)
    rospy.spin()