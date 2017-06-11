#!/usr/bin/env python
import rospy
from project.srv import direction,directionRequest,directionResponse
from project.srv import dirturn,dirturnRequest,dirturnResponse
from random import randint
from matrix_op import matrix_op

## second step on vertex visit has to be done here
##finally should have a list of next turns at vertices.
rospy.init_node("direction_service_node")


def callback(request):
    check=request.check
    resp=directionResponse()
    resp.response=[]
    ###Choosing which direction to go
    op=matrix_op()
    I=request.I
    adj=op.inci_to_adj(I)
    ###Get path from adj matrix using djikstra algo
    #TO DO
    return resp    




    
""" 
    while True:
            i=randint(0,2)
            if check[i]>range_threshold:
                break

    
    if i==0:
       resp.response="L"

    elif i==1:
        resp.response="F"
    elif i==2:
         resp.response="R"
"""


   



range_threshold=3
while not rospy.is_shutdown():
    rospy.Service('/direction_service_server',direction,callback)
    rospy.spin()