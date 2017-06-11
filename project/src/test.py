#!/usr/bin/env python
import rospy
from math import pi,atan2
import numpy as np
from math import pi,asin,sin
from matrix_op import matrix_op
if __name__ == '__main__':
    I1=np.array([[1,1,0,0,0],
                 [0,1,1,0,0],
                 [0,0,1,-1,-1]])
    I2=np.array([1,1,1])
    
    E1cap=0
    E2cap=0
    Vcap=0
    
    I=np.zeros((7,7))
    I[0][0]=1
    I[1][1]=1
    I[2][4]=1
    I[2][5]=1
    I[3][3]=1
    I[4][5]=1
    I[4][6]=1
    I[5][0]=1
    I[5][1]=1
    I[5][2]=1
    I[5][4]=1
    I[6][2]=1
    I[6][3]=1
    I[6][6]=1

    op=matrix_op()
    adj=op.inci_to_adj(I)
    print adj