#!/usr/bin/env python
import rospy
import numpy as np
from matrix_op import matrix_op
from math import pi

op=matrix_op()
I1=np.array([[-pi,-pi/2]])
tags_I1=['v1']
I2=np.array([[2*pi,pi/2,pi]])
tags_I2=['v2']
op.merge_matrices(I1,tags_I1,I2,tags_I2)
