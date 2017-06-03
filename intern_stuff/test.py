# -*- coding: utf-8 -*-
"""
Created on Wed May 17 22:04:53 2017

@author: shashank
"""
import numpy as np
import scipy as sp
def del_row(matrix,rno):
    matrix=sp.delete(matrix,rno-1,0)
    return matrix
    
def del_column(matrix,cno):
    matrix=sp.delete(matrix,cno-1,1)    
    return matrix
    
def non_zero_element_count(matrix,cno):
    count=0
    for i in matrix[:,cno-1]:
        if(i!=0):
            count+=1;
            
    return count



a=np.array([[0,2,0],[0,5,6],[0,8,9]])
b=np.array([[1,2,3],[4,5,6],[7,8,9]])
c=np.array([[a,0],[0,b]])
d=np.array([[1.0,2.0,3.0],[10,2,3]])
c=np.concatenate((a,b.T),axis=1)

