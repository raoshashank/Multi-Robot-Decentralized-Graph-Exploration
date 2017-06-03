# -*- coding: utf-8 -*-
"""
Created on Fri May 19 11:31:12 2017

@author: shashank
"""
import numpy as np

def out(matrix):
    out_edges=[]
    for i in range(matrix.shape(0)):
        for j in range(matrix.shape(1)):
            if matrix[i][j]<0:
                out_edges.append(j)
    return out_edges
    
def completed(matrix):
    completed_edges=[]
    count=0    
    for i in range(matrix.shape(0)):
        for j in range(matrix.shape(1)):
            if matrix[i][j]>0:
                count+=1;
        if count==2:
            completed_edges.append(j)
        count=0
    return completed_edges
    
    
def unexplored(matrix):
    unexplored_edges=[]
    count=0    
    for i in range(matrix.shape(0)):
        for j in range(matrix.shape(1)):
            if matrix[i][j]<0:
                count+=1
        if count==1:
            unexplored_edges.append(j)
        count=0
    return unexplored_edges
    

#def concatenate(completed_I1,completed_I2,out_I1,out_I2,unexplored_I1,unexplored_I2):

def Order_Matrix(I_merged_matr
    