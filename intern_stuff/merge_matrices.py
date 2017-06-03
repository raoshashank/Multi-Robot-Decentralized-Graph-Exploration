# -*- coding: utf-8 -*-
"""
Created on Wed May 17 19:09:48 2017

@author: shashank
"""
import numpy as np
import scipy as sp
#import random package
#how will angles be generated?
#def vertex_tag(matrix,rno):
    
def out(matrix):
    out_edge=[]
    count=0    
    for i in range(matrix.shape[1]):
        count=0
        for j in range(matrix.shape[0]):
            if matrix[j][i]<0:
                count+=1
        if count==1:
            out_edge.append(i)
    
    return out_edge
    
def completed(matrix):
    completed_edges=[]
    count=0    
    for i in range(matrix.shape[1]):
        count=0
        for j in range(matrix.shape[0]):
            if matrix[j][i]>0:
                count+=1
        if count==2:
            completed_edges.append(i)
    
    return completed_edges
    
    
def unexplored(matrix):
    unexplored_edge=[]
    count=0    
    for i in range(matrix.shape[1]):
        count=0
        for j in range(matrix.shape[0]):
            if matrix[j][i]>0:
                count+=1
        if count==1:
            unexplored_edge.append(i)
    
    return unexplored_edge
    
    
def get_vertex_tag(matrix,rno):
    return 0
    
def delete_row(matrix,rno):
    matrix=sp.delete(matrix,rno-1,0)
    return matrix
    
def delete_column(matrix,cno):
    matrix=sp.delete(matrix,cno-1,1)
    return matrix
    
def non_zero_element_count(matrix):
    count=0
    for i in matrix:
        if i!=0:
            count+=1;
            
    return count

E1cap=0
E2cap=0
Vcap=0


def make(I1,I2):
    V1=I1.shape[0]
    E1=I1.shape[1]
    V2=I2.shape[0]
    E2=I2.shape[1]
    for i in range(V1+V2):
      for j in range(E1+E2):
          if i<V1 and j<E1:
              I[i][j]=I1[i][j]
              
          if i>V1 and j>E1:
              I[i][j]=I2[i-V1][j-E1]
              
          else: 
            I[i][j]=0
    
    return I
              
          

    
##Algorithm 1
def merge_matrices(I1,I2):   
    V1=I1.shape[0]
    E1=I1.shape[1]
    V2=I2.shape[0]
    E2=I2.shape[1]
    E1cap=E1
    E2cap=E2
    Vcap=V1+V2
    delj=[]
    deli=[]
    ###I=[[I1,0],[0,I2]]
    I=make(I1,I2)
    for i1 in range(1,V1):
     for j1 in range(1,E1):
        for i2 in range(V1+1,V1+V2):
          for j2 in range(E1+1,E1+E2): 
            if get_vertex_tag(I,i1)==get_vertex_tag(I,i2) and np.absolute(I[i1,j1])==np.absolute(I[i2,j2]) :
              if np.sign(I[i1,j1])!=np.sign(I[i2,j2]) :
                  I[i2,j2]=-np.absolute(I[i1,j1])
                  I[i1,j1]=I[i2,j2]
              if non_zero_element_count(I[(V1+1):(V1+V2),j2])==2 :
                  delj.append(j1)
                  E1cap-=1
              else:
                  if non_zero_element_count(I[1:V1,j1])==2:
                      delj.append(j2)
                      E2cap-=1
                  else:
                      delj.append(j1)
                      E1cap-=1
                      
              if non_zero_element_count(I[(V1+1):(V1+V2),j1]<2) :
                  I[(V1+1):(V1+V2),j1]=I[(V1+1):(V1+V2),j1]+I[(V1+1):(V1+V2),j2]
              
              if non_zero_element_count(I[1:V1,j2]<2) :
                  I[1:V1,j2]=I[1:V1,j2]+I[1:V1,j1]
              
              deli.append(i1)
    for i in deli:
        delete_row(I,i)
    for j in delj:
        delete_column(I,j)   
    return I              
        
##Algorithm2
"""
def Order_Matrix(I_Merged):
    I1=I_Merged[:,1:E1cap]
    I2=I_Merged[:,(E1cap+1):(E1cap+E2cap)]
    I1_completed=completed(I1)
    I2_completed=completed(I2)
    I1_out=out(I1)
    I2_out=out(I2)
    I1_unexplored=unexplored(I1)
    I2_unexplored=unexplored(I2)
    I_ordered=
"""

if __name__ == '__main__':
    I1=np.array([[1,1,0,0,0],
                 [0,1,1,0,0],
                 [0,0,1,-1,-1]])
    I2=np.array([1,1,1],[])
    print make(I1,I2)
