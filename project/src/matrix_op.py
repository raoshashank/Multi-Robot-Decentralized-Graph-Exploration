#!/usr/bin/env python
import rospy 
import numpy as np

class matrix_op:
   def non_zero_element_sign(I):
       for i in I:
           if i!=0:
               if i>0:
                   return 0 
                else return 1
                
   def non_zero_element_count(I):
       count=0
       for i in I:
           if i==0:
               count+=1
       return count


   def out(self,matrix):
    out_edge=np.zeros((matrix.shape[0],0))
    count=0    
    for i in range(matrix.shape[1]):
        count=0
        for j in range(matrix.shape[0]):
            if matrix[j][i]<0:
                count+=1
        if count==1:
            out_edge=np.column_stack((out_edge,matrix[:,i]))

    return out_edge
    
   def completed(self,matrix):
    completed_edges=np.zeros((matrix.shape[0],0))
    count=0    
    for i in range(matrix.shape[1]):
        count=0
        for j in range(matrix.shape[0]):
            if matrix[j][i]>0:
                count+=1
        if count==2:
            completed_edges=np.column_stack((completed_edges,matrix[:,i]))
    
    return completed_edges
    
    
   def unexplored(self,matrix):
    unexplored_edge=np.zeros((matrix.shape[0],0))
    count=0    
    for i in range(matrix.shape[1]):
        count=0
        for j in range(matrix.shape[0]):
            if matrix[j][i]>0:
                count+=1
        if count==1:
            unexplored_edge=np.column_stack((unexplored_edge,matrix[:,i]))
    
    return unexplored_edge

   def inci_to_adj(self,I):
    R=I.shape[0]
    C=I.shape[1]
    temp=[]
    adj=np.zeros((R,R))
    i=0
    j=0
    for c in range(0,C):
     for r in range(0,R):
         if I[r][c]!=0:
             temp.append(r)
    
     i=temp[0]
     j=temp[1]
     adj[i][j]=1
     adj[j][i]=1
     temp=[]
    return adj 
   
   def get_vertex_tag(tags_I,i):
    return tags_I[i]




   def merge_matrices(self,I1,tags_I1,I2,tags_I2):   
    global E1cap,E2cap,Vcap
    V1=I1.shape[0]
    if len(I1.shape)!=1:
        E1=I1.shape[1]
    else:
        E1=0
    V2=I2.shape[0]

    if len(I2.shape)!=1:
        E2=I2.shape[1]
    else:
        E2=0
    E1cap=E1
    E2cap=E2
    Vcap=V1+V2
    delj=[]
    deli=[]
    
    tags_I1.append(tags_I2)
    tags=tags_I1

    ###I=[[I1,0],[0,I2]]
    I=np.row_stack((np.column_stack((I1,np.zeros((V1,E2)))),np.column_stack((np.zeros((V2,E1)),I2))))
    
    for i1 in range(0,V1):
     for j1 in range(0,E1):
        for i2 in range(V1,V1+V2):
          for j2 in range(E1,E1+E2): 
            if tags[i1]==tags[i2] and np.absolute(I[i1,j1])==np.absolute(I[i2,j2]) :
              if np.sign(I[i1,j1])!=np.sign(I[i2,j2]) :
                  I[i2,j2]=-np.absolute(I[i1,j1])
                  I[i1,j1]=I[i2,j2]
              if non_zero_element_count(I[V1:(V1+V2),j2])==2 :
                  delj.append(j1)
                  E1cap-=1
              else:
                  if non_zero_element_count(I[0:V1,j1])==2:
                      delj.append(j2)
                      E2cap-=1
                  else:
                      delj.append(j1)
                      E1cap-=1
                      
              if non_zero_element_count (I[V1:(V1+V2),j1]<2) :
                  I[V1:(V1+V2),j1]=I[V1:(V1+V2),j1]+I[V1:(V1+V2),j2]
              
              if non_zero_element_count(I[0:V1,j2]<2):
                  I[0:V1,j2]=I[0:V1,j2]+I[0:V1,j1]
              
              deli.append(i1)
    for i in deli:
        delete_row(I,i)
    for j in delj:
        delete_column(I,j)   
    return [I,Vcap,E1cap,E2cap]  

   def Order_Matrix(self,I_Merged):
        global E1cap,E2cap,Vcap
        I1=I_Merged[:,1:E1cap]
        I2=I_Merged[:,(E1cap+1):(E1cap+E2cap)]
        I1_completed=self.completed(I1)
        I2_completed=self.completed(I2)
        I1_out=self.out(I1)
        I2_out=self.out(I2)
        I1_unexplored=self.unexplored(I1)
        I2_unexplored=self.unexplored(I2)
        I_ordered=np.column_stack((I1_completed,I2_completed,I1_out,I2_out,I1_unexplored,I2_unexplored)) 
        return I_ordered   

   def first_step_on_vertex_visit(self,I_V,I_R,I_dash):
       ##TO DO 
        I_double_dash=self.merge_matrices(I_dash,I_R)
        I=self.merge_matrices(I_double_dash,I_V)
        ordered=self.Order_Matrix(I)
        return ordered



   