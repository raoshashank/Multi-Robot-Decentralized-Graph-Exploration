#!/usr/bin/env python
import rospy 
import numpy as np

class matrix_op:
   def non_zero_element(self,I):
       i=[x for x in I if x!=0]
       return i 
                
   def non_zero_element_count(self,I):
       count=0
       for i in I:
           if i!=0:
               count+=1
       return count


   def out(self,matrix):
    #only 1 -ve element and only 1 element
    out_edge=np.zeros((matrix.shape[0],0))
    for i in range(matrix.shape[1]):
            if self.non_zero_element_count(matrix[:,i])==1 and self.non_zero_element(matrix[:,i])<0:
             out_edge=np.column_stack((out_edge,matrix[:,i]))
 
    return out_edge
    
   def completed(self,matrix):
    completed_edges=np.zeros((matrix.shape[0],0))
    for i in range(matrix.shape[1]):
            if self.non_zero_element_count(matrix[:,i])==2:
             completed_edges=np.column_stack((completed_edges,matrix[:,i]))
    
    return completed_edges
    
    
   def unexplored(self,matrix):
    unexplored_edge=np.zeros((matrix.shape[0],0))
    for i in range(matrix.shape[1]):
      if self.non_zero_element_count(matrix[:,i])==1 and self.non_zero_element(matrix[:,i])>0:
             unexplored_edge=np.column_stack((unexplored_edge,matrix[:,i]))
    
    return unexplored_edge

   def inci_to_adj(self,I):
    R=I.shape[0]
    C=I.shape[1]
    temp=[]
    adj=np.zeros((R,R))
    i=0
    j=0
    count=0
    for c in range(0,C):
     for r in range(0,R):
         if I[r][c]!=0:
             temp.append(r)
             count+=1
     if count==2:
        i=temp[0]
        j=temp[1]
        adj[i][j]=1
        adj[j][i]=1
     temp=[]
     count=0

    return adj 




   
   def merge_matrices(self,I1,I2):
    #when I_R is empty ie; at start of exploration
    if I1.shape[1]==0:
        return [I2,I2.shape[0],I2.shape[1]-1,0]

    vert_col_I1=I1[:,0]
    vert_col_I2=I2[:,0]
    I1=I1[:,1:I1.shape[1]]
    I2=I2[:,1:I2.shape[1]]
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
    I=np.row_stack((np.column_stack((I1,np.zeros((V1,E2)))),np.column_stack((np.zeros((V2,E1)),I2))))
    
    vert_col=np.append(vert_col_I1,vert_col_I2)
  
    for i1 in range(0,V1):
     for i2 in range(V1,V1+V2):     
      if vert_col[i1].tag==vert_col[i2].tag  :     
       rospy.loginfo("Found duplicate vertex:"+str(vert_col[i1].tag)) 
       for j1 in range(0,E1): 
         for j2 in range(E1,E1+E2): 
             if np.absolute(I[i1,j1])==np.absolute(I[i2,j2]):
                    if np.sign(I[i1,j1])!=np.sign(I[i2,j2]):
                        I[i2,j2]=-np.absolute(I[i1,j1])
                        I[i1,j1]=I[i2,j2]
                    if self.non_zero_element_count(I[V1:,j2])==2:
                        delj.append(j1)
                        E1cap-=1
                    else:
                        if self.non_zero_element_count(I[0:V1,j1])==2:
                            delj.append(j2)
                            E2cap-=1
                        else:
                            delj.append(j1)
                            E1cap-=1
                            
                    if self.non_zero_element_count(I[V1:,j1])<2 :
                        I[V1:,j1]=I[V1:,j1]+I[V1:,j2]
              
                    if self.non_zero_element_count(I[0:V1,j2])<2:
                        I[0:V1,j2]=I[0:V1,j2]+I[0:V1,j1]
              
                    deli.append(i1)
                    
    rospy.loginfo("delj"+str(delj))
    I=np.delete(I,(deli),axis=0)
    I=np.delete(I,(delj),axis=1)
    vert_col=np.delete(vert_col,(deli),axis=0)
    I=np.column_stack((vert_col.transpose(),I))
    Vcap=I.shape[0]
    if E1cap<0:
        E1cap=0
    if E2cap<0:
        E2cap=0
    return [I,Vcap,E1cap,E2cap] 

   def Order_Matrix(self,I_Merged,E1cap,E2cap,Vcap):
        I1=I_Merged[:,0:E1cap]
        I2=I_Merged[:,E1cap:]
        I1_completed=self.completed(I1)
        I2_completed=self.completed(I2)
        I1_out=self.out(I1)
        I2_out=self.out(I2)
        I1_unexplored=self.unexplored(I1)
        I2_unexplored=self.unexplored(I2)
        I_ordered=np.column_stack((I1_completed,I2_completed,I1_out,I2_out,I1_unexplored,I2_unexplored)) 
        Ec=I1_completed.shape[1]+I2_completed.shape[1]
        return [Ec,I_ordered]        



   