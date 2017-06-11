import numpy as np
def convert(I):
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



if  __name__ == "__main__":
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

    adj=convert(I)
    print adj
    
    