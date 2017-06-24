import networkx as nx 
import numpy as np
from project.msg import vertex_info,vertices

# op=matrix_op()
# A=

v1=vertex_info()
v1.x=-5
v1.y=-5.2
v1.tag='v1'
v1.I=[]
v2=vertex_info()
v2.x=-5
v2.y=-5.2
v2.tag='v2'
v2.I=[]
A=np.array([[v1, 4, 3, 0],
             [v2,0, 0, 1]])
vert_col_A=A[:,0]
A=A[:,1:A.shape[1]]
vert_col=np.append(vert_col_A,vert_col_A)
print vert_col
A1=A
#for i in range(A.shape[1]):
B=np.row_stack((A1,A))
B=np.column_stack((vert_col,B))
#print B
B[:,1:B.shape[1]]=np.array([[0,0,0],[0,0,0],[0,0,0],[0,0,0]])
print B
# temp=vertices()
# temp.v=[]
# v1=vertex_info()
# v1.x=-5
# v1.y=-5.2
# v1.tag='v1'
# v1.I=[]
# temp.v.append(v1)
# v1=vertex_info()
# v1.x=-1.6
# v1.y=-5.2
# v1.tag='v2'
# v1.I=[]
# temp.v.append(v1)
# v1=vertex_info()
# v1.x=3.5
# v1.y=-5.2
# v1.tag='v3'
# v1.I=[]
# temp.v.append(v1)
# v1=vertex_info()
# v1.x=-1.6
# v1.y=1
# v1.tag='v4'
# v1.I=[]
# temp.v.append(v1)
# v1=vertex_info()

# v1.x=-1.6
# v1.y=5.2
# v1.tag='v5'
# v1.I=[]
# temp.v.append(v1)
# v1=vertex_info()

# v1.x=-7.6
# v1.y=1
# v1.tag='v6'
# v1.I=[]
# temp.v.append(v1)
# v1=vertex_info()

# v1.x=-7.6
# v1.y=5.2
# v1.tag='v7'
# v1.I=[]
# temp.v.append(v1)
# v1=vertex_info()

# v1.x=-7.6
# v1.y=-3.5
# v1.tag='v8'
# v1.I=[]
# temp.v.append(v1)
# v1=vertex_info()

# v1.x=7.6
# v1.y=1
# v1.tag='v9'
# v1.I=[]
# temp.v.append(v1)
# v1=vertex_info()

# v1.x=7.6
# v1.y=5.2
# v1.tag='v10'
# v1.I=[]
# temp.v.append(v1)
# v1=vertex_info()

# v1.x=7.6
# v1.y=-5.4
# v1.tag='v11'
# v1.I=[]
# temp.v.append(v1)

# I=np.zeros((7,6))
# I[0][0]=1
# I[1][0]=1
# I[1][1]=1
# I[2][1]=1
# I[2][2]=1
# I[2][3]=1
# I[3][2]=1
# I[4][3]=1
# I[4][4]=1
# I[4][5]=1
# I[5][4]=1
# I[6][5]=1
# op=matrix_op()
# adj=op.inci_to_adj(I)
# G=nx.from_numpy_matrix(adj, create_using=nx.DiGraph())
# print(nx.dijkstra_path(G, 0, 6))

