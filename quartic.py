import numpy as  np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d
l=5
h=4
u=0.3

limit=10
fig = plt.figure()
ax=fig.add_subplot(111,projection='3d')
ax.set_xlim(-limit,limit)
ax.set_ylim(-limit,limit)
ax.set_zlim(-limit,limit)
ax.set_xlabel('"X')
ax.set_ylabel("Y")
ax.set_zlabel("Z")

"""
d=np.matrix([[0,l/2,l,10/np.sqrt(2),10/np.sqrt(2)],
            [0,0,0,0,0],
            [0,h,0,10/np.sqrt(2),-10/np.sqrt(2)]])

d=np.matrix([[0,l,l/2,10,10],
            [0,0,0,0,0],
            [0,0,h,10,-10]])

b=np.matrix([[-8,-8,16,-2,2],
            [18,14,-32,5,-3],
            [-11,-5,16,-1,1],
            [0,1,0,0,0],
            [1,0,0,0,0]])
f=np.matrix([u**4,u**3,u**2,u,1])"""

m=np.matrix([[2,-2,1,1]
            ,[-3,3,-2,-1]
            ,[0,0,1,0]
            ,[0,0,0,0]])
b=np.matrix([[0,5,6/np.sqrt(2),6/np.sqrt(2)]
            ,[1,1,0,0]
            ,[1,1,6/np.sqrt(2),-6/np.sqrt(2)]])

#fig=plt.figure()
#ax=plt.axes(projection='3d')

for u in np.arange(0,1,0.01):
    Y = np.matrix([u ** 3, u ** 2, u, 1])
    df=Y@m@np.transpose(b)
    ax.scatter(df.item(0),df.item(1),df.item(2),cmap='Greens')
    print(df.item(0),df.item(1),df.item(2))
    if u<0.5:
        print(df)

#fig=plt.figure()
#ax=fig.add_subplot(111,projection='3d')
#bf = np.matmul(f, b)
#print(bf)


#ax.scatter(df.item(0), df.item(1), df.item(2), cmap='Greens')
"""

for u in np.arange(0,1,0.01):
    f=np.matrix([u**4,u**3,u**2,u,1])
    bf = np.matmul(f, b)
    df=np.matmul(d,bf.transpose())
    ax.scatter(df.item(0),df.item(1),df.item(2),cmap='Greens')
    if u < 0.5:
        print(df)    

    ax.scatter(df.item(0),df.item(1),df.item(2),cmap='Greens')
    if u<0.5:
        print(df)
plt.show()
"""



ax.scatter(0,0,0,cmap='Greens')
ax.scatter(5,5,5,cmap='Greens')

plt.show()





