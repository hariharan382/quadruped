#! /usr/binenv python3
from logging import exception
from math import atan2, sin, sqrt
import math
from numpy.lib.polynomial import poly1d


from numpy.matrixlib.defmatrix import matrix

import numpy as np
import matplotlib.pyplot as plt

from matplotlib.animation import FuncAnimation

from mpl_toolkits.mplot3d import Axes3D

import matplotlib.pyplot as mpl
mpl.rcParams['font.size'] = 30

o=np.matrix([[0,0,0,1]])

class FK:
    def __init__(self, theta1, theta2,theta3):
        self.theta1=theta1
        self.theta2=theta2
        self.theta3=theta3


    def point(self,x):
        df=x@np.transpose(o)
        #ax.scatter(df.item(0),df.item(1),df.item(2),cmap='Greens')
        return df

    def lin_plot(self,x,y):
        a=x
        b=y
        plt.plot([a.item(0),b.item(0)],[a.item(1),b.item(1)],[a.item(2),b.item(2)],linewidth=0.5)

    def dh(self,q, a, d, t):

        z = np.matrix([[np.cos(math.radians(t)), -np.sin(math.radians(t)), 0, q],
                    [np.sin(math.radians(t)) * np.cos(math.radians(a)), np.cos(math.radians(t)) * np.cos(math.radians(a)),
                    -np.sin(math.radians(a)), -d * np.sin(math.radians(a))],
                    [np.sin(math.radians(t)) * np.sin(math.radians(a)), np.cos(math.radians(t)) * np.sin(math.radians(a)),
                    np.cos(math.radians(a)), d * np.cos(math.radians(a))],
                    [0, 0, 0, 1]])
        return z

    def fk(self,theta1, theta2, theta3):
        fg = self.dh(0, theta1, 0, 0)
        a = self.dh(0, -90, 2, theta2)
        b = self.dh(l2, 0, 0, theta3)
        c = self.dh(l3, 0, 0, 45)

        self.lin_plot(o, self.point(fg))
        self.lin_plot(self.point(fg), self.point(fg @ a))
        self.lin_plot(self.point(fg @ a), self.point(fg @ a @ b))
        self.lin_plot(self.point(fg @ a @ b), self.point(fg @ a @ b @ c))
        #print(fg @ a @ b @ c @np.transpose(o))
        q=fg @ a @ b @ c @np.transpose(o)
        x1=q.item(0)
        y1=q.item(1)
        z1=q.item(2)
        return x1,y1,z1


def point(x):
    df=x@o
    #print(df)
    ax.scatter(df.item(0),df.item(1),df.item(2),cmap='Greens')
    #print(df.item(0),"------",df.item(1),"------",df.item(2))
    return df

def dh(q, a, d, t):

    z = np.matrix([[np.cos(math.radians(t)), -np.sin(math.radians(t)), 0, q],
                  [np.sin(math.radians(t)) * np.cos(math.radians(a)), np.cos(math.radians(t)) * np.cos(math.radians(a)),
                   -np.sin(math.radians(a)), -d * np.sin(math.radians(a))],
                  [np.sin(math.radians(t)) * np.sin(math.radians(a)), np.cos(math.radians(t)) * np.sin(math.radians(a)),
                   np.cos(math.radians(a)), d * np.cos(math.radians(a))],
                  [0, 0, 0, 1]])
    return z

def ik(x, y, z):
    p=math.sqrt(z**2+y**2)
    phi=math.degrees(math.atan2(y,z))
    q=math.sqrt(p**2-l1**2)

    D=(z**2+q**2-l2**2-l3**2)/(2*l2*l3)
    theta1=phi+math.degrees(atan2(q,l1))-90
    theta3=math.degrees(math.atan2(math.sqrt(1-D**2),D))
    theta2=math.degrees(math.atan2(l3*math.sin(math.radians(theta3)),l2+l3*math.cos(math.radians(theta3)))) - math.degrees(math.atan2(x,q))
    
    
  

    return theta1,theta2,theta3

l1=2
l2=4
l3=4

limit=10
fig = plt.figure()
ax=fig.add_subplot(111,projection='3d')
ax.set_xlim(-limit,limit)
ax.set_ylim(-limit,limit)
ax.set_zlim(-limit,limit)
ax.set_xlabel('X')
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.quiver(0, 0, 0, 0, -3, 0, 
 arrow_length_ratio=0.1)
ax.quiver(0, 0, 0, 3, 0, 0, 
 arrow_length_ratio=0.1)
ax.quiver(0, 0, 0, 0, 0, 3, 
 arrow_length_ratio=0.1)


h=2.5
s=8

def rotz(gamma):
    rz = np.matrix([[np.cos(gamma), -np.sin(gamma), 0,gp[0]],
                   [np.sin(gamma), np.cos(gamma), 0,gp[1]],
                   [0, 0, 1,gp[2]],
                   [0,0,0,1]])
    
    return rz

mat = np.matrix([[1, -6, 15, -20, 15, -6, 1],
                 [-6, 30, -60, 60, -30, 6, 0],
                 [15, -60, 90, -60, 15, 0, 0],
                 [-20, 60, -60, 20, 0, 0, 0],
                 [15, -30, 15, 0, 0, 0, 0],
                 [-6, 6, 0, 0, 0, 0, 0],
                 [1, 0, 0, 0, 0, 0, 0]])
#ang=45
z=FK(0,45,90)
gp=z.fk(0,45,90)

gp=np.round(gp,3)   
"""
print(type(hp))
print(ik(hp[0],hp[1],hp[2]))
plt.scatter(hp[0],hp[1],hp[2],linewidths=5, c='b')"""
print("-----------------------------")
#///////////////   ANGLE    ///////////////////////
hp=[0,0,0]
r_z=rotz(np.radians(22.5))
print("r_z :   ",r_z)
p1=np.matrix([[hp[0],hp[1],hp[2],1],
              [hp[0]-s/2,hp[1],hp[2],1],
              [hp[0]-s/2,hp[1],hp[2]+3*h/4,1],
              [hp[0],hp[1],hp[2]+h,1],
              [hp[0]+(s/2)+1,hp[1],hp[2]+h+1,1],
              [hp[0]+s/2,hp[1],hp[2],1],
              [hp[0],hp[1],hp[2],1]])
T=np.matrix([[1,0,0,gp[0]],
             [0,1,0,gp[1],
             [0,0,1,gp[2]],
             [0,0,0,1]]])
point_matrix=r_z@np.transpose(p1)
print(np.transpose(p1))
print("/////////////////////////////////")
print(point_matrix)


for t in np.arange(0,1,0.05):
    p = np.matrix([t ** 6, t ** 5, t ** 4, t ** 3, t ** 2, t, 1])

    x1=p@mat@np.transpose(point_matrix[0,:])
    y1=p@mat@np.transpose(point_matrix[1,:])
    z1=p@mat@np.transpose(point_matrix[2,:])
    #print("x1:  ",x1,"  ","y1:  ",y1,"z1  ",z1)

       
    
    ax.scatter(np.asscalar(x1),np.asscalar(y1),np.asscalar(z1),cmap='Greens')
    theta1,theta2,theta3=ik(np.asscalar(x1),np.asscalar(y1),np.asscalar(z1))
    #print("x:",np.asscalar(x1),"y: ",np.asscalar(y1),"z: ",np.asscalar(z1))

    g = FK(round(theta1,0),round(theta2,0),round(theta3,0))
    g.fk(theta1, theta2,theta3)

ax.scatter(5,5,5,cmap='Greens')

plt.tight_layout()
plt.show()
    

