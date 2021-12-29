import matplotlib.pyplot as plt
import numpy as np
import mpl_toolkits.mplot3d
from math import sin, cos
import math

l = 8
b = 4

limit=10
fig = plt.figure()
ax=fig.add_subplot(111,projection='3d')
ax.set_xlim(-limit,limit)
ax.set_ylim(-limit,limit)
ax.set_zlim(-limit,limit)
ax.set_xlabel('"X')
ax.set_ylabel("Y")
ax.set_zlabel("Z")

o=np.matrix([[0,0,0,1]])

FR = np.matrix([[1, 0, 0, l / 2],
                [0, 0, 0, b / 2],
                [0, 0, 0, 0],
                [0, 0, 0, 1]])
bl = np.matrix([[1, 0, 0, -l / 2],
                [0, 1, 0, -b / 2],
                [0, 0, 1, 0],
                [0, 0, 0, 1]])
br = np.matrix([[1, 0, 0, l / 2],
                [0, 0, 0, -b / 2],
                [0, 0, 0, 0],
                [0, 0, 0, 1]])
fl = np.matrix([[1, 0, 0, -l / 2],
                [0, 1, 0, b / 2],
                [0, 0, 1, 0],
                [0, 0, 0, 1]])

T = np.matrix([[1, 0, 0, 3],
                [0, 1, 0, 3],
                [0, 0, 1, 0],
                [0, 0, 0, 1]])

def dh(q, a, d, t):

    z = np.matrix([[np.cos(math.radians(t)), -np.sin(math.radians(t)), 0, q],
                  [np.sin(math.radians(t)) * np.cos(math.radians(a)), np.cos(math.radians(t)) * np.cos(math.radians(a)),
                   -np.sin(math.radians(a)), -d * np.sin(math.radians(a))],
                  [np.sin(math.radians(t)) * np.sin(math.radians(a)), np.cos(math.radians(t)) * np.sin(math.radians(a)),
                   np.cos(math.radians(a)), d * np.cos(math.radians(a))],
                  [0, 0, 0, 1]])
    return z


def point(x):
    df=x@np.transpose(o)
    ax.scatter(df.item(0),df.item(1),df.item(2),cmap='Greens')
    return df

"""def lin_plot(x,y):
    a=point(x)
    b=point(y)
    plt.plot([a.item(0),b.item(0)],[a.item(1),b.item(1)],[a.item(2),b.item(2)],linewidth=0.5)

lin_plot(FR,fl)
lin_plot(fl,bl)
lin_plot(br,FR)
lin_plot(br,bl)"""



def rotx(alpha):
    """
    Create a 3x3 rotation matrix about the x axis
    """
    rx = np.array([[1, 0,          0,        0 ],
                   [0, cos(alpha), -sin(alpha),0],
                   [0, sin(alpha), cos(alpha),0 ],
                   [0,0,0,1]])

    return rx


def roty(beta):
    """
    Create a 3x3 rotation matrix about the y axis
    """
    ry = np.array([[cos(beta),   0, sin(beta),0],
                   [0,           1, 0        ,0],
                   [-sin(beta),  0, cos(beta),0],
                   [0,0,0,1]])

    return ry


def rotz(gamma):
    """
    Create a 3x3 rotation matrix about the z axis
    """
    rz = np.array([[cos(gamma), -sin(gamma), 0,0],
                   [sin(gamma), cos(gamma),  0,0],
                   [0,          0,           1,0],
                   [0,0,0,1]])

    return rz


def rotxyz(alpha, beta, gamma):
    """
    Create a 3x3 rotation matrix about the x,y,z axes
    """
    return rotx(alpha).dot(roty(beta)).dot(rotz(gamma))

l1=2
l2=4
l3=4
q=rotxyz(0,3.14/2,0)
"""

q=rotxyz(3.14/2,0,0)
print(q)
print("------")"""
theta1=0
theta2=45
tehta3=90

fg=dh(0,theta1,0,0)
print(fg)

b=dh(l2,0,0,90)
c=dh(l3,0,0,45)

def lin_plot(x,y):
    a=x
    b=y
    plt.plot([a.item(0),b.item(0)],[a.item(1),b.item(1)],[a.item(2),b.item(2)],linewidth=0.5)

for i in range(1):
    fg=dh(0,theta1,0,0)
    a = dh(0, -90, 2, 45)
    print("==============")
    print(fg)
    print("//////////////////////////////////----------->")
    print(fg@np.transpose(o))

    print("-------")
    z = point(fg)
    print("88888888888888888888888888")
    print(z)
    print("]]]]]]]]]]]")
    print(lin_plot(o, z))
    print("----------")

    lin_plot(o,point(fg))
    lin_plot(point(fg),point(fg @ a))
    lin_plot(point(fg@a),point(fg @ a @ b))
    lin_plot(point(fg @ a @ b),point(fg @ a @ b @ c))
    h=fg @ a @ b @ c
    print(np.around(h))
    print(h.item(3))
    print(h.item(7))
    print((h.item(11)))

"""for i in range(4):
    a = dh(0, -90, 2, i*20)
    print(np.round(point(a), 2))
    print(point(a @ b))
    print(point(a @ b @ c))"""
ax.scatter(0,2,-6,cmap='Greens',linewidths=10)
plt.show()