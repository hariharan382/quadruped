#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np
import mpl_toolkits.mplot3d
from math import sin, cos
import math
import rospy
from std_msgs.msg import Int32
import time
from datetime import datetime



limit = 10
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim(-limit, limit)
ax.set_ylim(-limit, limit)
ax.set_zlim(-limit, limit)
ax.set_xlabel('"X')
ax.set_ylabel("Y")
ax.set_zlabel("Z")

o = np.matrix([[0],
               [0],
               [0],
               [1]])

def point(x):
    df = x @ o
    print(df)
    ax.scatter(df.item(0), df.item(1), df.item(2), cmap='Greens')
    # print(df.item(0),"------",df.item(1),"------",df.item(2))
    return df

def dh(q, a, d, t):
    z = np.matrix([[np.cos(math.radians(t)), -np.sin(math.radians(t)), 0, q],
                   [np.sin(math.radians(t)) * np.cos(math.radians(a)),
                    np.cos(math.radians(t)) * np.cos(math.radians(a)),
                    -np.sin(math.radians(a)), -d * np.sin(math.radians(a))],
                   [np.sin(math.radians(t)) * np.sin(math.radians(a)),
                    np.cos(math.radians(t)) * np.sin(math.radians(a)),
                    np.cos(math.radians(a)), d * np.cos(math.radians(a))],
                   [0, 0, 0, 1]])
    return z

def ik(x, y, z):
    alpha = math.degrees(math.atan(y / z))
    phi = math.degrees(math.atan(x / z))
    r = math.sqrt(z ** 2 + x ** 2)
    w = r * np.cos(math.radians(phi))
    # r0=math.sqrt(h2**2-l1**2)
    h = math.sqrt(y ** 2 + z ** 2)

    P = (l1 ** 2 + h ** 2 - w ** 2) / (2 * l1 * h)
    Q = (l2 ** 2 + r ** 2 - l3 ** 2) / (2 * l2 * r)
    R = (r ** 2 - l2 ** 2 - l3 ** 2) / (2 * l2 * l3)
    theta1 = alpha + math.degrees(math.atan2(P, math.sqrt(1 - P ** 2)))
    theta2 = phi + math.degrees(math.atan2(Q, math.sqrt(1 - Q ** 2)))
    theta3 = math.degrees(math.atan2(math.sqrt(1 - R ** 2), R))
    return theta1, theta2, theta3


l1 = 2
l2 = 4
l3 = 4

m = np.matrix([[2, -2, 1, 1]
                  , [-3, 3, -2, -1]
                  , [0, 0, 1, 0]
                  , [1, 0, 0, 0]])


def lin_plot(x, y):
    a = x
    b = y
    plt.plot([a.item(0), b.item(0)], [a.item(1), b.item(1)], [a.item(2), b.item(2)], linewidth=0.5)


def rotx(alpha):
    rx = np.array([[1, 0, 0, 0],
                   [0, cos(alpha), -sin(alpha), 0],
                   [0, sin(alpha), cos(alpha), 0],
                   [0, 0, 0, 1]])
    return rx


def roty(beta):
    ry = np.array([[cos(beta), 0, sin(beta), 0],
                   [0, 1, 0, 0],
                   [-sin(beta), 0, cos(beta), 0],
                   [0, 0, 0, 1]])
    return ry


def rotz(gamma):
    rz = np.array([[cos(gamma), -sin(gamma), 0, 0],
                   [sin(gamma), cos(gamma), 0, 0],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]])
    return rz


def rotxyz(alpha, beta, gamma):
    return rotx(alpha).dot(roty(beta)).dot(rotz(gamma))


q = rotxyz(0, 3.14 / 2, 0)
fg = dh(0, 0, 0, 0)
a = dh(0, -90, 2, 45)
b = dh(l2, 0, 0, 90)
c = dh(l3, 0, 0, 45)

"""print(fg)
print("===================")
print(point(np.eye(4)))
print(o)
print(np.transpose(point(np.eye(4)))@np.transpose(o))"""

# print("/////")
w = np.array([[0, 0, 0, 1]])
z = np.matrix([[1, 0, 0, 0],
               [0, 1, 0, 0],
               [0, 0, 1, 0],
               [0, 0, 0, 1]])

"""lin_plot(point(z), point(fg))
print("------------------111111111111--------")
lin_plot(point(fg), point(fg @ a))
print("------------------22222222222-----------")
lin_plot(point(fg @ a), point(fg @ a @ b))
print("---------------33333333333--------------")
lin_plot(point(fg @ a @ b), point(fg @ a @ b @ c))
print("----------------444444444444444---------------------")
print(np.around(fg @ a @ b @ c))"""

somevariable=4

ef = fg @ a @ b @ c
st_x = ef.item(3)
st_y = ef.item(7)
st_z = np.round(ef.item(11), 2)

# print(st_x,st_y,st_z)
sl = 4

bl = np.matrix([[ef.item(3), ef.item(3) - sl, -6 / np.sqrt(2), 6 / np.sqrt(2)]
                   , [2, 2, 0, 0]
                   , [-5.66, -5.66, 6 / np.sqrt(2), -6 / np.sqrt(2)]])

mat = np.matrix([[1, -6, 15, -20, 15, -6, 1],
                 [-6, 30, -60, 60, -30, 6, 0],
                 [15, -60, 90, -60, 15, 0, 0],
                 [-20, 60, -60, 20, 0, 0, 0],
                 [15, -30, 15, 0, 0, 0, 0],
                 [-6, 6, 0, 0, 0, 0, 0],
                 [1, 0, 0, 0, 0, 0, 0]])
h = 5
s = 5
points = np.matrix([[0, 2, -5.65],
                    [0 + (s / 2), 2, -5.65],
                    [0 + (s / 2), 2, -5.65 + h],
                    [0, 2, -5.65 + h],
                    [0 - (s / 2), 2, -5.65 + h],
                    [0 - (s / 2), 2, -5.65],
                    [0, 2, -5.65],
                    ])
x = points[:, 0]
y = points[:, 1]
z = points[:, 2]
#print(z.shape)

"""for t in np.arange(0, 1, 0.1):
    p = np.matrix([t ** 6, t ** 5, t ** 4, t ** 3, t ** 2, t, 1])
    x1 = p @ mat @ x
    y1 = p @ mat @ y

    z1 = p @ mat @ z
    print(z1)

    ax.scatter(np.asscalar(x1), np.asscalar(y1), np.asscalar(z1), cmap='Greens')
    # print((df.item(0)," --- ",df.item(1)," ----- ",df.item(2)))
    #print(np.asscalar(x1), "  ", np.asscalar(y1), "  ", np.asscalar(z1))
    theta1, theta2, theta3 = ik(np.asscalar(x1), np.asscalar(y1), np.asscalar(z1))

    #g = fk.FK(round(theta1, 0), round(theta2, 0), round(theta3, 0))
    #print(round(theta1, 0), " --- ", round(theta2, 0), " ----- ", round(theta3, 0))
    #g.fk(theta1, theta2, theta3)
# print(ik(0,0,-5))"""

#plt.show()
#print("hello")

"""for u in np.arange(0,1,0.1):
    Y = np.matrix([u ** 3, u ** 2, u, 1])
    df=Y@m@np.transpose(bl)
    ax.scatter(df.item(0),df.item(1),df.item(2),cmap='Greens')
    #print((df.item(0)," --- ",df.item(1)," ----- ",df.item(2)))
    theta1,theta2,theta3=ik(df.item(0),df.item(1),df.item(2))
    z = fk.FK(round(theta1,0),round(theta2,0),round(theta3,0))
    print(round(theta1,0)," --- ",round(theta2,0)," ----- ",round(theta3,0))
    z.fk(theta1, theta2,theta3)"""

#!/usr/bin/env python



    

def talker():
    s1 = rospy.Publisher('angle_topic1', Int32, queue_size=10)
    s2 = rospy.Publisher('angle_topic2', Int32, queue_size=10)
    s3 = rospy.Publisher('angle_topic3', Int32, queue_size=10)
    s4 = rospy.Publisher('angle_topic4', Int32, queue_size=10)
    s5 = rospy.Publisher('angle_topic5', Int32, queue_size=10)
    s6 = rospy.Publisher('angle_topic6', Int32, queue_size=10)
    rospy.init_node('sender', anonymous=True)

    while(somevariable == 4):
        for t in np.arange(0, 1, 0.05):
            if(t==0):
                curr=rospy.get_time()
                
                #ti = rospy.Time.from_sec(time.time())
                #seconds = ti.to_sec()
                #print(seconds)
            p = np.matrix([t ** 6, t ** 5, t ** 4, t ** 3, t ** 2, t, 1])
            x1 = p @ mat @ x
            y1 = p @ mat @ y

            z1 = p @ mat @ z
            
            #print(z1)

            ax.scatter(np.asscalar(x1), np.asscalar(y1), np.asscalar(z1), cmap='Greens')
    # print((df.item(0)," --- ",df.item(1)," ----- ",df.item(2)))
    #print(np.asscalar(x1), "  ", np.asscalar(y1), "  ", np.asscalar(z1))
            theta1, theta2, theta3 = ik(np.asscalar(x1), np.asscalar(y1), np.asscalar(z1))
            s1.publish(np.int16(190-theta1))
            s4.publish(np.int16(190-theta1))
            #rospy.sleep(0.5)
            theta1=np.round(theta1,1)
            theta2=np.round(theta2,1)
            theta3=np.round(theta3,1)
            Y=125
            """if(Y-theta2 != 0):
                s2.publish(np.int16(190-theta2))
                s5.publish(np.int16(190-theta2))
                s3.publish(np.int16(theta3-(Y-theta2)-30))
                s6.publish(np.int16(theta3-(Y-theta2)-30))
                Y=theta2"""
            if(Y-theta2 != 0):
                s2.publish(np.int16(theta2))
                s5.publish(np.int16(theta2))
                s3.publish(np.int16(190-theta3-(Y-theta2)))
                s6.publish(np.int16(190-theta3-(Y-theta2))
                Y=theta2
            rospy.sleep(0.05)
            if (t>0.94):
                final_time=rospy.get_time()
                dur=final_time-curr
                rospy.loginfo("time taken is %s",dur)
                print(final_time,"   ",curr)

            #rospy.loginfo("%s ---- %s ---- %s",theta1,190-theta2,theta3)

            rate = rospy.Rate(0.5) # 10hz

    rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
