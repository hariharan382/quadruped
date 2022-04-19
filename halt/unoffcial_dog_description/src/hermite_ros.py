#!/usr/bin/env python
import rospy
import numpy as np
import time
import matplotlib.pyplot as plt
from std_msgs.msg import Int32
from math import atan2,atan,sqrt

M=np.array([[-8,     18, -11,      0, 1],
            [-8,     14,  -5,      0, 0],
            [   16,    -32,  16,      0, 0],
            [   -1,    2.5,  -2,    0.5, 0],
            [    2,     -3,   1,      0, 0]]) 

stepLength=7

l1=2
l2=12
l3=12
h=5
off1=2

def point_matrix(P1,theta):
    right = 1
    x=10
    theta=90-theta
    P3 = np.array([h, stepLength * np.cos(np.radians(theta)) * 0.5, stepLength * np.sin(np.radians(theta)) * 0.5])
    P2 = np.array([0, stepLength * np.cos(np.radians(theta)), stepLength * np.sin(np.radians(theta))])
    T1 = np.array([np.cos(np.radians(45)), -x * np.cos(np.radians(theta)) * np.sin(np.radians(45)),
                   -x * np.sin(np.radians(theta)) * np.sin(np.radians(45))])
    T2 = np.array([-np.cos(np.radians(45)), -x * np.cos(np.radians(theta)) * np.sin(np.radians(45)),
                   -x * np.sin(np.radians(theta)) * np.sin(np.radians(45))])

    return np.array([P1,P2,P3,T1,T2])

P1=np.array([0,0,0])


def ik(x,y,z,right = 1):
    phi = atan(abs(x) / abs(y))
    P = sqrt(x ** 2 + y ** 2)
    Q = sqrt(P ** 2 - l1 ** 2)
    if right == 1:
        theta1 = atan2(x,y) + atan(Q/l1)   
    else:
        a=0
        theta1 = atan(Q / l1) -np.pi -atan2(x,y)
    gamma= atan2(-Q,z-off1)
    R=sqrt((z-off1)**2+Q**2)
    N=(R**2-l2**2-l3**2)/(2*l2*l3)
    M=(l3**2-l2**2-R**2)/(2*l2*R)
    theta2=gamma+atan2((sqrt(1-M**2)),M)
    theta3=atan2((sqrt(1-N**2)),N)
    if right == -1:
        return np.array([np.round(np.degrees(theta1), 0), np.round(np.degrees(theta2),0), np.round(np.degrees(theta3), 0)])
    if right ==1:
        return np.array([np.round(np.degrees(theta1),0), np.round(np.degrees(theta2),0), np.round(np.degrees(theta3),0)])


def hermite(theta,right):
    pub1 = rospy.Publisher('angle_1_topic', Int32, queue_size=10)
    pub2 = rospy.Publisher('angle_2_topic', Int32, queue_size=10)
    pub3 = rospy.Publisher('angle_3_topic', Int32, queue_size=10)
    rate = rospy.Rate(10)
    home=np.matrix([[1,0,0,-16.97],
                    [0,1,0,right*2],
                    [0,0,1,2],
                    [0,0,0,1]])
   
    stepLength=7
    velocity=0.16
    while not rospy.is_shutdown():
        #for i in np.arange(0,1,1):
        T = stepLength/velocity
        start = rospy.get_time()
        sl=0.3
        while (True):
            for u in np.arange(0,1.05,0.05):

                U = np.array([u ** 4, u ** 3, u ** 2, u, 1])
                B = np.dot(M, U)
                P1 = np.array([0, 0, 0])
                L=point_matrix(P1,theta)
                df = np.dot(B, L)
                df=np.array([df[0],df[1],df[2],1])
                
                fp=np.dot(home,df)
                goal=ik(fp.item(0),fp.item(1),fp.item(2),1)
                #print(goal[0],"  ",goal[1],"  ",goal[2])
                theta1=goal[0]
                theta2=goal[1]
                theta3=goal[2]
                
                Y=30
                if((Y-theta2)<=0):
                    theta1=90-theta1
                    theta2=abs(Y-theta2)+30
                    theta3=180-abs(theta3-80)-abs(Y-theta2)
                    pub1.publish(np.int32(theta1))
                    pub2.publish(np.int32(theta2))
                    pub3.publish(np.int32(theta3))
                elif(0<Y-theta2 and y-theta<30):
                    theta1=90-theta1
                    theta2=theta2+30
                    theta3=180-abs(theta3-80) + abs(Y-theta2)
                    pub1.publish(np.int32(theta1))
                    pub2.publish(np.int32(theta2))
                    pub3.publish(np.int32(theta3))
                else:
                    theta1=90-theta1
                    theta2=30
                    theta3=180-abs(theta3-80) + abs(Y-theta2)
                time.sleep(sl)
                


                if u == 1:
                    for t in np.arange(0,1.1,0.1):
                        cf=L[0][:]*(t) + L[1][:]*(1-t)
                        cf=np.array([cf[0],cf[1],cf[2],1])
                        c1=np.dot(home,cf)
                        print(c1)
                        goal=ik(c1.item(0),c1.item(1),c1.item(2),1)
                        theta1=goal[0]
                        theta2=goal[1]
                        theta3=goal[2]
                        #print(goal[0],"  ",goal[1],"  ",goal[2])
                        Y=30
                        if((Y-theta2)<=0):
                            theta1=90-theta1
                            theta2=abs(Y-theta2)+30
                            theta3=180-abs(theta3-80)-abs(Y-theta2)
                            pub1.publish(np.int32(theta1))
                            pub2.publish(np.int32(theta2))
                            pub3.publish(np.int32(theta3))
                        elif(0<Y-theta2 and Y-theta<30):
                            theta1=90-theta1
                            theta2=theta2+30
                            theta3=180-abs(theta3-80) + abs(Y-theta2)
                            pub1.publish(np.int32(theta1))
                            pub2.publish(np.int32(theta2))
                            pub3.publish(np.int32(theta3))
                        else:
                            theta1=90-theta1
                            theta2=30
                            theta3=180-abs(theta3-80) + abs(Y-theta2)

                        time.sleep(sl)
                        
            t = rospy.get_time() - start      
        
        
        
       
def listener():
    rospy.init_node("angle_publishing_node",anonymous=True)
    hermite(0,1)
    rospy.spin()


if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass