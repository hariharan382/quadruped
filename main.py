#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np
from math import sin, cos
import math
import rospy
from std_msgs.msg import Int32
import time

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

mat = np.matrix([[1, -6, 15, -20, 15, -6, 1],
                 [-6, 30, -60, 60, -30, 6, 0],
                 [15, -60, 90, -60, 15, 0, 0],
                 [-20, 60, -60, 20, 0, 0, 0],
                 [15, -30, 15, 0, 0, 0, 0],
                 [-6, 6, 0, 0, 0, 0, 0],
                 [1, 0, 0, 0, 0, 0, 0]])
h = 3
s = 5
points = np.matrix([[0, 2, -5.65],
                    [0 + (s / 2), 2, -5.65],
                    [0 + (s / 2), 2, -5.65 + h],
                    [0, 2, -5.65 + h],
                    [0 - (s / 2), 2, -5.65 + h],
                    [0 - (s / 2), 2, -5.65],
                    [0, 2, -5.65],
                    ])

def talker():
    LF1 = rospy.Publisher('LF1_angle_topic1', Int32, queue_size=10)
    LF2 = rospy.Publisher('LF2_angle_topic2', Int32, queue_size=10)
    LF3 = rospy.Publisher('LF3_angle_topic3', Int32, queue_size=10)
    LB1 = rospy.Publisher('LB1_angle_topic1', Int32, queue_size=10)
    LB2 = rospy.Publisher('LB2_angle_topic2', Int32, queue_size=10)
    LB3 = rospy.Publisher('LB3_angle_topic3', Int32, queue_size=10)
    RF1 = rospy.Publisher('RF1_angle_topic1', Int32, queue_size=10)
    RF2 = rospy.Publisher('RF2_angle_topic2', Int32, queue_size=10)
    RF3 = rospy.Publisher('RF3_angle_topic3', Int32, queue_size=10)
    RB1 = rospy.Publisher('RB1_angle_topic', Int32, queue_size=10)
    RB2 = rospy.Publisher('RB2_angle_topic2', Int32, queue_size=10)
    RB3 = rospy.Publisher('RB3_angle_topic3', Int32, queue_size=10)

    rospy.init_node('sender', anonymous=True)

    while(True):
        def left(x):
            for t in np.arange(0, 1, 0.05):
            p = np.matrix([t ** 6, t ** 5, t ** 4, t ** 3, t ** 2, t, 1])
            x1 = p @ mat @ x
            y1 = p @ mat @ y
            z1 = p @ mat @ z
            theta1, theta2, theta3 = ik(np.asscalar(x1), np.asscalar(y1), np.asscalar(z1))
            
            if x=='F':
                LF1.publish(np.int16(theta1))
                theta1=np.round(theta1,1)
                theta2=np.round(theta2,1)
                theta3=np.round(theta3,1)
                Y=45
                if(Y-theta2 != 0):
                    LF2.publish(np.int16(190-theta2))
                    LF3.publish(np.int16(theta3-(Y-theta2)-30))
                    Y=theta2
                rospy.sleep(0.05)
                
            else :
                LB1.publish(np.int16(theta1))
                theta1=np.round(theta1,1)
                theta2=np.round(theta2,1)
                theta3=np.round(theta3,1)
                Y=45
                if(Y-theta2 != 0):
                    LB2.publish(np.int16(190-theta2))
                    LB3.publish(np.int16(theta3-(Y-theta2)-30))
                    Y=theta2
        def right(x):
            for t in np.arange(0, 1, 0.05):
            p = np.matrix([t ** 6, t ** 5, t ** 4, t ** 3, t ** 2, t, 1])
            x1 = p @ mat @ x
            y1 = p @ mat @ y
            z1 = p @ mat @ z
            theta1, theta2, theta3 = ik(np.asscalar(x1), np.asscalar(y1), np.asscalar(z1))
            if x=='F':
                RF1.publish(np.int16(190-theta1))
                theta1=np.round(theta1,1)
                theta2=np.round(theta2,1)
                theta3=np.round(theta3,1)
                Y=45
                if(Y-theta2 != 0):
                    RF2.publish(np.int16(theta2))
                    RF3.publish(np.int16(theta3-(Y-theta2)-30))
                    Y=theta2
                rospy.sleep(0.05)
                
            else :
                RB1.publish(np.int16(190theta1))
                theta1=np.round(theta1,1)
                theta2=np.round(theta2,1)
                theta3=np.round(theta3,1)
                Y=45
                if(Y-theta2 != 0):
                    RB2.publish(np.int16(190-theta2))
                    RB3.publish(np.int16(theta3-(Y-theta2)-30))
                    Y=theta2



        
            
     

            #rospy.loginfo("%s ---- %s ---- %s",theta1,190-theta2,theta3)

            rate = rospy.Rate(0.01) # 10hz



    rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
