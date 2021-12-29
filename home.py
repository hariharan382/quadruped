#!/usr/bin/env python3
import matplotlib.pyplot as plt
import numpy as np
from math import sin, cos
import math
import rospy
from std_msgs.msg import Int32
import time





def talker():

    rospy.init_node('homing', anonymous=True)
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

    def left(x,y,z):
        LF1.publish(x)
        
        if Y=45:




   
    while(True):
        LF1.publish(0)
        LF3.publish(45)
        LF2.publish(130)
        
    #----------------------------
        LB1.publish(0)
        LB2.publish(130)
        LB3.publish(45)
    #-------------------------
        RF1.publish(180)
        RF3.publish(130)
        RF2.publish(45)
        
    #------------------------
        RB1.publish(180)
        RB2.publish(45)
        RB3.publish(130)

    rate = rospy.Rate(0.5)
    rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
