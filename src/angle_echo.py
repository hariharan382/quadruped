#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float64

def callback1(data):
    pub = rospy.Publisher('angle_1_topic', Float64, queue_size=10)
    #rospy.init_node('angle_1', anonymous=True)
    x=np.degrees(data.data)
    pub.publish(np.round(x,0))
def callback2(data):
    pub = rospy.Publisher('angle_2_topic', Float64, queue_size=10)
    #rospy.init_node('angle_2', anonymous=True)
    x=np.degrees(data.data)
    x=abs(x)
    pub.publish(np.round(x,0))
def callback3(data):
    pub = rospy.Publisher('angle_3_topic', Float64, queue_size=10)
    #rospy.init_node('angle_3', anonymous=True)
    x=np.degrees(data.data)
    pub.publish(np.round(x,0))

    #print("third angle    :",x)
    
def listener():


    rospy.init_node('angle_echo', anonymous=True)

    rospy.Subscriber("/dynamo/LFJ1_position_controller/command", Float64, callback1)
    rospy.Subscriber("/dynamo/LFJ2_position_controller/command", Float64, callback2)
    rospy.Subscriber("/dynamo/LFJ3_position_controller/command", Float64, callback3)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
