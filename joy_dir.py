#! /usr/binenv python3
from math import sin
from numpy.lib.polynomial import poly1d

from numpy.matrixlib.defmatrix import matrix
import rospy
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import Joy
from matplotlib.animation import FuncAnimation

from servo_topic.src.gait_check import rotz

h=4
s=8

def rotz(gamma):
    rz = np.array([[cos(gamma), -sin(gamma), 0],
                   [sin(gamma), cos(gamma), 0],
                   [0, 0, 1])
    return rz

mat = np.matrix([[1, -6, 15, -20, 15, -6, 1],
                 [-6, 30, -60, 60, -30, 6, 0],
                 [15, -60, 90, -60, 15, 0, 0],
                 [-20, 60, -60, 20, 0, 0, 0],
                 [15, -30, 15, 0, 0, 0, 0],
                 [-6, 6, 0, 0, 0, 0, 0],
                 [1, 0, 0, 0, 0, 0, 0]])


def callback(data):
    front_back=data.axes[1]
    right_left=-data.axes[0]
    len=data.axes[2]
    global ang=np.arctan2(front_back,right_left) * 180 / np.pi
    h=5
    if(ang <0):
        ang=360+ang

    def animate(ang):
        r_z=rotz(ang)
        p1=np.matrix([[0,s/2,0],
                      [0,s,0],
                      [0,s+1,(3*h/4)+1],
                      [0,s/2,h],
                      [0,0,(3*h/2)],
                      [0,0,0],
                      [0,s/2,0]])

        point_matrix=r_z@p1 
        p = np.matrix([t ** 6, t ** 5, t ** 4, t ** 3, t ** 2, t, 1])
        
        for t in np.arange(0,1,0.5):
            x1=p@mat@point_matrix[:,0]
            y1=p@mat@point_matrix[:,1]
            z1=p@mat@point_matrix[:,2]

            plt.plot(x1,y1,z1,label="joystick")
            plt.plt.tight_layout()

    
    rospy.loginfo("%f",ang)


ani=FuncAnimation(plt.plt.gcf(),animate,interval=10)

def listener():
    rospy.init_node("joy_listener_node",anonymous=True)
    rospy.Subscriber("/joy",Joy,callback)
    rospy.spin()

def animate(i):
    

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass