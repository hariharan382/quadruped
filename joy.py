#! /usr/binenv python3
from math import sin
import rospy
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import Joy
from matplotlib.animation import FuncAnimation

#ani=FuncAnimation(plt.gcf,animate,1000)

m=np.matrix([[2,-2,1,1]
            ,[-3,3,-2,-1]
            ,[0,0,1,0]
            ,[0,0,0,0]])


def animate(u):
    for u in np.arange(0,1,0.01):
    Y = np.matrix([u ** 3, u ** 2, u, 1])
    df=Y@m@b.transpose()
    ax.scatter(df.item(0),df.item(1),df.item(2),cmap='Greens')


def callback(data):
    front_back=data.axes[1]
    right_left=-data.axes[0]
    len=data.axes[2]
    ang=np.arctan2(front_back,right_left) * 180 / np.pi
    h=5
    if(ang <0):
        ang=360+ang
    
    rospy.loginfo("%f",ang)

    b=np.matrix([[0,h*len*cos(np.radians(ang)),cos(np.radians(ang))/cos(np.radians(45)),6/np.sqrt(2)]
            ,[0,h*len*sin(np.radians(ang)),cos(np.radians(ang))/cos(np.radians(45)),6/np.sqrt(2)]
            ,[0,0,cos(np.radians(45)),-6/np.sqrt(2)]])
    ani=FuncAnimation(plt.gcf,animate,1000)



def listener():
    rospy.init_node("joy_listener_node",anonymous=True)
    rospy.Subscriber("/joy",Joy,callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
plt.show()