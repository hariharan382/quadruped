#!/usr/bin/env python3
import numpy as np
from math import *
import math

class dynamo_kinematics:
    def __init__(self,l1=2,l2=12,l3=12,front_length=0.270,back_length=0.120,width=0.150,off1=2):
        self.l1,self.l2,self.l3,self.front_length,self.back_length,self.width,self.off1=l1,l2,l3,front_length,back_length,width,off1
    def leg_Ik(self,x,y,z,right = 1):
        #print("x:  ",x ,"y:  ",y,"z:  ",z)
        #phi = atan(abs(x) / abs(y))
        if abs(y)!=0:
            phi = atan(abs(x) / abs(y))
        else:
            phi=np.pi 

        P = sqrt(x ** 2 + y ** 2)
        Q = sqrt(P ** 2 - self.l1 ** 2)
        if right == 1:
            theta1 = atan2(x,y) + atan(Q/self.l1)   
        else:
            a=0
            theta1 = atan(Q / self.l1) -np.pi -atan2(x,y)
        gamma= atan2(-Q,z-self.off1)
        R=sqrt((z-self.off1)**2+Q**2)
        N=(R**2-self.l2**2-self.l3**2)/(2*self.l2*self.l3)
        M=(self.l3**2-self.l2**2-R**2)/(2*self.l2*R)
        theta2=gamma+atan2((sqrt(1-M**2)),M)
        theta3=atan2((sqrt(1-N**2)),N)
        if right == -1:
            #return np.array([np.round(np.degrees(theta1), 1), np.round(np.degrees(theta2)), np.round(np.degrees(theta3), 1)])
            #print("theta1:  -- ",np.round(np.degrees(theta1),1),"theta2__    ", np.round(np.degrees(theta2))," theta3 --- ", np.round(np.degrees(theta3)))
            #return np.round(np.degrees(theta1), 0),np.round(np.degrees(theta2),0),np.round(np.degrees(theta3), 0)
            return np.round(theta1, 5), np.round(theta2,5), np.round(theta3, 5)
            
        if right ==1:
            #return np.array([np.round(np.degrees(theta1),1), np.round(np.degrees(theta2)), np.round(np.degrees(theta3),1)])
            #print([np.round(np.degrees(theta1),1),"    ", np.round(np.degrees(theta2)),"    ", np.round(np.degrees(theta3))])
            return np.round(theta1, 5), np.round(theta2,5), np.round(theta3, 5)
    
    def dh(self,a, q, d, t):    
        
        z = np.matrix([[np.cos(math.radians(t)), -np.sin(math.radians(t)), 0, q],
                    [np.sin(math.radians(t)) * np.cos(math.radians(a)), np.cos(math.radians(t)) * np.cos(math.radians(a)),
                    -np.sin(math.radians(a)), -d * np.sin(math.radians(a))],
                    [np.sin(math.radians(t)) * np.sin(math.radians(a)), np.cos(math.radians(t)) * np.sin(math.radians(a)),
                    np.cos(math.radians(a)), d * np.cos(math.radians(a))],
                    [0, 0, 0, 1]])
        return z
    
    def leg_fk(self,theta1,theta2,theta3,right):
        
        if (right==-1):
            theta1=theta1
            theta2=-theta2
            theta3=-theta3
        if right==-1:
            g=1
        else:
            g=0
        o=np.matrix([[0,0,0,1]])
        fg = self.dh(0, 0, self.off1, -right*theta1)
        a = self.dh(-right*90, 0, self.l1,-g*180 + (90 + theta2))
        b = self.dh(0, self.l2, 0, theta3)
        c = self.dh(0, self.l3, 0, 0)
        q=fg @ a @ b @ c @np.transpose(o)
        x1 = np.round(q.item(0))
        y1 = np.round(q.item(1))
        z1 = np.round(q.item(2))

        ar = np.array([x1,y1,z1])
        return ar

    def body_IK(self,front,right,height,roll,pitch,yaw):
        R1 = np.array([
            [1, 0, 0, 0], 
            [0, 1, -np.sin(roll), 0],
            [0,np.sin(roll),np.cos(roll),0],
            [0,0,0,1]])
        R2 = np.array([
            [np.cos(yaw),0, np.sin(yaw), 0], 
            [0, 1, 0, 0],
            [-np.sin(yaw),0, np.cos(yaw),0],
            [0,0,0,1]])
        R3 = np.array([
            [np.cos(pitch),-np.sin(pitch), 0,0], 
            [np.sin(pitch),np.cos(pitch),0,0],
            [0,0,1,0],
            [0,0,0,1]])
        R = R3@R2@R1
        T = np.array([[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0]])
        Transformation = T + R
        Trb =np.array([
            [1,0,0,0],
            [0,1,0,self.width/2],
            [0,0,1,-self.back_length],
            [0,0,0,1]])@Transformation
        Trf = np.array([
            [1,0,0,0],
            [0,1,0,self.width/2],
            [0,0,1,self.front_length],
            [0,0,0,1]])@Transformation
        Tlf = np.array([
            [1,0,0,0],
            [0,1,0,-self.width/2],
            [0,0,1,self.front_length],
            [0,0,0,1]])@Transformation
        Tlb =np.array([
            [1,0,0,0],
            [0,1,0,-self.width/2],
            [0,0,1,-self.back_length],
            [0,0,0,1]])@Transformation
        return np.array([Trb,Trf,Tlf,Tlb])
    def body_to_leg_IK(self,front =0,right =0,height = 0.0,roll = 0,pitch = 0,yaw =0):
        T = self.body_IK(front,right,height,roll,pitch,yaw)
        Trb,Trf,Tlf,Tlb = T[0],T[1],T[2],T[3]
        LF = (T[2]@np.array([self.leg_fk(0,45,90,1)[0],self.leg_fk(0,45,90,1)[1],self.leg_fk(0,45,90,1)[2],1]))[:3]
        LB = (T[3]@np.array([self.leg_fk(0,45,90,1)[0],self.leg_fk(0,45,90,1)[1],self.leg_fk(0,45,90,1)[2],1]))[:3]
        RF = (T[1]@np.array([self.leg_fk(0,45,90,-1)[0],self.leg_fk(0,45,90,-1)[1],self.leg_fk(0,45,90,-1)[2],1]))[:3]
        RB = (T[0]@np.array([self.leg_fk(0,45,90,-1)[0],self.leg_fk(0,45,90,-1)[1],self.leg_fk(0,45,90,-1)[2],1]))[:3]
        J = np.zeros(12)
        J[0],J[1],J[2]  = self.leg_Ik(LF[0],LF[1],LF[2],-1)
        J[3],J[4],J[5]  = self.leg_Ik(LB[0],LB[1],LB[2],-1)
        J[6],J[7],J[8]  = self.leg_Ik(RF[0],RF[1],RF[2],1)
        J[9],J[10],J[11]= self.leg_Ik(RB[0],RB[1],RB[2],1)
        return J,LF,LB,RF,RB
if __name__ == "__main__":
    kinematics = dynamo_kinematics()
    #print("hari")
    #print(kinematics.leg_Ik(-16.97,2,8,1))
    #J,LF,LB,RF,RB = kinematics.body_to_leg_IK(0,0,0.15,0,0,0.35)
    #print(LF)



