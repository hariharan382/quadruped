#!/usr/bin/env python3
from distutils.command.clean import clean
from lib2to3.pgen2.token import OP, RBRACE
from re import I
import numpy as np
from sympy import O, deg

from std_msgs.msg import Float64,Int32
from math import *
import rospy
import actionlib
from kinematics import dynamo_kinematics
from dynamo_msgs.msg import Dynamo_motionAction, Dynamo_motionGoal, Dynamo_motionResult, Dynamo_motionFeedback
from sensor_msgs.msg import Joy,JointState
from dynamo_msgs.msg import Param

pubLF1 = rospy.Publisher('/unoffcial_dog_controller/left_front_adduction_joint_position_controller/command', Float64, queue_size=10)
pubLF2 = rospy.Publisher('/unoffcial_dog_controller/left_front_thigh_joint_position_controller/command', Float64, queue_size=10)
pubLF3 = rospy.Publisher('/unoffcial_dog_controller/left_front_knee_joint_position_controller/command', Float64, queue_size=10)
pubLB1 = rospy.Publisher('/unoffcial_dog_controller/left_back_adduction_joint_position_controller/command', Float64, queue_size=10)
pubLB2 = rospy.Publisher('/unoffcial_dog_controller/left_back_thigh_joint_position_controller/command', Float64, queue_size=10)
pubLB3 = rospy.Publisher('/unoffcial_dog_controller/left_back_knee_joint_position_controller/command', Float64, queue_size=10)
pubRF1 = rospy.Publisher('/unoffcial_dog_controller/front_right_adduction_joint_position_controller/command', Float64, queue_size=10)
pubRF2 = rospy.Publisher('/unoffcial_dog_controller/front_right_thigh_joint_position_controller/command', Float64, queue_size=10)
pubRF3 = rospy.Publisher('/unoffcial_dog_controller/front_right_knee_joint_position_controller/command', Float64, queue_size=10)
pubRB1 = rospy.Publisher('/unoffcial_dog_controller/right_back_adduction_joint_position_controller/command', Float64, queue_size=10)
pubRB2 = rospy.Publisher('/unoffcial_dog_controller/right_back_thigh_joint_position_controller/command', Float64, queue_size=10)
pubRB3 = rospy.Publisher('/unoffcial_dog_controller/right_back_knee_joint_position_controller/command', Float64, queue_size=10)

#arduinio
ard_LF1 = rospy.Publisher('LF1_topic', Int32, queue_size=1)
ard_LF2 = rospy.Publisher('LF2_topic', Int32, queue_size=1)
ard_LF3 = rospy.Publisher('LF3_topic', Int32, queue_size=1)
ard_LB1 = rospy.Publisher('LB1_topic', Int32, queue_size=1)
ard_LB2 = rospy.Publisher('LB2_topic', Int32, queue_size=1)
ard_LB3 = rospy.Publisher('LB3_topic', Int32, queue_size=1)
ard_RF1 = rospy.Publisher('RF1_topic', Int32, queue_size=1)
ard_RF2 = rospy.Publisher('RF2_topic', Int32, queue_size=1)
ard_RF3 = rospy.Publisher('RF3_topic', Int32, queue_size=1)
ard_RB1 = rospy.Publisher('RB1_topic', Int32, queue_size=1)
ard_RB2 = rospy.Publisher('RB2_topic', Int32, queue_size=1)
ard_RB3 = rospy.Publisher('RB3_topic', Int32, queue_size=1)

M=np.array([[   -8,     18, -11,      0, 1],
            [   -8,     14,  -5,      0, 0],
            [   16,    -32,  16,      0, 0],
            [   -1,    2.5,  -2,    0.5, 0],
            [    2,     -3,   1,      0, 0]])

def home_matrix(right):
        home=np.matrix([[1,0,0,-16.97],
                    [0,1,0,right*2],
                    [0,0,1,2],
                    [0,0,0,1]])
        
        return home

class gait:

    l1=2
    l2=12
    l3=12
    front_length=9
    back_length=9
    width=8
    off1=2
    right=1
    p_c=1
    

    halt = 1
    rotate = 0
    
    #gait params
    swing = 1
    stepLength = 6
    angular_step = (pi/180)*15
    velocity =36
    omega = 0.3
    stepHeight = 0.001
    rotate_start = 1
    

    #robot_height
    height=3
    
    jnt = []
    # joystic call back to determine the direction cosine values
    def joystick_callback(self,joy_status):
        #right = -joy_status.axes[0]
        #front = joy_status.axes[1]
        front_back=joy_status.axes[1]
        right_left=-joy_status.axes[0]
        r = -1*joy_status.buttons[3] + joy_status.buttons[4]
        
        #print(r)
        
        if abs(r):
            self.rotate = r
            self.halt = 0
            self.theta=0
            print(r)
        elif front_back !=0 or right_left !=0:
            self.halt = 0
            self.rotate = 0
            self.theta=np.arctan2(right_left,front_back)* 180 / np.pi
            print(self.theta)
        else:
            self.halt = 1
            self.rotate = 0
            self.theta=0

    # joint state subscriber to determine the current joint states which later be used by forward kinematics
    def joint_state_callback(self,joint_state):
        self.jnt = joint_state.position
    
    #blender matrix generater for determining the swing phase trajectory
    def point_matrix(self,P1,theta,stepLength):
        right = 1
        x=10
        theta=90-theta
        P3 = np.array([self.height, stepLength * np.cos(np.radians(theta)) * 0.5, stepLength * np.sin(np.radians(theta)) * 0.5])
        P2 = np.array([0, stepLength * np.cos(np.radians(theta)), stepLength * np.sin(np.radians(theta))])
        T1 = np.array([np.cos(np.radians(45)), -x * np.cos(np.radians(theta)) * np.sin(np.radians(45)),
                   -x * np.sin(np.radians(theta)) * np.sin(np.radians(45))])
        T2 = np.array([-np.cos(np.radians(45)), -x * np.cos(np.radians(theta)) * np.sin(np.radians(45)),
                   -x * np.sin(np.radians(theta)) * np.sin(np.radians(45))])

        return np.array([P1,P2,P3,T1,T2])
    
    
    def walk(self):
        while self.jnt == []:
            pass
        kinem = dynamo_kinematics()
        
        """LF = kinem.leg_fk(self.jnt[0],self.jnt[1],self.jnt[2],-1)
        LB = kinem.leg_fk(self.jnt[3],self.jnt[4],self.jnt[5],-1)
        RF = kinem.leg_fk(self.jnt[6],self.jnt[7],self.jnt[8],1)
        RB = kinem.leg_fk(self.jnt[9],self.jnt[10],self.jnt[11],1) """
        """LF=np.array([-16.97,-2,2,1])
        RF=np.array([-16.97,2,2,1])
        RB=np.array([-16.97,2,2,1])
        LB=np.array([-16.97,-2,2,1])"""

        LF=np.array([-16.97,-2,2,1])
        RF=np.array([-16.97,2,2,1])
        RB=np.array([-16.97,2,2,1])
        LB=np.array([-16.97,-2,2,1])

        LF[0] = -16.97
        RF[0] = -16.97
        RB[0] = -16.97
        LB[0] = -16.97
        Trb =np.array([
            [1,0,0,0],
            [0,1,0,self.width/2],
            [0,0,1,-self.back_length],
            [0,0,0,1]])
        Trf = np.array([
            [1,0,0,0],
            [0,1,0,self.width/2],
            [0,0,1,self.front_length],
            [0,0,0,1]])
        Tlf = np.array([
            [1,0,0,0],
            [0,1,0,-self.width/2],
            [0,0,1,self.front_length],
            [0,0,0,1]])
        Tlb =np.array([
            [1,0,0,0],
            [0,1,0,-self.width/2],
            [0,0,1,-self.back_length],
            [0,0,0,1]])
        P1=np.array([0,0,0])
        
        while not rospy.is_shutdown():

            if not self.halt and not abs(self.rotate):
                T = self.stepLength/self.velocity
                self.rotate_start =1
                LF=np.array([-16.97,-2,2,1])
                RF=np.array([-16.97,2,2,1])
                RB=np.array([-16.97,2,2,1])
                LB=np.array([-16.97,-2,2,1])
                if self.swing:
                    
                    swing_lf_points = self.point_matrix(P1,self.theta,self.stepLength)
                    swing_rb_points = self.point_matrix(P1,self.theta,self.stepLength)
                    rf_end_point =np.dot(home_matrix(1),np.array([swing_rb_points[1][0],swing_rb_points[1][1],swing_rb_points[1][2],1]))
                    lb_end_point = np.dot(home_matrix(-1),np.array([swing_lf_points[1][0],swing_lf_points[1][1],swing_lf_points[1][2],1]))
                    #print("phase-1 ------")
                    #print("p_r:  ", rf_end_point)
                    #print("p_lb:  ", lb_end_point)
                    start = rospy.get_time()
                    t = 0
                    print("phase_1",self.stepLength)
                    x=0

                    while(t<T):
                        u = t/T
                        u = round(u,4)
                        U = np.array([u**4,u**3,u**2,u,1])
                        B = np.dot(M,U)   
                        
                        v_lf = np.dot(B,swing_lf_points)
                        v_lf=np.array([v_lf[0],v_lf[1],v_lf[2],1])
                        p_lf = np.dot(home_matrix(-1),v_lf)
                    
                        v_rb=np.dot(B,swing_rb_points)
                        v_rb=np.array([v_rb[0],v_rb[1],v_rb[2],1])
                        p_rb = np.dot(home_matrix(1),v_rb)
                        #print(p_rb)
                        
                        p_rf=(RF*(u) + rf_end_point*(1-u))
                        #print("phase-1     p_rf:   ",p_rf)
                        #print(p_rf.item(0))
                        

                        p_lb=(LB*(u) + lb_end_point*(1-u))
                        #print("swing_phase (phase-1) ......................................")
                        RB_J1,RB_J2,RB_J3 = kinem.leg_Ik(p_rb.item(0),p_rb.item(1),p_rb.item(2),1)
                        LF_J1,LF_J2,LF_J3 = kinem.leg_Ik(p_lf.item(0),p_lf.item(1),p_lf.item(2),-1)
                        #print("stance phase (phase-1)                   ////////////////////////////////////////////        ")
                        RF_J1,RF_J2,RF_J3 = kinem.leg_Ik(p_rf.item(0),p_rf.item(1),p_rf.item(2),1)
                        LB_J1,LB_J2,LB_J3 = kinem.leg_Ik(p_lb.item(0),p_lb.item(1),p_lb.item(2),-1)
                        #print("p_rf:",p_rf)
                        
                        change=x-LF_J2
                        #print("phase-1  change : ",np.round(change,8))
                        pubLF1.publish(-LF_J1)
                        pubLF2.publish(-LF_J2)
                        pubLF3.publish(-LF_J3)
                        pubRB1.publish(-RB_J1)
                        pubRB2.publish(-RB_J2)
                        pubRB3.publish(-RB_J3)
                        pubLB1.publish(-LB_J1)
                        pubLB2.publish(-LB_J2)
                        pubLB3.publish(-LB_J3)
                        pubRF1.publish(-RF_J1)
                        pubRF2.publish(-RF_J2)
                        pubRF3.publish(-RF_J3)
                        #print("RF_j1: ",degrees(RF_J1),"RFj2: ",degrees(RF_J2),"RFj3: ",degrees(RF_J3))
                        t = rospy.get_time() - start
                        x=LF_J2
                        h_off=30
                        
                        if(30+degrees(RB_J2) and 30+degrees(RF_J2) > 0):
                            #RB_List.append(degrees(RB_J2))
                            #RF_List.append(degrees(RF_J2))
                            #print((RF_List[1]-RF_List[0]), " delta -- phase -1")

                            ard_RB1.publish(np.int32(90+degrees(RB_J1)))
                            ard_RB2.publish(np.int32(degrees(RB_J2)+30))
                            #ard_RB3.publish(np.int32(180-degrees(RB_J3)-(RB_List[1]-RB_List[0])))
                            ard_RB3.publish(np.int32(180-(degrees(RB_J3)-45)-degrees(RB_J2))+h_off)
                            ard_RF1.publish(np.int32(90+degrees(RF_J1)))
                            ard_RF2.publish(np.int32(degrees(RF_J2)+30))
                            #print("rf3_:   ",degrees(RF_J3))
                            #ard_RF3.publish(np.int32(180-degrees(RF_J3)-(RF_List[1]-RF_List[0])))
                            ard_RF3.publish(np.int32(180-(degrees(RF_J3)-45)-degrees(RF_J2)+h_off)) 
                            
                            
                            

                        
                        if(180-degrees(RB_J2) and 180-degrees(RF_J2)<=180):
                            #LB_List.append(degrees(LB_J2))
                            #LF_List.append(degrees(LF_J2))
                            ard_LB1.publish(np.int32(90-degrees(LB_J1)))
                            ard_LB2.publish(np.int32(150-degrees(LB_J2)))
                            ard_LB3.publish(np.int32(degrees(LB_J3)-45+degrees(LB_J2)-h_off))
                            ard_LF1.publish(np.int32(90-degrees(LF_J1)))
                            ard_LF2.publish(np.int32(150-degrees(LF_J2)))
                            ard_LF3.publish(np.int32(degrees(LF_J3)-45+degrees(LF_J2)-h_off-10)) 
                            
                    
                            rate.sleep()

                        
                       
                    self.swing = 0
                else:
                    swing_rf_points = self.point_matrix(P1,self.theta,self.stepLength)
                    swing_lb_points = self.point_matrix(P1,self.theta,self.stepLength)
                    rb_end_point = np.dot(home_matrix(1), np.array([swing_rf_points[1][0],swing_rf_points[1][1],swing_rf_points[1][2],1]))
                    lf_end_point = np.dot(home_matrix(-1), np.array([swing_lb_points[1][0],swing_lb_points[1][1],swing_lb_points[1][2],1]))
        
                    start = rospy.get_time()
                    t = 0
                    print("phase-2    step_length:",self.stepLength)
    
                
                    while (t < T):
                        t = rospy.get_time() - start
                        u = t/T
                        U = np.array([u**4,u**3,u**2,u,1]).T
                        B = np.dot(M,U)
                        v_rf=np.dot(B,swing_rf_points) 
                        v_rf=np.array([v_rf[0],v_rf[1],v_rf[2],1])
                        p_rf=np.dot(home_matrix(1),v_rf) 
                        
                        v_lb=np.dot(B,swing_lb_points) 
                        v_lb=np.array([v_lb[0],v_lb[1],v_lb[2],1])
                        p_lb=np.dot(home_matrix(-1),v_lb) 
                        
                        p_rb=(RB*(u) + rb_end_point*(1-u))
                        #print(" phase-2          p_rb:   ",p_rb)
                        p_lf=(LF*(u) + lf_end_point*(1-u))
                        #print("swing_phase (phase-2) ......................................")
                        
                        RF_J1,RF_J2,RF_J3 = kinem.leg_Ik(p_rf.item(0),p_rf.item(1),p_rf.item(2),1)
                        LB_J1,LB_J2,LB_J3 = kinem.leg_Ik(p_lb.item(0),p_lb.item(1),p_lb.item(2),-1)
                        #print("stance phase (phase-2)                   ////////////////////////////////////////////        ")
                        RB_J1,RB_J2,RB_J3 = kinem.leg_Ik(p_rb.item(0),p_rb.item(1),p_rb.item(2),1)
                        LF_J1,LF_J2,LF_J3 = kinem.leg_Ik(p_lf.item(0),p_lf.item(1),p_lf.item(2),-1)
                        #print("phase-2 ", -LF_J2)
                        pubLF1.publish(-LF_J1)
                        pubLF2.publish(-LF_J2)
                        pubLF3.publish(-LF_J3)
                        pubRB1.publish(-RB_J1)
                        pubRB2.publish(-RB_J2)
                        pubRB3.publish(-RB_J3)
                        pubLB1.publish(-LB_J1)
                        pubLB2.publish(-LB_J2)
                        pubLB3.publish(-LB_J3)
                        pubRF1.publish(-RF_J1)
                        pubRF2.publish(-RF_J2)
                        pubRF3.publish(-RF_J3)
                        #print("Rfj1: ",degrees(RF_J1),"RFj2: ",degrees(RF_J2),"RFj3: ",degrees(RF_J2))
                        t = rospy.get_time() - start
                        
                        if(30+degrees(RB_J2) and 30+degrees(RF_J2) > 0):
                            #RB_List.append(degrees(RB_J2))
                            #RF_List.append(degrees(RF_J2))
                            #print((RF_List[1]-RF_List[0]), " delta -- phase -2")

                            ard_RB1.publish(np.int32(90+degrees(RB_J1)))
                            ard_RB2.publish(np.int32(degrees(RB_J2)+30))
                            ard_RB3.publish(np.int32(180-(degrees(RB_J3)-45)-degrees(RB_J2)+h_off))
                            ard_RF1.publish(np.int32(90+degrees(RF_J1)))
                            ard_RF2.publish(np.int32(degrees(RF_J2)+30))
                            #print("rf3_:   ",degrees(RF_J3))
                            #ard_RF3.publish(np.int32(180-degrees(RF_J3)-(RF_List[1]-RF_List[0])))
                            ard_RF3.publish(np.int32(180-(degrees(RF_J3)-45)-degrees(RF_J2)+h_off))
                 
                            rate.sleep()
                            
                           

                        
                        if(180-degrees(RB_J2) and 180-degrees(RF_J2)<=180):
                            
                            #LB_List.append(degrees(LB_J2))
                            #LF_List.append(degrees(LF_J2))
                            ard_LB1.publish(np.int32(90-degrees(LB_J1))) #ard_LB1.publish(np.int32(90-degrees(90)))
                            ard_LB2.publish(np.int32(150-degrees(LB_J2)))
                            ard_LB3.publish(np.int32(degrees(LB_J3)-45+degrees(LB_J2)-h_off))
                            ard_LF1.publish(np.int32(90-degrees(LF_J1)))
                            ard_LF2.publish(np.int32(150-degrees(LF_J2)))
                            ard_LF3.publish(np.int32(degrees(LF_J3)-45+degrees(LF_J2)-h_off-10))
                            
                          
                            #rate.sleep() 
                            
                        
                    self.swing = 1
                LF = p_lf
                LF[0] = -16.97
                LB = p_lb
                LB[0] = -16.97
                RF = p_rf
                RF[0] = -16.97
                RB = p_rb
                RB[0] = -16.97
            #---------------------------------- rotate ------------------------------------
            #-------------------------------------------------------------------------------------
            #------------------------------------------------------------------------------------
            #-------------------------------------------------------------------------------------
            #------------------------------------------------------------------------------------
            #-------------------------------------------------------------------------------------
            #------------------------------------------------------------------------------------
            if self.rotate!=0 and not self.halt:
                T = self.stepLength/self.velocity
                LF=np.array([-16.97,-2,2,1])
                RF=np.array([-16.97,2,2,1])
                RB=np.array([-16.97,2,2,1])
                LB=np.array([-16.97,-2,2,1])
                ang=np.degrees(atan2(92,42))
                rotate_step=4
                

                if self.p_c==1:
                    
                    if self.rotate == -1:
                        swing_lf_points = self.point_matrix(P1,120,rotate_step)
                        swing_rb_points = self.point_matrix(P1,-60,rotate_step) #60
                        swing_rf_points = self.point_matrix(P1,70,rotate_step) #-70
                        swing_lb_points = self.point_matrix(P1,-110,rotate_step)#110
                    else:
                        swing_lf_points = self.point_matrix(P1,-70,rotate_step)
                        swing_rb_points = self.point_matrix(P1,110,rotate_step) #60
                        swing_rf_points = self.point_matrix(P1,-120,rotate_step) #-70
                        swing_lb_points = self.point_matrix(P1,60,rotate_step)#110

                    rf_end_point =np.dot(home_matrix(1),np.array([swing_rf_points[1][0],swing_rf_points[1][1],swing_rf_points[1][2],1]))
                    lb_end_point = np.dot(home_matrix(-1),np.array([swing_lb_points[1][0],swing_lb_points[1][1],swing_lb_points[1][2],1]))
                    #print("phase-1 ------")
                    #print("p_r:  ", rf_end_point)
                    #print("p_lb:  ", lb_end_point)
                    start = rospy.get_time()
                    t = 0
                    
                    x=0

                    while(t<T):
                        u = t/T
                        u = round(u,4)
                        U = np.array([u**4,u**3,u**2,u,1])
                        B = np.dot(M,U)   
                        
                        v_lf = np.dot(B,swing_lf_points)
                        v_lf=np.array([v_lf[0],v_lf[1],v_lf[2],1])
                        p_lf = np.dot(home_matrix(-1),v_lf)
                    
                        v_rb=np.dot(B,swing_rb_points)
                        v_rb=np.array([v_rb[0],v_rb[1],v_rb[2],1])
                        p_rb = np.dot(home_matrix(1),v_rb)
                        #print(p_rb)
                        
                        p_rf=(RF*(u) + rf_end_point*(1-u))
                        #print("phase-1     p_rf:   ",p_rf)
                        #print(p_rf.item(0))
                        

                        p_lb=(LB*(u) + lb_end_point*(1-u))
                        #print("swing_phase (phase-1) ......................................")
                        RB_J1,RB_J2,RB_J3 = kinem.leg_Ik(p_rb.item(0),p_rb.item(1),p_rb.item(2),1)
                        LF_J1,LF_J2,LF_J3 = kinem.leg_Ik(p_lf.item(0),p_lf.item(1),p_lf.item(2),-1)
                        #print("stance phase (phase-1)                   ////////////////////////////////////////////        ")
                        RF_J1,RF_J2,RF_J3 = kinem.leg_Ik(p_rf.item(0),p_rf.item(1),p_rf.item(2),1)
                        LB_J1,LB_J2,LB_J3 = kinem.leg_Ik(p_lb.item(0),p_lb.item(1),p_lb.item(2),-1)
                        #print("p_rf:",p_rf)
                        #print("phase_1",LB_J1)
                        
                        
                        #print("phase-1  change : ",np.round(change,8))
                        pubLF1.publish(-LF_J1)
                        pubLF2.publish(-LF_J2)
                        pubLF3.publish(-LF_J3)
                        pubRB1.publish(-RB_J1)
                        pubRB2.publish(-RB_J2)
                        pubRB3.publish(-RB_J3)
                        pubLB1.publish(-LB_J1)
                        pubLB2.publish(-LB_J2)
                        pubLB3.publish(-LB_J3)
                        pubRF1.publish(-RF_J1)
                        pubRF2.publish(-RF_J2)
                        pubRF3.publish(-RF_J3)
                        #print("RF_j1: ",degrees(RF_J1),"RFj2: ",degrees(RF_J2),"RFj3: ",degrees(RF_J3))
                        t = rospy.get_time() - start
                        x=LF_J2
                        h_off=30
                        
                        """if(30+degrees(RB_J2) and 30+degrees(RF_J2) > 0):
                            #RB_List.append(degrees(RB_J2))
                            #RF_List.append(degrees(RF_J2))
                            #print((RF_List[1]-RF_List[0]), " delta -- phase -1")

                            ard_RB1.publish(np.int32(90))
                            ard_RB2.publish(np.int32(degrees(RB_J2)+30))
                            #ard_RB3.publish(np.int32(180-degrees(RB_J3)-(RB_List[1]-RB_List[0])))
                            ard_RB3.publish(np.int32(180-(degrees(RB_J3)-45)-degrees(RB_J2))+h_off)
                            ard_RF1.publish(np.int32(90))
                            ard_RF2.publish(np.int32(degrees(RF_J2)+30))
                            #print("rf3_:   ",degrees(RF_J3))
                            #ard_RF3.publish(np.int32(180-degrees(RF_J3)-(RF_List[1]-RF_List[0])))
                            ard_RF3.publish(np.int32(180-(degrees(RF_J3)-45)-degrees(RF_J2)+h_off)) 
                            
                            
                            

                        
                        if(180-degrees(RB_J2) and 180-degrees(RF_J2)<=180):
                            #LB_List.append(degrees(LB_J2))
                            #LF_List.append(degrees(LF_J2))
                            ard_LB1.publish(np.int32(90))
                            ard_LB2.publish(np.int32(150-degrees(LB_J2)))
                            ard_LB3.publish(np.int32(degrees(LB_J3)-45+degrees(LB_J2)-h_off))
                            ard_LF1.publish(np.int32(90))
                            ard_LF2.publish(np.int32(150-degrees(LF_J2)))
                            ard_LF3.publish(np.int32(degrees(LF_J3)-45+degrees(LF_J2)-h_off-10)) """
                            
                    
                            #rate.sleep()

                        
                       
                    self.p_c=0 
                else:

                    if self.rotate == -1:
                        swing_lf_points = self.point_matrix(P1,120,rotate_step)
                        swing_rb_points = self.point_matrix(P1,-60,rotate_step) #60
                        swing_rf_points = self.point_matrix(P1,70,rotate_step) #-70
                        swing_lb_points = self.point_matrix(P1,-110,rotate_step)#110
                    else:
                        swing_lf_points = self.point_matrix(P1,-70,rotate_step)
                        swing_rb_points = self.point_matrix(P1,110,rotate_step) #60
                        swing_rf_points = self.point_matrix(P1,-120,rotate_step) #-70
                        swing_lb_points = self.point_matrix(P1,60,rotate_step)#110
                    
                    rb_end_point = np.dot(home_matrix(1), np.array([swing_rb_points[1][0],swing_rb_points[1][1],swing_rb_points[1][2],1]))
                    lf_end_point = np.dot(home_matrix(-1), np.array([swing_lf_points[1][0],swing_lf_points[1][1],swing_lf_points[1][2],1]))
                    #print("phase-2")
                    #print("p_lf:  ", lf_end_point)
                    #print("p_rb:  ", rb_end_point)
                    start = rospy.get_time()
                    t = 0
                    #print("phase-2")
    
                
                    while (t < T):
                        t = rospy.get_time() - start
                        u = t/T
                        U = np.array([u**4,u**3,u**2,u,1]).T
                        B = np.dot(M,U)
                        v_rf=np.dot(B,swing_rf_points) 
                        v_rf=np.array([v_rf[0],v_rf[1],v_rf[2],1])
                        p_rf=np.dot(home_matrix(1),v_rf) 
                        
                        v_lb=np.dot(B,swing_lb_points) 
                        v_lb=np.array([v_lb[0],v_lb[1],v_lb[2],1])
                        p_lb=np.dot(home_matrix(-1),v_lb) 
                        
                        p_rb=(RB*(u) + rb_end_point*(1-u))
                        #print(" phase-2          p_rb:   ",p_rb)
                        p_lf=(LF*(u) + lf_end_point*(1-u))
                        #print("swing_phase (phase-2) ......................................")
                        
                        RF_J1,RF_J2,RF_J3 = kinem.leg_Ik(p_rf.item(0),p_rf.item(1),p_rf.item(2),1)
                        LB_J1,LB_J2,LB_J3 = kinem.leg_Ik(p_lb.item(0),p_lb.item(1),p_lb.item(2),-1)
                        #print("stance phase (phase-2)                   ////////////////////////////////////////////        ")
                        RB_J1,RB_J2,RB_J3 = kinem.leg_Ik(p_rb.item(0),p_rb.item(1),p_rb.item(2),1)
                        LF_J1,LF_J2,LF_J3 = kinem.leg_Ik(p_lf.item(0),p_lf.item(1),p_lf.item(2),-1)
                        #print("phase-2 ", -LF_J2)
                        pubLF1.publish(-LF_J1)
                        pubLF2.publish(-LF_J2)
                        pubLF3.publish(-LF_J3)
                        pubRB1.publish(-RB_J1)
                        pubRB2.publish(-RB_J2)
                        pubRB3.publish(-RB_J3)
                        pubLB1.publish(-LB_J1)
                        pubLB2.publish(-LB_J2)
                        pubLB3.publish(-LB_J3)
                        pubRF1.publish(-RF_J1)
                        pubRF2.publish(-RF_J2)
                        pubRF3.publish(-RF_J3)
                        #print("Rfj1: ",degrees(RF_J1),"RFj2: ",degrees(RF_J2),"RFj3: ",degrees(RF_J2))
                        t = rospy.get_time() - start
                        
                        """if(30+degrees(RB_J2) and 30+degrees(RF_J2) > 0):
                            #RB_List.append(degrees(RB_J2))
                            #RF_List.append(degrees(RF_J2))
                            #print((RF_List[1]-RF_List[0]), " delta -- phase -2")

                            ard_RB1.publish(np.int32(90))
                            ard_RB2.publish(np.int32(degrees(RB_J2)+30))
                            ard_RB3.publish(np.int32(180-(degrees(RB_J3)-45)-degrees(RB_J2)+h_off))
                            ard_RF1.publish(np.int32(90))
                            ard_RF2.publish(np.int32(degrees(RF_J2)+30))
                            #print("rf3_:   ",degrees(RF_J3))
                            #ard_RF3.publish(np.int32(180-degrees(RF_J3)-(RF_List[1]-RF_List[0])))
                            ard_RF3.publish(np.int32(180-(degrees(RF_J3)-45)-degrees(RF_J2)+h_off))
                 
                            rate.sleep()
                            
                           

                        
                        if(180-degrees(RB_J2) and 180-degrees(RF_J2)<=180):
                            
                            #LB_List.append(degrees(LB_J2))
                            #LF_List.append(degrees(LF_J2))
                            ard_LB1.publish(np.int32(90)) #ard_LB1.publish(np.int32(90-degrees(90)))
                            ard_LB2.publish(np.int32(150-degrees(LB_J2)))
                            ard_LB3.publish(np.int32(degrees(LB_J3)-45+degrees(LB_J2)-h_off))
                            ard_LF1.publish(np.int32(90))
                            ard_LF2.publish(np.int32(150-degrees(LF_J2)))
                            ard_LF3.publish(np.int32(degrees(LF_J3)-45+degrees(LF_J2)-h_off-10)) """
                            
                          
                            #rate.sleep() 
                            
                        
                    self.p_c=1
                LF = p_lf
                LF[0] = -16.97
                LB = p_lb
                LB[0] = -16.97
                RF = p_rf
                RF[0] = -16.97
                RB = p_rb
                RB[0] = -16.97


if __name__ == "__main__":
    unofficial_dog = gait()
    rospy.init_node('Walk_control_server')
    rospy.Subscriber("/joy",Joy, unofficial_dog.joystick_callback)
    rospy.Subscriber("/unoffcial_dog_controller/joint_states",JointState, unofficial_dog.joint_state_callback)
    rate=rospy.Rate(40)
    unofficial_dog.walk()
    