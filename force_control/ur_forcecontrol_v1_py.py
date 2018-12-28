#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from ur5_planning.msg import forcecontrol_msg
from geometry_msgs.msg import WrenchStamped
from ur5_pose_get import *
from netf_data_get import *
from ur5_kinematics import *
import math

import numpy
from numpy import matlib,linalg
#Import the module

from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from pykdl_utils.kdl_kinematics import KDLKinematics
from transfer import *
import cmath
class Forcectl():
    def __init__(self,urdfname,weights):
        self.PI=math.pi
        self.q_start=[0.0,-self.PI,self.PI/2,-self.PI/2,self.PI/2,0.0]
        self.FILTER_A=0.5
        self.urdfname=urdfname
        self.weights=weights
    def Init_node(self):
        rospy.init_node("Force_control_v1")
        #first get pose from UR5
        ur_reader = Urposition()
        ur_sub = rospy.Subscriber("/joint_states", JointState, ur_reader.callback)
        netf_reader = NetfData()
        netf_sub = rospy.Subscriber("/netft_data", WrenchStamped, netf_reader.callback)
        ur_pub = rospy.Publisher("/ur_driver/URScript", String, queue_size=10)
        return ur_reader,netf_reader,ur_pub
    def first_order_filtering(self,input_value):
        Value=0
        NewValue = input_value
        Value = (int)((NewValue * self.FILTER_A + (1.0 - self.FILTER_A) * Value))
        return Value
    def get_data_from_kdl(self):
        robot = URDF.from_xml_file(self.urdfname)
        tree = kdl_tree_from_urdf_model(robot)
        chain = tree.getChain("base_link", "ee_link")
        # print chain.getNrOfJoints()
        # forwawrd kinematics
        kdl_kin = KDLKinematics(robot, "base_link", "ee_link")
        return kdl_kin
    def get_urobject_ur5kinetmatics(self):
        ur0 = Kinematic()
        return ur0
    """
    0,use kdl
    1,use ur5_kinematics
    """
    def get_T_from_joint(self,jointq,flag):
        if flag==0:
            kdl_kin=self.get_data_from_kdl()
            q=jointq
            #q = [0, 0, 1, 0, 1, 0]
            pose = kdl_kin.forward(q)  # forward kinematics (returns homogeneous 4x4 matrix)
            return pose
        elif flag==1:#numpy mat datastruct
            ur0=self.get_urobject_ur5kinetmatics()
            T=ur0.Forward(jointq)
            return numpy.mat(T).reshape((4,4))
        elif flag==2:#list datastruct
            ur0=self.get_urobject_ur5kinetmatics()
            T=ur0.Forward(jointq)
            return T
        else:
            print "get T error------"
    def get_IK_from_T(self,T,flag,q_last):
        if flag==0:
            kdl_kin=self.get_data_from_kdl()
            q_ik = kdl_kin.inverse(T)  # inverse kinematics
            # print "----------iverse-------------------\n", q_ik

            if q_ik is not None:
                return q_ik
            else:
                print "ik failed-------------"
                return None
        elif flag==1:
            ur0 = self.get_urobject_ur5kinetmatics()
            return ur0.best_sol(self.weights,q_last,T)
        else:
            print "flag error-----please use 0 or 1"

    #T is numpy.array
    def get_T_translation(self,T):
        trans_x=  T[0].tolist()[0][3]
        trans_y = T[1].tolist()[0][3]
        trans_z = T[2].tolist()[0][3]
        return [trans_x,trans_y,trans_z]
    def get_force_error(self,force_val_desire,f_val_from_sensor):
        force_error=list(map(lambda x: x[0] - x[1], zip(force_val_desire, f_val_from_sensor)))
        return force_error

    def get_new_translation_to_base(self,kp,R,force_val_desire,f_val_from_sensor):
        force_error=self.get_force_error(force_val_desire,f_val_from_sensor)
        new_translation=-kp*numpy.dot(R,numpy.mat(force_error).T)
        return new_translation
    def get_new_T(self,T,new_translation):
        trans_x=  T[0].tolist()[0][3]+new_translation.tolist()[0][0]
        trans_y = T[1].tolist()[0][3]+new_translation.tolist()[1][0]
        trans_z = T[2].tolist()[0][3]+new_translation.tolist()[2][0]
        R=numpy.mat(t2r(T)).tolist()
        T0=R[0].append(trans_x)
        T1=R[1].append(trans_y)
        T2=R[2].append(trans_z)
        T3=R.append([0,0,0,1])
        T=numpy.matrixlib.defmatrix.matrix(numpy.mat(R))
        return T

def main():
    urdfname = "/data/ros/ur_ws_yue/src/ur5_planning/urdf/ur5.urdf"
    ratet = 30
    kp = 0.00001
    kd = 0.0
    k_spring = 500
    #desire force
    force_val_desire=[0]*3
    f_val_from_sensor=[0]*3
    weights=[1.] * 6
    templist=[0.]*48
    lambda1=-3.666666
    detat=0.05
    z=0.3
    ace=50
    vel=0.1
    urt=0
    t=0
    ratet=30
    F0=Forcectl(urdfname,weights)
    ur_reader, netf_reader, ur_pub=F0.Init_node()
    rate = rospy.Rate(ratet)
    while not rospy.is_shutdown():
        templist=[]
        #1get T from joint Q
        if len(ur_reader.ave_ur_pose)!=0:
            q_now=ur_reader.ave_ur_pose
            print "q_now-----",q_now
            T_Now=F0.get_T_from_joint(q_now,1)
            T_Now_list=F0.get_T_from_joint(q_now,2)
            print "T_Now",T_Now
            print "T_Now list",T_Now_list
            print "Trans------",F0.get_T_translation(T_Now)
            R_Now=numpy.mat(t2r(T_Now))
            print "R",R_Now
        else:
            print "wait q_now data ok--------"
        if len(netf_reader.ave_netf_force_data)!=0:
            print "netf_reader.ave_netf_force_data",netf_reader.ave_netf_force_data
            #z,x,y=[z,0,0]
            f_val_from_sensor=[netf_reader.ave_netf_force_data[2],0,0]
            force_error=F0.get_force_error(force_val_desire,f_val_from_sensor)
            print "force_error",force_error
            print "force_error",numpy.mat(force_error).T
            new_translation=F0.get_new_translation_to_base(kp,R_Now,force_val_desire,f_val_from_sensor)
            print "new_translation----",new_translation.tolist()
            New_T=F0.get_new_T(T_Now,new_translation)
            print "new_T",New_T
            print "new_T",New_T.tolist()[0]+New_T.tolist()[1]+New_T.tolist()[2]+New_T.tolist()[3]
            q_new=F0.get_IK_from_T(New_T.tolist()[0]+New_T.tolist()[1]+New_T.tolist()[2]+New_T.tolist()[3],1,q_now)
            print "q_now---",q_now
            print "q_new---",q_new.tolist()
            #move ur5
            ur_reader.urscript_pub(ur_pub, q_new, vel, ace, urt)
        else:
            print "wait netf_utils data ok-----"

        rate.sleep()
if __name__ == '__main__':
    main()