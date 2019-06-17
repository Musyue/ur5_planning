#!/usr/bin/env python
# -*- coding: utf-8 -*-

import frompitoangle
import numpy
from numpy import matlib,linalg
#Import the module
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from pykdl_utils.kdl_kinematics import KDLKinematics
from ur5_kinematics import Kinematic
#import cv2
import rospy
import yaml,os
from trans_methods import *
from get_arpose_from_ar import *
from ar_track_alvar_msgs.msg import AlvarMarkers
from hand_in_eye import *
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from ur5_pose_get import *
from std_msgs.msg import Float64
from ur5_planning.msg import uv
from uv_sub_node import *

from ur5_planning.msg import structure_point
from structure_point_uv_sub import *


class VisonControl():
    def __init__(self,califilename,sim,lambda1,urdfname):
        self.califilename=califilename
        self.sim=sim
        self.lambda1=lambda1
        self.urdfname=urdfname
        rospy.init_node("vision_control")
    #if flag=1,use our kinematics for inverse
    def get_jacabian_from_joint(self,urdfname,jointq,flag):
        #robot = URDF.from_xml_file("/data/ros/ur_ws/src/universal_robot/ur_description/urdf/ur5.urdf")
        robot = URDF.from_xml_file(urdfname)
        tree = kdl_tree_from_urdf_model(robot)
        # print tree.getNrOfSegments()
        chain = tree.getChain("base_link", "ee_link")
        # print chain.getNrOfJoints()
        # forwawrd kinematics
        kdl_kin = KDLKinematics(robot, "base_link", "ee_link")
        q=jointq
        #q = [0, 0, 1, 0, 1, 0]
        pose = kdl_kin.forward(q)  # forward kinematics (returns homogeneous 4x4 matrix)
        # print pose
        #print list(pose)
        q0=Kinematic()
        if flag==1:
            q_ik=q0.best_sol_for_other_py( [1.] * 6, 0, q0.Forward(q))
        else:
            q_ik = kdl_kin.inverse(pose)  # inverse kinematics
        # print "----------iverse-------------------\n", q_ik

        if q_ik is not None:
            pose_sol = kdl_kin.forward(q_ik)  # should equal pose
            print "------------------forward ------------------\n",pose_sol

        J = kdl_kin.jacobian(q)
        #print 'J:', J
        return J,pose
    #kx=f/px,ky=f/py
    #sim=1,use camera default from Macine Vision Toolbox for MATLAB
    def get_cam_data(self):
        if self.sim==1:
            kx=0.008/10**(-5)
            ky=0.008/10**(-5)
            u0=512
            v0=512
            cam = {'kx': kx, 'ky': ky, "u0": u0, "desiruvv0": v0}
            return cam
        f=open(self.califilename)
        yamldata=yaml.load(f)
        #print yamldata
        kx =  yamldata['camera_matrix']['data'][0]
        #print kx
        ky = yamldata['camera_matrix']['data'][4]
        #print ky
        u0=yamldata['camera_matrix']['data'][2]
        v0 = yamldata['camera_matrix']['data'][5]
        cam = {'kx': kx, 'ky': ky, "u0": u0, "v0": v0}
        return cam
        #print yaml.load(f)


    """ read data from yaml, here it temporary uses the list exist"""
    def get_instrinc_param(self):
        data = numpy.array(
            [476.437121, 0.000000, 327.822153, 0.000000, 475.466603, 256.489387, 0.000000, 0.000000, 1.000000])
        instrinc_param = data.reshape((3, 3))
       # print(instrinc_param)
        return instrinc_param

    """  input : camera pos of ar tag  3*1; only for one point-0.08401211423342386, 0.004883804261170381, 0.7855804355335336, -0.09810482217655597, 0.9939528146814213, -0.03307682330079316, -0.036594669187119074
        output :  image space u,v coordinate"""
    def get_uv_from_ar(self,pos):
        #pos = [-0.0694628511461, 0.0487799361822, 0.988924230718]
        # print("pos:", pos)
        cam_pos = numpy.array( pos )
        # 归一化
        # print("cam pos1:", cam_pos)rate = rospy.Rate(0.1)
        # cam_pos = cam_pos.reshape((3,1)) / cam_pos[2]
        cam_pos = cam_pos.T / cam_pos[2]
        # print(cam_pos)
        # print("cam pos2:", cam_pos)
        imgpos = numpy.dot( self.get_instrinc_param(), cam_pos)
        #print imgpos
        imgpos = imgpos[0:2]
        #print("imgps2:", imgpos)
        return imgpos.tolist()
        # print(imgpos)
    def get_z_from_ar(self):
        pass
    #cal image jacbian
    def vis2jac(self,uv,z):
        cam=self.get_cam_data()
        rh0=[0.0000032,0.0000032]
        camf=0.4762192#0.6240429#m
        kx = cam['kx']
        ky = cam['ky']
        #--------------sgl-------------
        # print kx
        pu=camf/kx
        pv=camf/ky
        #f/pu=kx,f/pv=ky
        # arfx=kx/camf
        # arfy=ky/camf
        # kx=arfx*camf
        # ky=arfy*camf
        uba=uv[0]-cam['u0']
        vba=uv[1]-cam['v0']
        L=[[-kx/z,0,uba/z,1/kx*uba*vba,-(kx**2+uba**2)/kx,vba,0,-ky/z,vba/z,(ky**2+vba**2)/ky,-uba*vba/ky,-uba]]
        J=numpy.array(L).reshape((2,6))
        return J
    #uv more than one
    #,uv = [[672, 672], [632, 662]]
    def vis2jac_mt1(self,uvm,z):
        if len(uvm)>1:
            L=self.vis2jac(uvm[0],z)
            #L=numpy.array(L).reshape((2,6))
            for i in xrange(1,len(uvm)):
                J=numpy.row_stack((L,self.vis2jac(uvm[i],z)))
                #print "-------",i,J
                L=J
            #print "vision jacobian last\n",J
            return J
        else:
            return self.vis2jac(uvm[0],z)

    def get_feature_error(self,desireuv,nowuv):
        kk=numpy.mat(nowuv).T-numpy.mat(desireuv).T
        #kk=numpy.mat([5,0])
        return kk.reshape((1,2))
    #cam speed (udot,vdot)(xdot,ydot,zdot,wxdot,wydot,wzdot)
    #get camera frame speed,you must change to ee frame
    def get_cam_vdot(self,uvm,z,desireuv,nowuv):
        J=self.vis2jac_mt1(uvm,z)
        JJ=numpy.linalg.pinv(J)
        e=self.get_feature_error(desireuv,nowuv)
        vdot=self.lambda1*numpy.dot(JJ,e.T)
        return vdot
    #只需要xy,z轴旋转
    def get_cam_vdot_wz(self, uvm, z, desireuv,nowuv):
        J = self.vis2jac_mt1(uvm, z)
        print "J:", J
        JJ = numpy.linalg.pinv(J)  # pseduo inverse of jacobian

        feature_error = self.get_feature_error( desireuv, nowuv )
        # print "e:", feature_error
        print "JJ:", JJ
        vdot = self.lambda1 * np.dot(JJ, feature_error.T)
        print "vdot:", vdot
        v_list = vdot.reshape((1, 6)).tolist()[0]
        flag_list = [1, 0, -1, 1, 1, 1]  # [z,x,y,wx,wz,wy ]
        vdot_z = [1.0 * v_list[i] * flag_list[i] for i in range(6)]
        # vdot_z = v_list[:2] + [0, 0, 0]
        # vdot_z.append( v_list[-1] )
        print "vdot_z:", vdot_z
        return np.matrix(vdot_z).T
    #samebody tranlasition to jacbian
    #joint speed (q0dot,q1dot,q2dot,q3dot,q4dot,q5dot)
    def get_joint_speed_bk(self,uvm,z,desireuv,nowuv,q):
        #1,get base to ee jacabian
        Jacabian_joint=self.get_jacabian_from_joint(self.urdfname,q,0)
        #2,get ee(AX=XB) to camera frame jacabian
        X=get_ur_X()#numpu array
        #tr2jac
        jac = tr2jac(X,1)
        #print "------X",X
        inv_X_jac = jac.I
        #get ee speed
        #print "tr2jac-----\n",jac
        cam_speed = self.get_cam_vdot(uvm, z, desireuv, nowuv)
        print "cam_speed--------",cam_speed
        ee_speed = np.dot(inv_X_jac, cam_speed)
        print "ee_speed-----before changing--------",ee_speed
        v_list = ee_speed.reshape((1, 6)).tolist()[0]
        flag_list = [1, 1, 0, 0, 0, 0]
        vdot_z = [1.0 * v_list[i] * flag_list[i] for i in range(6)]
        print("ee_speed_after--------------\n",vdot_z)
        j_speed=numpy.dot(Jacabian_joint.I,numpy.mat(vdot_z).T)
        return j_speed
    def get_structure_point_speed(self,sturucture_point_buf,camera_center_u0v0,f,Z_dsr,L,lamda1):
        cam=self.get_cam_data()
        pu=f/cam['kx']
        pv=f/cam['ky']
        u0=camera_center_u0v0[0]
        v0=camera_center_u0v0[1]
        u2=sturucture_point_buf[1][0]
        v2=sturucture_point_buf[1][1]
        u1=sturucture_point_buf[2][0]
        v1=sturucture_point_buf[2][1]
        u3=sturucture_point_buf[3][0]
        v3=sturucture_point_buf[3][1]
        u4=sturucture_point_buf[4][0]
        v4=sturucture_point_buf[4][1]

        x1 = (u1 - u0) * pu / f
        y1 = (v1 - v0) * pv / f
        x2 = (u2 - u0) * pu / f
        y2 = (v2 - v0) * pv / f
        x3 = (u3 - u0) * pu / f
        y3 = (v3 - v0) * pv / f
        x4 = (u4 - u0) * pu / f
        y4 = (v4 - v0) * pv / f

        s1 = 0.5 * (1 / y1 - 1 / y3)
        s2 = 0.5 * (1 / y1 + 1 / y3)
        s3 = 0.5 * (1 / x2 + 1 / x4)
        print "x2--------------------------------",x2
        s = np.array([s1,s2,s3])
        s1_d = Z_dsr / L

        s2_d = 0
        s3_d = 0
        s_d = np.array([s1_d,s2_d,s3_d])
        J_im=[0,0,-1/L,0,0,0,0,0,0,-1,0,0,0,0,0,0,3,0]
        J_im_m=np.matrix(J_im).reshape((3,6))
        print "J_im_m",J_im_m.I
        print "ssd_1111",np.matrix(s - s_d).T
        ss_d=np.matrix(s - s_d).T
        v_c = -lamda1*np.dot( J_im_m.I ,ss_d)
        print "V_c",v_c
        return v_c,s1,s2,s3
    def get_joint_speed1(self,q,v_c):
        Jacabian_joint,T_06=self.get_jacabian_from_joint(self.urdfname,q,0)
        #2,get ee(AX=XB) to camera frame jacabian
        # X=get_ur_X()#numpu array、
        X=np.array([0,0,1,0.135,1,0,0,0.145,0,1,0,0.002,0,0,0,1]).reshape(4,4)
        ebT=T_06
        print "ebt-----------------------",ebT
        #tr2jac
        jac = tr2jac_new(X,1)
        jac_b2e=tr2jac_new(T_06,0)
        #print "------X",X
        inv_X_jac = jac
        print "inv_X_jac",inv_X_jac
        cam_speed =v_c
        ee_speed_in_eeframe = np.dot(inv_X_jac, cam_speed)
        print "ee_speed-----before changing--------",ee_speed_in_eeframe

        v_list = ee_speed_in_eeframe.reshape((1, 6)).tolist()[0]
        print "v_list",v_list
        #[z,y,]
        flag_list = [0, 0, 0, 0, 0, 1]#xzyxzy
        vdot_z = [1.0 * v_list[i] * flag_list[i] for i in range(6)]
        ee_speed_in_base = np.dot(jac_b2e , numpy.mat(vdot_z).T)
        print "vdot_z-------------", vdot_z
        print("ee_speed_after--------------\n",ee_speed_in_base)
        j_speed=numpy.dot(Jacabian_joint.I,ee_speed_in_base)
        return j_speed

    def get_joint_speed2(self, q, v_c):
        Jacabian_joint, T_06 = self.get_jacabian_from_joint(self.urdfname, q, 0)
        # 2,get ee(AX=XB) to camera frame jacabian
        X = get_ur_X()  # numpu array
        ebT = T_06
        # tr2jac
        jac = tr2jac(X, 1)
        jac_b2e = tr2jac(T_06, 0)
        # print "------X",X
        inv_X_jac = jac.I
        print "inv_X_jac", inv_X_jac
        cam_speed = v_c
        ee_speed_in_eeframe = np.dot(inv_X_jac, cam_speed)
        v_list = ee_speed_in_eeframe.reshape((1, 6)).tolist()[0]
        print "v_list", v_list
        # [z,y,]
        flag_list = [1, 1, 1, 1, 1, 1]  # xzyxzy
        vdot_z = [1.0 * v_list[i] * flag_list[i] for i in range(6)]
        ee_speed_in_base = np.dot(jac_b2e.I, numpy.mat(vdot_z).T)
        print "ee_speed-----before changing--------", ee_speed_in_base

        print("ee_speed_after--------------\n", vdot_z)
        j_speed = numpy.dot(Jacabian_joint.I, ee_speed_in_base)
        return j_speed
    def get_deta_joint_angular1(self,detat,q,v_c):
        j_speed=self.get_joint_speed1(q,v_c)
        #print j_speed
        joint_angular=float(detat)*numpy.array(j_speed)
        #print '-------joint_angular-----\n',joint_angular
        return joint_angular
    def get_joint_speed(self,uvm,z,desireuv,nowuv,q):
        #1,get base to ee jacabian
        Jacabian_joint,T_06=self.get_jacabian_from_joint(self.urdfname,q,0)
        #2,get ee(AX=XB) to camera frame jacabian
        X=get_ur_X()#numpu array
        ebT=T_06
        #tr2jac
        jac = tr2jac(X,1)
        jac_b2e=tr2jac(T_06,0)
        #print "------X",X
        inv_X_jac = jac.I
        #get ee speed
        #print "tr2jac-----\n",jac
        cam_speed = self.get_cam_vdot(uvm, z, desireuv, nowuv)
        print "cam_speed--------",cam_speed
        ee_speed_in_eeframe = np.dot(inv_X_jac, cam_speed)
        v_list = ee_speed_in_eeframe.reshape((1, 6)).tolist()[0]
        #[z,y,]
        flag_list = [1, 1, 1, 1, 1, 1]
        vdot_z = [1.0 * v_list[i] * flag_list[i] for i in range(6)]
        ee_speed_in_base = np.dot(jac_b2e.I, numpy.mat(vdot_z).T)
        print "ee_speed-----before changing--------",ee_speed_in_base

        print("ee_speed_after--------------\n",vdot_z)
        j_speed=numpy.dot(Jacabian_joint.I,ee_speed_in_base)
        return j_speed
    #
    def get_deta_joint_angular(self,detat,uvm,z,desireuv,nowuv,q):
        j_speed=self.get_joint_speed(uvm,z,desireuv,nowuv,q)
        #print j_speed
        joint_angular=float(detat)*numpy.array(j_speed)
        #print '-------joint_angular-----\n',joint_angular
        return joint_angular
    def get_joint_angular(self,qnow,detajoint):
        #result=[]
        listangular=[]
        for i in range(len(detajoint.tolist())):
            listangular.append(detajoint.tolist()[i][0]+qnow[i])
        print "list",detajoint.tolist()
        return listangular

def main():
    urdfname="/data/ros/ur_ws_yue/src/ur5_planning/urdf/ur5.urdf"
    filename="/data/ros/ur_ws_yue/src/ur5_planning/yaml/cam_500_industry.yaml"
    # urdfname="/data/ros/ur_ws/src/universal_robot/ur_description/urdf/ur5.urdf"
    desiruv=[]
    # desiruv=[[168,169]]
    camera_center_u0v0=[327,256]
    lambda1=0.666666
    detat=0.05
    z=0.25
    ace=50
    vel=0.1
    urt=0
    ratet=3
    f=0.4762192#m
    L=0.06#m
    lamda2=0.1
    Z_dsr=0.2#m
    p0=VisonControl(filename,0,lambda1,urdfname)


    #2get uv from ar
    # ar_reader = arReader()
    # ar_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, ar_reader.callback)

    ur_reader = Urposition()
    ur_sub = rospy.Subscriber("/joint_states", JointState, ur_reader.callback)

    feature_vector_s1_pub = rospy.Publisher("/feature_vector_s1", Float64, queue_size=10)
    feature_vector_s2_pub = rospy.Publisher("/feature_vector_s2", Float64, queue_size=10)
    feature_vector_s3_pub = rospy.Publisher("/feature_vector_s3", Float64, queue_size=10)

    feature_vector_v1_pub = rospy.Publisher("/feature_vector_v1", Float64, queue_size=10)
    feature_vector_v2_pub = rospy.Publisher("/feature_vector_v2", Float64, queue_size=10)
    feature_vector_v3_pub = rospy.Publisher("/feature_vector_v3", Float64, queue_size=10)
    feature_vector_v4_pub = rospy.Publisher("/feature_vector_v4", Float64, queue_size=10)
    feature_vector_v5_pub = rospy.Publisher("/feature_vector_v5", Float64, queue_size=10)
    feature_vector_v6_pub = rospy.Publisher("/feature_vector_v6", Float64, queue_size=10)

    now_uv_pub = rospy.Publisher("/nowuv_info", uv, queue_size=10)
    #give q to ur3
    ur_pub = rospy.Publisher("/ur_driver/URScript", String, queue_size=10)

    #get uvlist for circle
    uv_get=UVRead()
    uv_sub=rospy.Subscriber("/camera_uv/uvlist", uv,uv_get.callback)
    struv_sub=StructurePointUvRead()
    subb = rospy.Subscriber("/structure_point_uv", structure_point, struv_sub.structure_point_callback)
    rate = rospy.Rate(ratet)
    while not rospy.is_shutdown():
        # if len(struv_sub.sturucture_point_0_buf)!=0:
        #     p0.get_structure_point_speed(struv_sub.sturucture_point_0_buf[-1],camera_center_u0v0,f,Z_dsr,L,lamda2)
        desiruv=[]
        uvlist=[]

        if len(struv_sub.sturucture_point_0_buf) != 0:

            q_now=ur_reader.ave_ur_pose
            v_c,s1,s2,s3=p0.get_structure_point_speed(struv_sub.sturucture_point_0_buf[-1], camera_center_u0v0, f, Z_dsr, L, lamda2)
            #get joint speed in ee frame
            print "##############################################################"
            print "q_now\n", q_now
            print "joint speed\n",p0.get_joint_speed1(q_now,v_c)
            j_speed=p0.get_joint_speed1(q_now,v_c)
            print "##############################################################"
            print "deta joint angular---"
            feature_vector_s1_pub.publish(s1)
            feature_vector_s2_pub.publish(s2)
            feature_vector_s3_pub.publish(s3)
            feature_vector_v1_pub.publish(j_speed[0])
            feature_vector_v2_pub.publish(j_speed[1])
            feature_vector_v3_pub.publish(j_speed[2])
            feature_vector_v4_pub.publish(j_speed[3])
            feature_vector_v5_pub.publish(j_speed[4])
            feature_vector_v6_pub.publish(j_speed[5])
            "##################################################################"
            detaangular=p0.get_deta_joint_angular1(detat, q_now,v_c)
            print detaangular
            print "##############################################################"
            print "joint angular----"
            q_pub_now=p0.get_joint_angular(q_now,detaangular)
            print q_pub_now
            print "##############################################################"
            print "move ur base the servo system----"
            print "q_now\n", q_now
            print "q_pub_now\n",q_pub_now
            ss = "movej([" + str(q_pub_now[0]) + "," + str(q_pub_now[1]) + "," + str(q_pub_now[2]) + "," + str(
                q_pub_now[3]) + "," + str(q_pub_now[4]) + "," + str(q_pub_now[5]) + "]," + "a=" + str(ace) + "," + "v=" + str(
                vel) + "," + "t=" + str(urt) + ")"
            print ss
            ur_pub.publish(ss)
            rate.sleep()
        else:
            continue

if __name__=="__main__":
    main()