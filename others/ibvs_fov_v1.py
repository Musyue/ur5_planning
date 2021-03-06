#!/usr/bin/env python
# -*- coding: utf-8 -*-

import src.ur5_planning.scripts.frompitoangle
import numpy
from numpy import matlib,linalg
#Import the module
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from pykdl_utils.kdl_kinematics import KDLKinematics
from src.ur5_planning.scripts.new_kinematics import Kinematic
#import cv2
import rospy
import yaml,os
from src.ur5_planning.scripts.trans_methods import *
from src.ur5_planning.scripts.get_arpose_from_ar import *
from ar_track_alvar_msgs.msg import AlvarMarkers
from src.ur5_planning.scripts.hand_in_eye import *
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from src.ur5_planning.scripts.ur5_pose_get import *
from std_msgs.msg import Float64
from ur5_planning.msg import uv
from src.ur5_planning.scripts.xp_uv_sub_node import *




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
        q0=Kinematic(q)
        if flag==1:
            q_ik=q0.best_sol_for_other_py( [1.] * 6, 0, q0.Forward())
        else:
            q_ik = kdl_kin.inverse(pose)  # inverse kinematics
        # print "----------iverse-------------------\n", q_ik

        if q_ik is not None:
            pose_sol = kdl_kin.forward(q_ik)  # should equal pose
            print "------------------forward ------------------\n",pose_sol

        J = kdl_kin.jacobian(q)
        #print 'J:', J
        return J
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
            [854.095755, 0.000000, 331.439357, 0.000000, 853.591646, 229.264580, 0.000000, 0.000000, 1.000000])
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
        camf=0.8557602135243908#m
        kx = cam['kx']
        ky = cam['ky']
        #--------------sgl-------------
        # print kx
        arfx=kx/camf
        arfy=ky/camf
        # kx=arfx*camf
        # ky=arfy*camf
        uba=uv[0]-cam['u0']
        vba=uv[1]-cam['v0']
        L=[[-arfx/z,0,uba/z,1/arfx*uba*vba,-(arfx**2+uba**2)/arfx,vba,0,-arfy/z,vba/z,(arfy**2+vba**2)/arfy,-uba*vba/arfx,-uba]]
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
        return kk.reshape((1,2))
    #cam speed (udot,vdot)(xdot,ydot,zdot,wxdot,wydot,wzdot)
    #get camera frame speed,you must change to ee frame
    def get_cam_vdot(self,uvm,z,desireuv,nowuv):
        J=self.vis2jac_mt1(uvm,z)
        JJ=numpy.linalg.pinv(J)
        e=self.get_feature_error(desireuv,nowuv)
        vdot=self.lambda1*numpy.dot(JJ,e.T)
        return vdot
    #samebody tranlasition to jacbian
    #joint speed (q0dot,q1dot,q2dot,q3dot,q4dot,q5dot)
    def get_joint_speed(self,uvm,z,desireuv,nowuv,q):
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
        ee_speed = np.dot(inv_X_jac, cam_speed)
        #print("ee_speed\n",ee_speed)
        j_speed=numpy.dot(Jacabian_joint.I,ee_speed)
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
    urdfname="/data/ros/ur_ws/src/universal_robot/ur5_planning/urdf/ur5.urdf"
    filename="/data/ros/ur_ws/src/universal_robot/ur5_planning/yaml/cam_500_logitech.yaml"
    # urdfname="/data/ros/ur_ws/src/universal_robot/ur_description/urdf/ur5.urdf"
    desiruv=[[331,229]]
    lambda1=-1.0
    detat=0.05
    z=0.7
    ace=50
    vel=0.1
    urt=0
    ratet=5
    p0=VisonControl(filename,0,lambda1,urdfname)


    #2get uv from ar
   # ar_reader = arReader()
    #ar_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, ar_reader.callback)

    ur_reader = Urposition()
    ur_sub = rospy.Subscriber("/joint_states", JointState, ur_reader.callback)

    u_error_pub = rospy.Publisher("/feature_u_error", Float64, queue_size=10)
    v_error_pub = rospy.Publisher("/feature_v_error", Float64, queue_size=10)
    z_depth_pub = rospy.Publisher("/camera_depth", Float64, queue_size=10)
    now_uv_pub = rospy.Publisher("/nowuv_info", uv, queue_size=10)
    #give q to ur3
    ur_pub = rospy.Publisher("/ur_driver/URScript", String, queue_size=10)

    #get uvlist for circle
    xpuv_get=XpRead()
    xpuv_sub=rospy.Subscriber("/xp_uv/xp", uv,xpuv_get.callback)

    rate = rospy.Rate(ratet)
    while not rospy.is_shutdown():
        # desiruv=[]
        uvlist=[]
        if len(xpuv_get.uvlist_buf)==0:
            print "wait subscribe xp uv-----"
            time.sleep(3)
            continue
        else:
            print "xp cross_uv_buf",xpuv_get.uvlist_buf[-1]
            uvlist.append(xpuv_get.uvlist_buf[-1])

        print "##############################################################"
        print "uv-list------\n",uvlist
        now_uv_pub.publish(uvlist[0])
        print "##############################################################"
        print "###########################################################"
        # if len(uv_get.cross_uv_buf)==0:
        #     print "wait desire data sub---\n"
        #     time.sleep(3)
        # print "desire uv------\n", uv_get.cross_uv_buf[-1]
        # desiruv.append(uv_get.cross_uv_buf[-1])
        #get error
        print "##############################################################"
        feature_error=p0.get_feature_error(desiruv,uvlist[0])
        print "feature error\n",feature_error
        print feature_error.tolist()
        u_error_pub.publish(feature_error.tolist()[0][0])
        v_error_pub.publish(feature_error.tolist()[0][1])
        print "##############################################################"
        #get visual jacbian
        #print "visual jacbian"
        #print p0.vis2jac([612,412],z)
        print "##############################################################"
        #get cam speed vdot
        print "camera vdot\n",p0.get_cam_vdot(uvlist,z,desiruv,uvlist[0])
        print "##############################################################"
        q_now=ur_reader.ave_ur_pose
        #get joint speed in ee frame
        print "##############################################################"
        print "q_now\n", q_now
        print "joint speed\n",p0.get_joint_speed(uvlist,z,desiruv,uvlist[0],q_now)
        print "##############################################################"
        print "deta joint angular---"
        detaangular=p0.get_deta_joint_angular(detat,uvlist, z, desiruv, uvlist[0], q_now)
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

if __name__=="__main__":
    main()