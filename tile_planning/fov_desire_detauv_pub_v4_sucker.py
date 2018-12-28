#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy,math
import Quaternion as Q
import time
from numpy import linalg
import yaml
import os
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from pykdl_utils.kdl_kinematics import KDLKinematics

from ur5_planning.msg import uv
from ur5_planning.msg import tileuv
from sensor_msgs.msg import JointState
from ur5_pose_get import *
from std_msgs.msg import UInt16,Float64

from std_msgs.msg import String
# from tile_uv_sub_node import *

from led_state_sub import *
from frompitoangle  import *
"""
os.system("rostopic pub io_state std_msgs/String "55C8010155" --once")
"""
class Fovcontrol():
    def __init__(self,nodename,urdfname,detat,lamda,califilename,camf,kappa=0.7,delta=5):
        self.nodename=nodename
        self.califilename=califilename
        self.urdfname=urdfname
        self.camf=camf
        self.detat=detat
        self.kappa=kappa
        self.delta=delta
        self.lamda=lamda

        self.tile_0_buf=[]
        self.tile_1_buf = []
        self.ledstate=None
        self.changeuv=None
        self.w_pub = rospy.Publisher("/w_param", Float64, queue_size=10)
    def Init_node(self):
        rospy.init_node(self.nodename)
        # tile_reader = TileUvRead()
        tileuv_sub = rospy.Subscriber("/tile_uv", tileuv, self.callback)
        ur_pub = rospy.Publisher("/ur_driver/URScript", String, queue_size=10)
        return ur_pub
    """
    0:o
    1:d
    """
    def callback(self, msg):
        if msg.tile_id == 0:
            if len(self.tile_0_buf) == 10:
                self.tile_0_buf = self.tile_0_buf[1:]
                tile_id = msg.tile_id
                cen_uv = msg.cen_uv
                f1th_uv = msg.f1th_uv
                s2th_uv = msg.s2th_uv
                t3th_uv = msg.t3th_uv
                f4th_uv = msg.f4th_uv
                self.tile_0_buf.append(
                    [tile_id, cen_uv.uvinfo, f1th_uv.uvinfo, s2th_uv.uvinfo, t3th_uv.uvinfo, f4th_uv.uvinfo])
                # print "---------self.cross_uv_buf",self.cross_uv_buf
            else:
                tile_id = msg.tile_id
                cen_uv = msg.cen_uv
                f1th_uv = msg.f1th_uv
                s2th_uv = msg.s2th_uv
                t3th_uv = msg.t3th_uv
                f4th_uv = msg.f4th_uv
                self.tile_0_buf.append(
                    [tile_id, cen_uv.uvinfo, f1th_uv.uvinfo, s2th_uv.uvinfo, t3th_uv.uvinfo, f4th_uv.uvinfo])

        elif msg.tile_id == 1:
            if len(self.tile_1_buf) == 10:
                self.tile_0_buf = self.tile_0_buf[1:]
                tile_id = msg.tile_id
                cen_uv = msg.cen_uv
                f1th_uv = msg.f1th_uv
                s2th_uv = msg.s2th_uv
                t3th_uv = msg.t3th_uv
                f4th_uv = msg.f4th_uv
                self.tile_1_buf.append(
                    [tile_id, cen_uv.uvinfo, f1th_uv.uvinfo, s2th_uv.uvinfo, t3th_uv.uvinfo, f4th_uv.uvinfo])
                # print "---------self.cross_uv_buf",self.cross_uv_buf
            else:
                tile_id = msg.tile_id
                cen_uv = msg.cen_uv
                f1th_uv = msg.f1th_uv
                s2th_uv = msg.s2th_uv
                t3th_uv = msg.t3th_uv
                f4th_uv = msg.f4th_uv
                self.tile_1_buf.append(
                    [tile_id, cen_uv.uvinfo, f1th_uv.uvinfo, s2th_uv.uvinfo, t3th_uv.uvinfo, f4th_uv.uvinfo])
        else:
            print "wait opencv caulate tile uv ----"
            time.sleep(1)
        # print " msg.tile_id", msg.tile_id

    def Get_X_from_cali_quaternion(self,calibinfo):
        transition_L = numpy.array(calibinfo[:3]).T
        # print transition_L
        rot = calibinfo[3:6]
        s = calibinfo[6]
        q0 = Q.quaternion(s, numpy.mat(rot))
        # print "q02R--------\n", q0.r()
        # print q0.r()
        T_part1 = numpy.column_stack((q0.r(), transition_L))
        # print T_part1
        T_part2 = numpy.array([0, 0, 0, 1])
        # print T_part2
        T = numpy.row_stack((T_part1, T_part2))
        # print T
        T = T.tolist()
        T = T[0] + T[1] + T[2] + T[3]
        # print("T:" , T)
        return T
    """
    AX=XB
        # calibration_info = [
            translation: 
              x: -0.113752906534
              y: 0.331789837221
              z: 0.291185239983
            rotation: 
              x: -0.441807188502
              y: 0.561689021804
              z: -0.558754758427
              w: 0.42083841425
        #translation: 
          x: 0.134484285561
          y: 0.0795027844047
          z: 0.110225634787
        rotation: 
          x: 0.193356517556
          y: 0.645613390532
          z: 0.235883157601
          w: 0.700111236194
        #
        # ]
    """
    def Get_ur_X(self,info):

        aa = self.Get_X_from_cali_quaternion(info)
        aa = numpy.mat(aa)
        # print "X", aa
        return aa.reshape((4, 4))
    def get_jacabian_from_joint(self,urdfname,jointq):
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
        # # print pose
        # #print list(pose)
        # q0=Kinematic(q)
        # if flag==1:
        #     q_ik=q0.best_sol_for_other_py( [1.] * 6, 0, q0.Forward())
        # else:
        #     q_ik = kdl_kin.inverse(pose)  # inverse kinematics
        # # print "----------iverse-------------------\n", q_ik
        #
        # if q_ik is not None:
        #     pose_sol = kdl_kin.forward(q_ik)  # should equal pose
        #     print "------------------forward ------------------\n",pose_sol

        J = kdl_kin.jacobian(q)
        #print 'J:', J
        return J,pose

    def tr2jac(self,T, samebody):
        # T = np.array(T)
        R = self.tr2r(T)
        # jac = np.zeros((6, 6))
        """
        jac = [ jac_part1,  jac_part2;
                jac_part3,  jac_part4;
                    ]
        """
        if samebody == 1:
            jac_part1 = R.T
            jac_part2 = -numpy.dot(R.T, self.skew(self.transl(T)))
            jac_part3 = numpy.zeros((3, 3))
            jac_part4 = R.T

        else:
            jac_part1 = R.T
            jac_part2 = numpy.zeros((3, 3))
            jac_part3 = numpy.zeros((3, 3))
            jac_part4 = R.T
        jac_row1 = numpy.column_stack((jac_part1, jac_part2))
        jac_row2 = numpy.column_stack((jac_part3, jac_part4))
        jac = numpy.row_stack((jac_row1, jac_row2))
        return jac
    def tr2jac_new(self,T, samebody):
        # T = np.array(T)
        R = self.tr2r(T)
        # jac = np.zeros((6, 6))
        """
        jac = [ jac_part1,  jac_part2;
                jac_part3,  jac_part4;
                    ]
        """
        if samebody == 1:
            jac_part1 = R
            New_trans = numpy.dot(-1 * (R.I), self.transl(T))
            jac_part2 = -numpy.dot(R, self.skew(New_trans))
            # print "self.transl(T))",self.transl(T)
            # T1=[1,2,3]
            # print "self.skew(self.transl(T))\n",self.skew(New_trans)
            jac_part3 = numpy.zeros((3, 3))
            jac_part4 = R

        else:
            jac_part1 = R
            jac_part2 = numpy.zeros((3, 3))
            jac_part3 = numpy.zeros((3, 3))
            jac_part4 = R
        jac_row1 = numpy.column_stack((jac_part1, jac_part2))
        jac_row2 = numpy.column_stack((jac_part3, jac_part4))
        jac = numpy.row_stack((jac_row1, jac_row2))
        return jac

    """
    if l is 3*1 , then get 
    skew(l) = [ 0, -l(2), l(1)
                l(2), 0 , -l(0)
                -l(1), l(0), 0]
    if l is 1*1, then get
    skew(l) = [ 0 , -l[0]
                l[0], 0 ]

    """

    def skew(self,l):
        a, b = numpy.shape(l)
        try:
            if a == 3:
                s = numpy.array([0, -l[2], l[1], l[2], 0, -l[0], -l[1], l[0], 0])
                s = s.reshape((3, 3))
                # print "s:", s
                return s
            elif a == 1:
                s = numpy.array([0, -l[0], l[0], 0])
                s = s.reshape((2, 2))
                return s
        except:
            print("erro l size!!!  3*1 or 1*1 required!")

    def tr2r(self,T):
        r = [0, 1, 2]
        c = [0, 1, 2]
        R1 = T[r]
        R = R1[:, c]
        return R

    def transl(self,T):
        r = [3]
        c = [0, 1, 2]
        l1 = T[:, r]
        l = l1[c]
        return l
    def get_cam_data(self):
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
            [627.260603, 0.000000, 316.404078, 0.000000, 622.895967, 251.341039, 0.000000, 0.000000, 1.000000])
        instrinc_param = data.reshape((3, 3))
       # print(instrinc_param)
        return instrinc_param

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
    #cal image jacbian

    def vis2jac(self,uv,z):
        cam=self.get_cam_data()
        rh0=[0.0000032,0.0000032]
        camf=self.camf#m
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
    #uvm means now uv
    def get_cam_vdot(self,uvm,z,desireuv,nowuv):
        J=self.vis2jac_mt1(uvm,z)
        JJ=numpy.linalg.pinv(J)
        e=self.get_feature_error(desireuv,nowuv)
        vdot=self.lamda*numpy.dot(JJ,e.T)
        return vdot

    #只需要xy,z轴旋转
    def get_cam_vdot_wz(self, uvm, z, desireuv,nowuv):
        J = self.vis2jac_mt1(uvm, z)
        # print "J:", J
        JJ = numpy.linalg.pinv(J)  # pseduo inverse of jacobian

        feature_error = self.get_feature_error( desireuv, nowuv )
        # print "e:", feature_error
        # print "JJ:", JJ
        vdot = self.lamda * numpy.dot(JJ, feature_error.T)
        # print "vdot:", vdot
        v_list = vdot.reshape((1, 6)).tolist()[0]
        flag_list = [1, 0, 1, 1, 1, 1]  # [z,x,y,wx,wz,wy ]
        vdot_z = [1.0 * v_list[i] * flag_list[i] for i in range(6)]
        # vdot_z = v_list[:2] + [0, 0, 0]
        # vdot_z.append( v_list[-1] )
        # print "vdot_z:", vdot_z
        return numpy.matrix(vdot_z).T

    #samebody tranlasition to jacbian
    #joint speed (q0dot,q1dot,q2dot,q3dot,q4dot,q5dot)
    def get_joint_speed(self,uvm,z,desireuv,nowuv,q,info):
        #1,get base to ee jacabian
        Jacabian_joint,pose=self.get_jacabian_from_joint(self.urdfname,q)
        # print ""
        #2,get ee(AX=XB) to camera frame jacabian
        X=self.Get_ur_X(info)#numpu array
        #tr2jac
        jac = self.tr2jac(X,1)
        #print "------X",X
        inv_X_jac = jac.I
        #get ee speed
        #print "tr2jac-----\n",jac
        cam_speed = self.get_cam_vdot(uvm, z, desireuv, nowuv)
        # print "cam_speed--------\n",cam_speed
        ee_speed = numpy.dot(inv_X_jac, cam_speed)
        # print "ee_speed-----before changing--------\n",ee_speed
        v_list = ee_speed.reshape((1, 6)).tolist()[0]
        flag_list = [1, 0, 1, 1, 0, 0]
        vdot_z = [1.0 * v_list[i] * flag_list[i] for i in range(6)]
        # print("ee_speed_after--------------\n",vdot_z)
        j_speed=numpy.dot(Jacabian_joint.I,numpy.mat(vdot_z).T)
        return j_speed
    # def get_joint_speed(self,q,ee_speed):
    #     #1,get base to ee jacabian
    #     Jacabian_joint=self.get_jacabian_from_joint(self.urdfname,q)
    #     #2,get ee(AX=XB) to camera frame jacabian
    #     X=self.Get_ur_X(self.calibinfo)#numpu array
    #     # print "X",X
    #     # print "Jacabian_joint",Jacabian_joint[0]
    #     #tr2jac
    #     jac = self.tr2jac(X,1)
    #     #print "------X",X
    #     inv_X_jac = jac.I
    #     j_speed=numpy.dot(Jacabian_joint[0].I,ee_speed)
    #
    #     return j_speed
    #
    def get_deta_joint_angular(self,j_speed):
        #print j_speed
        joint_angular=float(self.detat)*numpy.array(j_speed)
        #print '-------joint_angular-----\n',joint_angular
        return joint_angular

    def get_joint_angular(self,qnow,detajoint):
        #result=[]
        listangular=[]
        for i in range(len(detajoint.tolist())):
            listangular.append(detajoint.tolist()[i][0]+qnow[i])
        # print "list",detajoint.tolist()
        return listangular
    def From_matrix_to_list(self,data):
        temp=data.tolist()
        result=[]
        for i in xrange(len(temp)):
            for ii in xrange(len(temp)):
                result.append(temp[i][ii])
        return result
    """
    x:real time uv from cv[u,v],this case means camera central point 
    x0:object uv,[u0,v0]
    xd:desire uv [ud,vd]
    f=(x-x0)'*(x-x0)-detat^2
    mm
    rc1=0.56
    rc2=-0.145
    b=0.1
    detax
    led13:high pick tile,and high data=1,low place tile
    """
    def get_fov_desire_uv(self,x,xo,xd,flag):
        rc1=0.56
        rc2=-0.145
        b=0.1
        N=4
        f=numpy.dot(numpy.mat(numpy.array(x)-numpy.array(xo)),(numpy.mat(numpy.array(x)-numpy.array(xo))).T)-self.delta**2
        print "f",f.tolist()[0][0]
        f=f.tolist()[0][0]
        a=1-(min(0,((min(0,f))**N-((self.kappa*self.delta)**2-self.delta**2)**N)))**N/(((self.kappa*self.delta)**2-self.delta**2)**(N**2))
        h = ((0.15 - rc1) ** 10)/ (b **10) + ((0.15 - rc2) **10) / (b **10) - 1
        w = 1 - (min(0, ((min(0, h)) ** N - (self.kappa ** 10 - 1) **N)))**N / (self.kappa ** 10 - 1) ** (N ** 2)
        self.w_pub.publish(w)
        xp = numpy.array(xo) - w * a * (numpy.array(x) - numpy.array(xd))
        detax=numpy.array(x)-xp
        print "detax",detax
        print "a", a
        print "h", h
        print "w", w
        if flag==1:
            return xd
        else:
        # print "xp", xp.tolist()
            return xp.tolist()
    def get_fov_desire_uv_0(self,xp):
        a=1
        print "a",a
        return xp
    #qq=[12,11,....]angular joint
    def get_detax(self,x):
        uvcen=[331,229]
        kk=numpy.mat(uvcen).T-numpy.mat(x).T
        return kk.reshape((1,2)).tolist()
    def ibvs_run_ur5(self,xo,xd,uvm,z,q,info,flag):

        """
        First,Get the uv to drive ur5
        x=uvm[0]=nowuv

        """
        xp=self.get_fov_desire_uv(uvm[0],xo,xd,flag)
        print "xp",xp
        """
        Second,caculating cam vodt and deta joint speed
        desireuv=xp
        """
        # cam_vdot=self.get_cam_vdot_wz(uvm,z,xp,uvm[0])

        joint_speed_dot=self.get_joint_speed(uvm, z, xp, uvm[0], q, info)
        print "joint_speed_dot",joint_speed_dot
        """
        Third,caculating deta joint speed
        """
        deta_joint_angular=self.get_deta_joint_angular(joint_speed_dot)
        print "deta_joint_angular",deta_joint_angular
        """
        Fourth,get joint angular
        """
        pub_joint=self.get_joint_angular(q, deta_joint_angular)
        print "pub_joint",pub_joint
        return pub_joint
    """
    A function for checking the tile is picking or no
    0:means there is no tile or the tile is picked
    """
    def check_uvbuf_is_zero(self,uv_buf):
        zero_flag=1
        count=0
        for i in uv_buf[1:]:
            if cmp((0.0,0.0),i)==0:
                count+=1
        if count==5:
            return 0
        else:
            return 1
    def Move_ur(self,q_pub_now,ace,vel,urt):
        ss = "movej([" + str(q_pub_now[0]) + "," + str(q_pub_now[1]) + "," + str(q_pub_now[2]) + "," + str(
            q_pub_now[3]) + "," + str(q_pub_now[4]) + "," + str(q_pub_now[5]) + "]," + "a=" + str(
            ace) + "," + "v=" + str(
            vel) + "," + "t=" + str(urt) + ")"
        return ss
    def getpi(self,listb):
        lista=[]
        listcc=[]
        for i in listb:
            temp=i/180*3.14
            lista.append((temp,i))
            listcc.append(temp)
        return listcc
    def Open_sucking_close(self,flag):
        Protocol="55C8010"+str(flag)+"55"
        Pub_str='rostopic pub io_state std_msgs/String '+Protocol+' --once'
        os.system(Pub_str)
def main():
    #uvlist=[123.0,112.0]
    uvlist=[]
    camf=624.0429 * 1e-03
    # uvcentral=[316,251]
    uvcentral = [358, 219]#sucking central
    First_joint_angular=[]
    calibinfo=[
        -0.0564433333037,
        0.0458615556053,
        -0.0396379835814,
        -0.637915599891,
        0.330383084363,
        -0.420221772942,
        0.554368439332
    ]
    urdfname = "/data/ros/ur_ws/src/universal_robot/ur5_planning/urdf/ur5.urdf"
    cailename = "/data/ros/ur_ws/src/universal_robot/ur5_planning/yaml/cam_500_logitech.yaml"
    nodename="fov_vision_control"
    ace=50
    vel=0.3
    urt=0
    detat=0.05
    ratet=30
    lamda=5
    z=0.6
    ur_reader = Urposition()
    ur_sub = rospy.Subscriber("/joint_states", JointState, ur_reader.callback)
    u_error_pub = rospy.Publisher("/feature_u_error", Float64, queue_size=10)
    v_error_pub = rospy.Publisher("/feature_v_error", Float64, queue_size=10)
    """
    Those two flag use to make sucker sucking tile
    """
    object_flag=0
    open_desire_flag= 0
    desire_flag=0
    """
    nodename,urdfname,delta,kappa,lamda,califilename,camf)
    """
    F0=Fovcontrol(nodename,urdfname,detat,lamda,cailename,camf)
    ur_pub=F0.Init_node()
    rate = rospy.Rate(ratet)
    First_joint_angular = ur_reader.ave_ur_pose

    Object_joint_angular_vision_state=[]
    Object_joint_angular_sucking_state = [307.61, -136.58, 65.96, -120.66, -306.25, -215.74]

    Desire_joint_angular_vision_state=[]
    Desire_joint_angular_place_state = [266.09, -178.29, 117.69, -121.76, -267.38, -216.20]

    print "First_joint_angular",First_joint_angular
    while not rospy.is_shutdown():
        try:
            """
            First,Go to object position,just need opreating tile 0,
            Now UV also is [316,251]
            """
            q_now = ur_reader.ave_ur_pose

            print "q_now",q_now
            if len(q_now) != 0:
                if len(F0.tile_0_buf)!=0:
                    if F0.check_uvbuf_is_zero(F0.tile_0_buf[-1]) != 0:
                        print "F0.tile_0_buf",F0.tile_0_buf[-1]
                        xd=[100,100]
                        xo=F0.tile_0_buf[-1][1]
                        uvm=[uvcentral]
                        q_pub_now=F0.ibvs_run_ur5(xo, xd, uvm, z, q_now, calibinfo,0)
                        feature_error = F0.get_feature_error(xo, uvm[0])
                        print "feature error\n", feature_error
                        print feature_error.tolist()
                        u_error_pub.publish(feature_error.tolist()[0][0])
                        v_error_pub.publish(feature_error.tolist()[0][1])
                        if abs(feature_error.tolist()[0][0])<=3 and abs(feature_error.tolist()[0][1])<=3:
                            object_flag=1
                            Object_joint_angular_vision_state=q_pub_now

                        ss=F0.Move_ur(q_pub_now,ace,vel,urt)
                        print ss
                        ur_pub.publish(ss)
                        # print "F0.tile_1_buf", F0.tile_1_buf[-1]

                """
                Second,Cartesian-space feedback dirve the UR5 back ,because of vision feedback is invalid
                """
                if len(F0.tile_0_buf) != 0:
                    if object_flag:
                        time.sleep(2)
                        print "sleep some seconds for sucking tile"

                        q_pub_now_sucking = F0.getpi(Object_joint_angular_sucking_state)
                        ss = F0.Move_ur(q_pub_now_sucking, ace, vel, urt)
                        print ss
                        ur_pub.publish(ss)
                        time.sleep(6)
                        print "Open sucker to pick object tile"
                        F0.Open_sucking_close(1)
                        time.sleep(2)

                    if F0.check_uvbuf_is_zero(F0.tile_0_buf[-1])==0 and F0.check_uvbuf_is_zero(F0.tile_1_buf[-1])==0:
                        print "Use cartesian-space feedback"
                        q_pub_now=First_joint_angular
                        ss=F0.Move_ur(q_pub_now,ace,vel,urt)
                        print ss
                        ur_pub.publish(ss)
                        open_desire_flag=1
                        time.sleep(2)
                        object_flag=0

                """
                Third,Go to desire position,just need opreating tile 1,
                Now UV also is [316,251]
                """
                if len(F0.tile_1_buf)!=0 and open_desire_flag:
                    # print "F0.tile_1_buf",F0.tile_1_buf[-1]
                    if F0.check_uvbuf_is_zero(F0.tile_1_buf[-1]) != 0:
                        print "F0.tile_1_buf",F0.tile_1_buf[-1]
                        xo=[100,100]
                        xd=F0.tile_1_buf[-1][1]
                        # xd=[100,100]
                        # xo=F0.tile_1_buf[-1][1]
                        uvm=[uvcentral]
                        q_pub_now=F0.ibvs_run_ur5(xo, xd, uvm, z, q_now, calibinfo,1)
                        feature_error = F0.get_feature_error(xd, uvm[0])
                        print "feature error\n", feature_error
                        print feature_error.tolist()
                        u_error_pub.publish(feature_error.tolist()[0][0])
                        v_error_pub.publish(feature_error.tolist()[0][1])
                        if abs(feature_error.tolist()[0][0])<=5 and abs(feature_error.tolist()[0][1])<=5:
                            desire_flag=1
                            """
                            Fourth,Placing tile,
                            Now UV also is [316,251]
                            """
                            time.sleep(1)
                            print "sleep some seconds for place tile"
                            q_pub_now_placing = F0.getpi(Desire_joint_angular_place_state)
                            ss = F0.Move_ur(q_pub_now_placing, ace, vel, urt)
                            print ss
                            ur_pub.publish(ss)
                            time.sleep(6)
                            print "Close sucker to place object tile"
                            F0.Open_sucking_close(0)
                            time.sleep(0.5)
                            print "move back starting Pose"
                            ss = F0.Move_ur(First_joint_angular, ace, vel, urt)
                            print ss
                            ur_pub.publish(ss)
                            time.sleep(2)
                            # print "F0.tile_1_buf", F0.tile_1_buf[-1]
                            # print "F0.tile_1_buf", F0.tile_1_buf[-1]
                        if desire_flag==0:
                            ss=F0.Move_ur(q_pub_now,ace,vel,urt)
                            print ss
                            ur_pub.publish(ss)


            else:
                print "UR5 is Not Ok,Please check"
        except KeyboardInterrupt:
            sys.exit()

        rate.sleep()
if __name__=="__main__":
    main()

