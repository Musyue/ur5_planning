#!/usr/bin/env python
# -*- coding: utf-8 -*-
#pos 0:06T0,Te0
#:06T0
#Te0:camera extrinsic parameters
#pos 1:06T1,Te1
#pos 2:06T2,Te2
import math
from numpy import *
import numpy as np
import Quaternion as Q
from ur5_kdl_from_urdf import *
from frompitoangle import *
from ur5_kinematics import *
import tf
from trans_methods import *
class HandToEye():
    def __init__(self,T0,Te0,T1,Te1,T2,Te2):
        self.T0=T0
        self.Te0=Te0
        self.T1=T1
        self.Te1=Te1
        self.T2=T2
        self.Te2=Te2
    def caculat_C_D(self,xTe1,xTe0,xT0,xT1):
        #Bc1 pos1 extrinsic parameters
        #Ac1 pos0 extrinsic parameters
        #Ad1 pos0 T
        #Bd1 pose1 T
        Bc1=xTe1
        Ac1=xTe0
        Ad1=xT0
        Bd1=xT1
        mBc1=np.mat(Bc1)
        mAc1=np.mat(Ac1)
        invmBc1=np.reshape(mBc1,(4,4)).I
        mAc1to4x4=np.reshape(mAc1,(4,4))
        C1=mAc1to4x4*invmBc1
        # print "C1----->\n",C1
        mAd1=np.mat(Ad1)
        mBd1=np.mat(Bd1)
        d1=(np.reshape(mAd1,(4,4)).I)*(np.reshape(mBd1,(4,4)))
        # print "d1------->\n",d1
        return C1,d1
    def from0to1(self,Te1,Te0,T0,T1):
        return self.caculat_C_D(Te1,Te0,T0,T1)
    def from1to2(self,Te2,Te1,T1,T2):
        return self.caculat_C_D(Te2,Te1,T1,T2)
    def vector_quaternion(self,R):
        result=[]
        q=math.acos((np.trace(R)-1)/2)
        # print "q:",q
        q1=(q/(2*math.sin(q)))*(R[2,1]-R[1,2])
        # print "q1:",q1
        q2=(q/(2*math.sin(q)))*(R[0,2]-R[2,0])
        # print "q2:",q2
        q3=(q/(2*math.sin(q)))*(R[1,0]-R[0,1])
        # print "q3:",q3
        result.append(q)
        result.append(q1)
        result.append(q2)
        result.append(q3)
        return result
    #AX=XB
    #R=[kc1,kc2,kc3]*inv[kd1,kd2,kd3]
    def final_X(self):
        c1,d1=self.from0to1(self.Te1,self.Te0,self.T0,self.T1)
        c2,d2=self.from1to2(self.Te2,self.Te1,self.T1,self.T2)
        rc1=c1[[0,1,2]][:,[0,1,2]]
        kc1=self.vector_quaternion(c1[[0,1,2]][:,[0,1,2]])[1:]
        # print mat(kc1)
        kc2=self.vector_quaternion(c2[[0,1,2]][:,[0,1,2]])[1:]
        rc2=c2[[0,1,2]][:,[0,1,2]]
        # print mat(kc2)
        kd1=self.vector_quaternion(d1[[0,1,2]][:,[0,1,2]])[1:]
        # print kd1
        kd2=self.vector_quaternion(d2[[0,1,2]][:,[0,1,2]])[1:]
        # print kd2
        kc3=np.cross(mat(kc1),mat(kc2))
        # print "kc3",kc3
        kd3=np.cross(mat(kd1),mat(kd2))
        # print "kd3",kd3
        #a=[kc1 kc2 kc3]
        #b=[kd1 kd2 kd3
        #R=a*inv(b)
        aa=np.hstack((mat(kc1).T,mat(kc2).T))
        a=np.hstack((aa,mat(kc3).T))
        #print a
        bb=np.hstack((mat(kd1).T,mat(kd2).T))
        b=np.hstack((bb,mat(kd3).T))
        R=a*(b.I)
        # print R
        tc1=c1[[0,1,2]][:,[3]]
        # print "tc1",tc1
        tc2=c2[[0,1,2]][:,[3]]
        # print "tc2",tc2
        td1=d1[[0,1,2]][:,[3]]
        # print "td1",td1
        td2=d2[[0,1,2]][:,[3]]
        # print "td2",td2
        c=R*td1-tc1
        # print "c---",c
        d=R*td2-tc2
        # print "d---",d
        a=rc1-np.eye(3)
        b=rc2-np.eye(3)
        # print "-----------------a---------------\n",a
        # print "-----------------b---------------\n",b
        h=np.vstack((a,b))
        # print h
        y=np.vstack((c,d))
        # print y
        hh=h.I*h
        t=hh.I*(h.I)*y*0.1
        # print "t-----",t
        xx=np.hstack((R,t))
        # print xx
        eye4x1=[0,0,0,1]
        # print mat(eye4x1)
        x=np.row_stack((xx,mat(eye4x1)))
        # print "------------------x-------------"
        # print x
        return x

    """
    ar_info is list(1*7), 1*3 is position; 4*7 is rotation 
    """
def get_X_from_ar_quatenion_tf():
    quaternion = (
        -0.0707213412369,
        0.995578841751,
        0.995578841751,
        0.0617148086872)

    print "quaternion------\n",tf.transformations.quaternion_matrix(quaternion)
def get_X_from_arquaternion_tf(ar_info):
    transition_L = np.array(ar_info[:3]).T
    T_part1_1=np.zeros((3,3))
    T_part1=np.column_stack((T_part1_1, transition_L))
    T_part2 = np.array([0, 0, 0, 0])
    TL = np.row_stack((T_part1, T_part2))
    quaternion = (ar_info[3],ar_info[4],ar_info[5],ar_info[6])
    T=tf.transformations.quaternion_matrix(quaternion)
    T=T+TL
    T=T.tolist()
    T=T[0]+T[1]+T[2]+T[3]
    print "new_tf",T
    return T
def get_X_from_ar_quaternion(ar_info):
        transition_L = np.array(ar_info[:3]).T
        # print transition_L
        rot = ar_info[3:6]
        s = ar_info[6]
        q0 = Q.quaternion(s, np.mat(rot))
        print "q02R--------\n",q0.r()
        # print q0.r()
        T_part1 = np.column_stack((q0.r(), transition_L))
        # print T_part1
        T_part2 = np.array([0, 0, 0, 1])
        # print T_part2
        T = np.row_stack((T_part1, T_part2))
        # print T
        T = T.tolist()
        T = T[0] + T[1] + T[2] + T[3]
        # print("T:" , T)
        return T
# def get_T_from_ar_and_ur(ar_info, ur_info):
#
#     return
def get_ur1_X():
    aa=np.mat([[
        -0.5694 ,   0.5107  ,  0.6442, - 0.01551,- 0.0329, - 0.7972  ,  0.6029, - 0.02527,0.8214  ,  0.3221  ,  0.4707   , 0.08654,0  ,       0    ,     0  ,  1.0000
    ]])
    aa=aa.reshape((4,4))
    # X = aa # numpu array
    # # tr2jac
    # jac = tr2jac(X, 1)
    # # print "------X",X
    # inv_X_jac = jac.I
    # print "inv_X_jac",inv_X_jac
    return aa
def get_ur_X():
    ar_info = [
    #20190117
        # translation:
    0.694264970012,
    0.498084973159,
    0.604353727679,

    0.766461091073,
    -0.185872336174,
    0.148997910405,
    0.596480086182

    ]
    aa=get_X_from_ar_quaternion(ar_info)
    aa=np.mat(aa)
    print "X",aa
    return aa.reshape((4, 4))
def get_ur2_X():
    ur5 = UR_robot()
    ar_info=[
    0.104110973563,
    0.0933282967429,
    0.0701830055673,
    -0.636905500965,
    0.270217810417,
    -0.630922242744,
    0.351099475058
    ]
    # ur5_info1=[54.63,-72.12,60.38,-76.25,-91.56,44.13]
    aa=get_X_from_ar_quaternion(ar_info)
    aa=np.mat(aa)
    # q0 = getpi_for_py(getpi(ur5_info1))
    # c = Kinematic(q0)
    # T0=[0.6320,0.7743,-0.0319,0.26329,-0.7716,0.6248,-0.1191,0.55700,-0.0723,0.0999,0.9924,0.48785,0.0,0.0,0.0,1.0000]
    #q_guess=
    #c.best_sol([1.] * 6,q0,T0)

    # #ur5.get_ik_pose(T0)
    # for i in range(len(c.get_ik_data(T0))):
    #
    #     kk=c.get_ik_data(T0)[i]
    #     print kk
   # print "best sol",c.best_sol([1.] * 6,q0,T0)
    # ur5.set_q( q0 )
    #
    # T0 = ur5.get_fk_pose()
    #print "T0",np.mat(T0).reshape((4,4))
    print aa.reshape((4,4))
    return aa.reshape((4,4))
def get_ur1_X():
    ur3 = UR_robot()

    ar_info0 = [
        0.0489604924604,
        0.104115324241,
        1.36773691564,
        0.00663474651814,
        0.989165861516,
        -0.12433993602,
        0.0777589792951
    ]
    # ar_info0=[
    # 0.0550051796881,
    # 0.0725637253379,
    # 0.651945125388,
    # 0.0159956997681,
    # 0.965095771746,
    # -0.028897097311,
    # -0.259806171432
    # ]
    ur3_info0 = [218.45, -12.7, -85.80, 2.34, -264.79, 283.09]
    #ur3_info0 = [14.92,-94.24,80.90,-71.14,-61.70,18.68]

    Te0 = get_X_from_arquaternion_tf( ar_info0)
    print "Te0", Te0
    q0 = getpi_for_py(getpi(ur3_info0))
    ur3.set_q( q0 )
    T0 = ur3.get_fk_pose()
    print "T0",T0
    ####  cal  te1 T1
    ar_info1 = [
        0.189481829976,
        -0.0260899667402,
        1.23753029881,
        -0.0773089645275,
        0.995300619959,
        -0.0581770229608,
        0.00392860185176

    ]
    # ar_info1=[
    #     0.163091515966,
    # -0.0150207279728,
    # 0.461865461716,
    # -0.222940333555,
    # 0.973792669258,
    # 0.0153585923978,
    # 0.0423031749907
    # ]
    # ur3_info1 = [70.93,-85.48,87.40,-91.78,-94.84,48.97]
    ur3_info1 = [207.13, -19.98, -85.50, 2.34, -264.79,283.09]
    Te1 = get_X_from_arquaternion_tf(ar_info1)
    print "Te1",Te1
    q1 = getpi_for_py(getpi(ur3_info1))
    # print("q1:", q1)
    ur3.set_q(q1)
    T1 = ur3.get_fk_pose()
    print("T1:", T1)

    #####  cal te2 T2
    ar_info2 = [
        -0.0585841595815,
        -0.218299099394,
        1.27599182339,
        -0.0707213412369,
        0.995578841751,
        0.995578841751,
        0.0617148086872
    ]
    ur3_info2 = [207.13, -14.71, -93.55, 4.22, -281.01, 283.09]
    # ar_info2 = [
    #     -0.0150297473928,
    # 0.0337093579007,
    # 0.585400483887,
    # 0.108905890232,
    # 0.981830000534,
    # 0.0380729591274,
    # -0.150664550929
    #
    # ]
    # ur3_info2 = [33.04,-89.10,77.28,-77.10,-75.05,48.93]
    Te2 = get_X_from_arquaternion_tf(ar_info2)

    q2 = getpi_for_py(getpi(ur3_info2))
    ur3.set_q(q2)
    T2 = ur3.get_fk_pose()

    c0 = HandInEye(T0, Te0, T1, Te1, T2, Te2)
    c0.caculat_C_D(Te1, Te0, T0, T1)
    # print "--------------------------from 0 to 1------------------------------------"
    c1, d1 = c0.from0to1(Te1, Te0, T0, T1)
    # print "--------------------------from 1 to 2------------------------------------"
    c2, d2 = c0.from1to2(Te2, Te1, T1, T2)
    # print c0.vector_quaternion(c2[[0, 1, 2]][:, [0, 1, 2]])
    # print "-------------------------final-X----------------------------------------"
    X = c0.final_X()
    print "X:\n", X
    return X
def get_u1r_X():
    ur3 = UR_robot()

    ar_info0 = [
        0.0489604924604,
        0.104115324241,
        1.36773691564,
        0.00663474651814,
        0.989165861516,
        -0.12433993602,
        0.0777589792951
    ]
    # ar_info0=[
    # 0.0550051796881,
    # 0.0725637253379,
    # 0.651945125388,
    # 0.0159956997681,
    # 0.965095771746,
    # -0.028897097311,
    # -0.259806171432
    # ]
    ur3_info0 = [218.45, -12.7, -85.80, 2.34, -264.79, 283.09]
    #ur3_info0 = [14.92,-94.24,80.90,-71.14,-61.70,18.68]

    Te0 = get_X_from_ar_quaternion( ar_info0)
    print "Te0", Te0
    q0 = getpi_for_py(getpi(ur3_info0))
    ur3.set_q( q0 )
    T0 = ur3.get_fk_pose()
    print "T0",T0
    ####  cal  te1 T1
    ar_info1 = [
        0.189481829976,
        -0.0260899667402,
        1.23753029881,
        -0.0773089645275,
        0.995300619959,
        -0.0581770229608,
        0.00392860185176

    ]
    # ar_info1=[
    #     0.163091515966,
    # -0.0150207279728,
    # 0.461865461716,
    # -0.222940333555,
    # 0.973792669258,
    # 0.0153585923978,
    # 0.0423031749907
    # ]
    # ur3_info1 = [70.93,-85.48,87.40,-91.78,-94.84,48.97]
    ur3_info1 = [207.13, -19.98, -85.50, 2.34, -264.79,283.09]
    Te1 = get_X_from_ar_quaternion(ar_info1)
    print "Te1",Te1
    q1 = getpi_for_py(getpi(ur3_info1))
    # print("q1:", q1)
    ur3.set_q(q1)
    T1 = ur3.get_fk_pose()
    print("T1:", T1)

    #####  cal te2 T2
    ar_info2 = [
        -0.0585841595815,
        -0.218299099394,
        1.27599182339,
        -0.0707213412369,
        0.995578841751,
        0.995578841751,
        0.0617148086872
    ]
    ur3_info2 = [207.13, -14.71, -93.55, 4.22, -281.01, 283.09]
    # ar_info2 = [
    #     -0.0150297473928,
    # 0.0337093579007,
    # 0.585400483887,
    # 0.108905890232,
    # 0.981830000534,
    # 0.0380729591274,
    # -0.150664550929
    #
    # ]
    # ur3_info2 = [33.04,-89.10,77.28,-77.10,-75.05,48.93]
    Te2 = get_X_from_ar_quaternion(ar_info2)

    q2 = getpi_for_py(getpi(ur3_info2))
    ur3.set_q(q2)
    T2 = ur3.get_fk_pose()

    c0 = HandInEye(T0, Te0, T1, Te1, T2, Te2)
    c0.caculat_C_D(Te1, Te0, T0, T1)
    # print "--------------------------from 0 to 1------------------------------------"
    c1, d1 = c0.from0to1(Te1, Te0, T0, T1)
    # print "--------------------------from 1 to 2------------------------------------"
    c2, d2 = c0.from1to2(Te2, Te1, T1, T2)
    # print c0.vector_quaternion(c2[[0, 1, 2]][:, [0, 1, 2]])
    # print "-------------------------final-X----------------------------------------"
    X = c0.final_X()
    print "X:\n", X
    return X

def main():
    Te1  = [ 0.218178,0.975410,-0.031214,-63.060175,0.967780,-0.220369,-0.121813,-21.839437,-0.125696,-0.003632,-0.992062,500.823439,0,0,0,1]
    Te0  = [ 0.128482,0.991024,-0.036917,-15.564192,0.991056,-0.129661,-0.031550,-31.592337,-0.036054,-0.032533,-0.998820,447.897520,0,0,0,1]
    T0   = [0.4696,0.8820,0.0399,190.184,0.8788,-0.4713,0.0747,355.823,0.0847,0.0000,-0.9964,-141.89,0,0,0,1]
    T1   = [0.5429,0.8341,0.0981,214.629,0.8208,-0.5516,0.1483,324.508,0.1778,0.0000,-0.9841,-101.991,0,0,0,1]
    #Bc2  pose 2右图像外参数   Ac2 1右图像外参数
    Te2 =[ 0.070836,0.996832,-0.036178,14.913514,0.975269,-0.076825,-0.207238,-27.935731,-0.209361,-0.020603,-0.977621,544.266948,0,0,0,1]
    T2=  [0.4100,0.9053,0.1106,153.909,0.8741,-0.4247,0.2358,328.212,0.2605,0.0000,-0.9655,-70.151,0,0,0,1]
    c0=HandInEye(T0,Te0,T1,Te1,T2,Te2)
    c0.caculat_C_D(Te1,Te0,T0,T1)
    print "--------------------------from 0 to 1------------------------------------"
    c1,d1=c0.from0to1(Te1,Te0,T0,T1)
    print "--------------------------from 1 to 2------------------------------------"
    c2,d2=c0.from1to2(Te2,Te1,T1,T2)
    print c0.vector_quaternion(c2[[0,1,2]][:,[0,1,2]])
    print "-------------------------final-X----------------------------------------"
    print c0.final_X()
if __name__=="__main__":
    #main()
    print get_ur_X()
    # get_ur2_X()
    # ar_info0 = [
    #     -0.0585841595815,
    #     -0.218299099394,
    #     1.27599182339,
    #     -0.0707213412369,
    #     0.995578841751,
    #     0.995578841751,
    #     0.0617148086872
    # ]
    # get_X_from_arquaternion_tf(ar_info0)
   # get_X_from_ar_quatenion_tf()
