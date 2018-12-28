#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
from numpy import *
import numpy as np
import yaml
import Quaternion as Q
import math
import os,sys
from ur_kinematics import *
class Fixedparameters():
    def __init__(self,cali_filename,fixed_filename):
        self.cali_filename = cali_filename
        self.fixed_filename = fixed_filename

    """
    Get data from cam_500_logitech.yaml
    """
    def Get_instrinc_parameters(self):
        f=open(self.cali_filename)
        yamldata=yaml.load(f)
        #print yamldata
        kx =  yamldata['camera_matrix']['data'][0]
        #print kx
        ky = yamldata['camera_matrix']['data'][4]
        #print ky
        u0=yamldata['camera_matrix']['data'][2]
        v0 = yamldata['camera_matrix']['data'][5]
        ins_matrix=yamldata['camera_matrix']['data']
        focal_length=yamldata['focallength']
        cam = {'kx': kx, 'ky': ky, "u0": u0, "v0": v0,"ins_matrix":ins_matrix,"focal_length":focal_length}
        f.close()
        return cam

    def caculat_C_D(self, xTe1, xTe0, xT0, xT1):
        # Bc1 pos1 extrinsic parameters
        # Ac1 pos0 extrinsic parameters
        # Ad1 pos0 T
        # Bd1 pose1 T
        Bc1 = xTe1
        Ac1 = xTe0
        Ad1 = xT0
        Bd1 = xT1
        mBc1 = np.mat(Bc1)
        mAc1 = np.mat(Ac1)
        invmBc1 = np.reshape(mBc1, (4, 4)).I
        mAc1to4x4 = np.reshape(mAc1, (4, 4))
        C1 = mAc1to4x4 * invmBc1
        # print "C1----->\n",C1
        mAd1 = np.mat(Ad1)
        mBd1 = np.mat(Bd1)
        d1 = (np.reshape(mAd1, (4, 4)).I) * (np.reshape(mBd1, (4, 4)))
        # print "d1------->\n",d1
        return C1, d1

    def from0to1(self, Te1, Te0, T0, T1):
        return self.caculat_C_D(Te1, Te0, T0, T1)

    def from1to2(self, Te2, Te1, T1, T2):
        return self.caculat_C_D(Te2, Te1, T1, T2)

    def vector_quaternion(self, R):
        result = []
        q = math.acos((np.trace(R) - 1) / 2)
        # print "q:",q
        q1 = (q / (2 * math.sin(q))) * (R[2, 1] - R[1, 2])
        # print "q1:",q1
        q2 = (q / (2 * math.sin(q))) * (R[0, 2] - R[2, 0])
        # print "q2:",q2
        q3 = (q / (2 * math.sin(q))) * (R[1, 0] - R[0, 1])
        # print "q3:",q3
        result.append(q)
        result.append(q1)
        result.append(q2)
        result.append(q3)
        return result

    # AX=XB
    # R=[kc1,kc2,kc3]*inv[kd1,kd2,kd3]
    def final_X(self,Te0,Te1,Te2,T0,T1,T2):
        c1, d1 = self.from0to1(Te1, Te0, T0, T1)
        c2, d2 = self.from1to2(Te2, Te1, T1, T2)
        rc1 = c1[[0, 1, 2]][:, [0, 1, 2]]
        kc1 = self.vector_quaternion(c1[[0, 1, 2]][:, [0, 1, 2]])[1:]
        # print mat(kc1)
        kc2 = self.vector_quaternion(c2[[0, 1, 2]][:, [0, 1, 2]])[1:]
        rc2 = c2[[0, 1, 2]][:, [0, 1, 2]]
        # print mat(kc2)
        kd1 = self.vector_quaternion(d1[[0, 1, 2]][:, [0, 1, 2]])[1:]
        # print kd1
        kd2 = self.vector_quaternion(d2[[0, 1, 2]][:, [0, 1, 2]])[1:]
        # print kd2
        kc3 = np.cross(mat(kc1), mat(kc2))
        # print "kc3",kc3
        kd3 = np.cross(mat(kd1), mat(kd2))
        # print "kd3",kd3
        # a=[kc1 kc2 kc3]
        # b=[kd1 kd2 kd3
        # R=a*inv(b)
        aa = np.hstack((mat(kc1).T, mat(kc2).T))
        a = np.hstack((aa, mat(kc3).T))
        # print a
        bb = np.hstack((mat(kd1).T, mat(kd2).T))
        b = np.hstack((bb, mat(kd3).T))
        R = a * (b.I)
        # print R
        tc1 = c1[[0, 1, 2]][:, [3]]
        # print "tc1",tc1
        tc2 = c2[[0, 1, 2]][:, [3]]
        # print "tc2",tc2
        td1 = d1[[0, 1, 2]][:, [3]]
        # print "td1",td1
        td2 = d2[[0, 1, 2]][:, [3]]
        # print "td2",td2
        c = R * td1 - tc1
        # print "c---",c
        d = R * td2 - tc2
        # print "d---",d
        a = rc1 - np.eye(3)
        b = rc2 - np.eye(3)
        # print "-----------------a---------------\n",a
        # print "-----------------b---------------\n",b
        h = np.vstack((a, b))
        # print h
        y = np.vstack((c, d))
        # print y
        hh = h.I * h
        t = hh.I * (h.I) * y
        # print "t-----",t
        xx = np.hstack((R, t))
        # print xx
        eye4x1 = [0, 0, 0, 1]
        # print mat(eye4x1)
        x = np.row_stack((xx, mat(eye4x1)))
        # print "------------------x-------------"
        # print x
        return x

    """
    ar_info is list(1*7), 1*3 is position; 4*7 is rotation 
    """

    def get_X_from_ar_quaternion(self,ar_info):
        transition_L = np.array(ar_info[:3]).T
        # print transition_L
        rot = ar_info[3:6]
        s = ar_info[6]
        q0 = Q.quaternion(s, np.mat(rot))

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

    def getpi(self,listb):
        lista = []
        for i in listb:
            temp = i / 180 * 3.14
            lista.append(temp)
        return lista
    def get_cali_info(self):
        reslut={}
        ur_ki = Kinematic()
        f = open(self.fixed_filename)
        yamldata = yaml.load(f)
        ar_info0 = yamldata["ur3_ar_info0"]
        ur_info0 = yamldata["ur3_info0"]
        Te0 = self.get_X_from_ar_quaternion(ar_info0)
        print "Te0----\n", Te0

        q0 = self.getpi(ur_info0)
        T0 = ur_ki.Forward(q0)
        print "T0-----\n", T0


        ####  cal  te1 T1
        ar_info1 = yamldata["ur3_ar_info1"]
        ur_info1 = yamldata["ur3_info1"]
        Te1 = self.get_X_from_ar_quaternion(ar_info1)
        print "Te1----\n", Te1
        q1 = self.getpi(ur_info1)
        T1 = ur_ki.Forward(q1)
        print "T1----\n", T1
        # print("T1:", T1)

        #####  cal te2 T2
        ar_info2 = yamldata["ur3_ar_info2"]
        ur_info2 = yamldata["ur3_info2"]

        Te2 = self.get_X_from_ar_quaternion(ar_info2)
        print "Te2----\n", Te2
        q2 = self.getpi(ur_info2)
        T2 = ur_ki.Forward(q2)
        print "T2----\n", T2
        f.close()
        return {"Te0":Te0,"T0":T0,"Te1":Te1,"T1":T1,"Te2":Te2,"T2":T2}

def get_ur_X():
    path=os.path.realpath(__file__)[0]
    sys.path.insert(0, path)
    cali_filename="../yaml/cam_500_logitech.yaml"
    fixed_filename="../yaml/fix_settings.yaml"
    cali_info={}
    cam_par={}
    result_dict={}
    c0 = Fixedparameters(cali_filename,fixed_filename)
    cali_info=c0.get_cali_info()
    c0.caculat_C_D(cali_info["Te1"], cali_info["Te0"], cali_info["T0"], cali_info["T1"])
    # print "--------------------------from 0 to 1------------------------------------"
    c1, d1 = c0.from0to1(cali_info["Te1"], cali_info["Te0"], cali_info["T0"], cali_info["T1"])
    # print "--------------------------from 1 to 2------------------------------------"
    c2, d2 = c0.from1to2(cali_info["Te2"], cali_info["Te1"], cali_info["T1"], cali_info["T2"])
    c0.vector_quaternion(c2[[0, 1, 2]][:, [0, 1, 2]])
    # print "-------------------------final-X----------------------------------------"
    X = c0.final_X(cali_info["Te0"], cali_info["Te1"], cali_info["Te2"], cali_info["T0"],cali_info["T1"], cali_info["T2"])
    print  "x", X
    cam_par=c0.Get_instrinc_parameters()
    cali_info={"X":X.tolist()}
    result_dict=dict(cam_par, **cali_info)
    print result_dict
    fi=open("../yaml/fixed_cam_param.yaml","w+")
    yaml.dump(result_dict,fi)
    fi.close()
def main():
    get_ur_X()
if __name__=="__main__":
    main()