#! /usr/bin/env python
import src.universal_robot.ur5_planning.scripts.frompitoangle
import numpy
from numpy import matlib,linalg
#Import the module
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from pykdl_utils.kdl_kinematics import KDLKinematics
from src.universal_robot.ur5_planning.scripts.new_kinematics import Kinematic
#import cv2
import yaml,os
class VisonControl():
    def __init__(self,califilename,sim,lambda1,urdfname):
        self.califilename=califilename
        self.sim=sim
        self.lambda1=lambda1
        self.urdfname=urdfname
    #if flag=1,use our kinematics for inverse
    def get_jacabian_from_joint(self,urdfname,jointq,flag):
        #robot = URDF.from_xml_file("/data/ros/ur_ws/src/universal_robot/ur_description/urdf/ur5.urdf")
        robot = URDF.from_xml_file(urdfname)
        tree = kdl_tree_from_urdf_model(robot)
        print tree.getNrOfSegments()

        chain = tree.getChain("base_link", "ee_link")
        print chain.getNrOfJoints()

        # forwawrd kinematics
        kdl_kin = KDLKinematics(robot, "base_link", "ee_link")
        q=jointq
        #q = [0, 0, 1, 0, 1, 0]
        pose = kdl_kin.forward(q)  # forward kinematics (returns homogeneous 4x4 matrix)
        print pose
        #print list(pose)
        q0=Kinematic(q)
        if flag==1:
            q_ik=q0.best_sol_for_other_py( [1.] * 6, 0, q0.Forward())
        else:
            q_ik = kdl_kin.inverse(pose)  # inverse kinematics
        print "----------iverse-------------------\n", q_ik

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
            cam = {'kx': kx, 'ky': ky, "u0": u0, "v0": v0}
            return cam
        f=open(self.filename)
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
    #uv desired position list
    def get_center_imagepos(self):
        pass
    def ouler2pose(self):
        pass
    def pose2image(self):
        pass
    #cal image jacbian
    def vis2jac(self,uv,z):
        cam=self.get_cam_data()
        #uv = [[672, 672], [632, 662]]
        x=(uv[0]-cam['u0'])*(1.0/cam['kx'])
        #print x
        y=(uv[1]-cam['v0'])*(1.0/cam['ky'])
        #print y
        L=[[1.0/z,0,-x/z,-x*y,(1.0+x**2),-y],[0,1.0/z,-y/z,-(1.0+y**2),x*y,x]]
        Lm=numpy.array(L).reshape((2,6))
        #print Lm
        J=-1.0*numpy.dot(numpy.diag([cam['kx'],cam['ky']]),Lm)
        #print numpy.shape(J)
        #print J
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
    def get_uv_from_ar(self):
        pass
    def get_feature_error(self,desireuv,nowuv):
        kk=numpy.mat(nowuv).T-numpy.mat(desireuv).T
        return kk.reshape((1,8))
    #cam speed (udot,vdot)(xdot,ydot,zdot,wxdot,wydot,wzdot)
    def get_cam_vdot(self,uvm,z,desireuv,nowuv):
        J=self.vis2jac_mt1(uvm,z)
        JJ=numpy.linalg.pinv(J)
        e=self.get_feature_error(desireuv,nowuv)
        vdot=self.lambda1*numpy.dot(JJ,e.T)
        return vdot
    #samebody tranlasition to jacbian
    #joint speed (q0dot,q1dot,q2dot,q3dot,q4dot,q5dot)
    def get_joint_speed(self,uvm,z,desireuv,nowuv,q):
        Jacabian_joint=self.get_jacabian_from_joint(self.urdfname,q,0)
        cam_speed=self.get_cam_vdot(uvm,z,desireuv,nowuv)
        j_speed=numpy.dot(Jacabian_joint.I,cam_speed)
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
    filename="/data/ros/ur_ws/src/universal_robot/ur5_vision/camera_sg.yaml"
    urdfname="/data/ros/ur_ws/src/universal_robot/ur_description/urdf/ur5.urdf"
    startuv=[[512,412],[312,412],[312,612],[512,612]]
    desiruv=[[612,412],[412,412],[412,612],[612,612]]
    #q_angular=[0,0,1,0,1,0]
    #q_angular=[194.82,-78.91,-111.49,8.09,-283.35,324.67]
    #qpi=frompitoangle.getpi_for_py(frompitoangle.getpi(q_angular))
    #print "q_pi",qpi
    qpi=[0,0,1,0,1,0]
    #qpi=[0,0,1,0,1,0]
    lambda1=1
    p0=VisonControl(filename,1,lambda1,urdfname)
    #p0.get_cam_data()
    #p0.vis2jac(uv[0],5,1,filename)
    #p0.vis2jac_mt1(startuv,5)
   # print "feature\n",p0.get_feature_error(desiruv,startuv)
    #print p0.get_cam_vdot(startuv,5,desiruv,startuv)
    #print p0.get_cam_vdot(startuv,5,desiruv,startuv)
    print "----------------joint speed\n",p0.get_joint_speed(startuv,1,desiruv,startuv,qpi)
    print "----------------deta joint angular\n",p0.get_deta_joint_angular(0.05,startuv,5,desiruv,startuv,qpi)
    print "----------------joint angular\n",p0.get_joint_angular(qpi,p0.get_deta_joint_angular(0.05,startuv,5,desiruv,startuv,qpi))
if __name__=="__main__":
    main()