#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from ur5_pose_get import *
from ur5_kinematics import *
import numpy
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from pykdl_utils.kdl_kinematics import KDLKinematics
import yaml
import time
rospy.init_node("move_ur5_by_urscript")
pub=rospy.Publisher("/ur_driver/URScript",String,queue_size=10)
ur_reader = Urposition()
ur_sub = rospy.Subscriber("/joint_states", JointState, ur_reader.callback)
rate=rospy.Rate(1)
def get_ur_pose():
    joint_angular = ur_reader.ave_ur_pose
    return joint_angular
def change_angle_to_pi(qangle):
    temp=[]
    for i in xrange(len(qangle)):
        temp.append(qangle[i]/180.0*3.14)
    return temp
def moveur(pub,q,ace,vel,t):
    ss="movej(["+str(q[0])+","+str(q[1])+","+str(q[2])+","+str(q[3])+","+str(q[4])+","+str(q[5])+"]," +"a="+str(ace)+","+"v="+str(vel)+","+"t="+str(t)+")"
    print ss
    pub.publish(ss)
def get_jacabian_from_joint(urdfname,jointq):
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
    print pose
    print list(pose)

    q_ik = kdl_kin.inverse(pose)  # inverse kinematics
    print "----------inverse-------------------\n", q_ik

    if q_ik is not None:
        pose_sol = kdl_kin.forward(q_ik)  # should equal pose
        print "------------------forward ------------------\n",pose_sol

    # J = kdl_kin.jacobian(q)
    #print 'J:', J
    return pose,q_ik
t=0
vel=0.1
ace=50
z=0.51#m
weights = [1.] * 6
urdfname = "/data/ros/ur_ws/src/universal_robot/ur5_planning/urdf/ur5.urdf"
while not rospy.is_shutdown():

    # qq=[
    #     # [307.61, -136.58, 65.96, -120.66, -306.25, -215.74]
    #     [182.22,-141.28,108.32,-137.17,-180.82,-218.81]
    #     # [266.09, -178.29, 117.69, -121.76, -267.38, -216.20]
    #     #202.22,-199.82
    #
    #     # [86.02,-140.65,112.10,-148.28,-88,-320.66]
    #     # [35.78,-105.15,87.76,-235.20,-90.99,195.06]
    #     # [93.93,-145.95,100.84,-121.30,-91.04,-326.16]
    #     # [34.26,-99.78,81.03,-239.41,-94.51,-184.10]
    #     # [75.59,-0.51,-88.94,-188.57,-75.17,148.67]
    #     # [267,-46,-127.23,-94.8,-79,-132.34]
    #     # [93,122,-75,-81,-110,-37],
    #     # [90, -62, -104, -105, -110, -34]
    #     # [42.27,-105.20,87.8,-212.62,-86.5,195.16]
    #     # [65.38,-108.78,94.23,-254.23,-75.22,166.93]
    #     ]
    # for ii in xrange(len(qq)):
    #     qt=change_angle_to_pi(qq[ii])
    #     # time.sleep(1)
    #     moveur(pub, qt,ace,vel,t)
    #     # time.sleep(1)
    k0=Kinematic()
    angular_joint=get_ur_pose()
    if len(angular_joint)!=0:

        print "joint angular",angular_joint
        # print "Forward",numpy.matrix(k0.Forward(angular_joint)).reshape(4,4)
        # T=k0.Forward(angular_joint)
        # T_new=[]
        # for i in xrange(len(T)):
        #     if i ==7:
        #         temp=T[i] + (-0.50)
        #         T_new.append(temp)
        #     else:
        #         T_new.append(T[i])
        # print "T_new",numpy.matrix(T_new).reshape(4,4)
        # q_invers_sol=k0.best_sol(weights,angular_joint,T_new)
        # print "new_pub\n",q_invers_sol[0].tolist()
        # print "sols\n",q_invers_sol[1]
        # for i in q_invers_sol[1]:
        #     moveur(pub, i, ace, vel, t)
        #     time.sleep(25)
        pose, q_ik=get_jacabian_from_joint(urdfname, angular_joint)

    rate.sleep()
