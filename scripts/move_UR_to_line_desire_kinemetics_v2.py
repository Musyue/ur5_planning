#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy
from std_msgs.msg import String,Float64
from frompitoangle import *
from ur5_kinematics import *
from ur5_pose_get import *
from transfer import *
from trans_methods import *
class UrLineCircle:
    def __init__(self,weights,radius):
        self.weights = weights
        self.radius=radius#m
        self.cont=400
        self.theta=-math.pi / 4
        # rotating 45 degree with Z aisx
        self.R0b = [math.cos(self.theta), -1*math.sin(self.theta), 0, math.sin(self.theta), math.cos(self.theta), 0, 0, 0,1]
    def Init_node(self):
        rospy.init_node("move_ur5_circle")
        pub = rospy.Publisher("/ur_driver/URScript", String, queue_size=10)
        return pub
    def get_urobject_ur5kinetmatics(self):
        ur0 = Kinematic()
        return ur0
    def get_draw_circle_xy(self,t,xy_center_pos):
        x = xy_center_pos[0] + self.radius * math.cos( 2 * math.pi * t / self.cont )
        y = xy_center_pos[1] + self.radius * math.sin( 2 * math.pi * t / self.cont)
        return  [x,y]
    def get_draw_line_x(self,t,transxyz0,transxyz_d):#transxyz[x,y,z]
        xn_1=t*(transxyz_d[0]-transxyz0[0])/self.cont
        return xn_1
    def get_draw_line_45(self,t,transxyz0,transxyz_d):
        pass
    def get_IK_from_T(self,T,q_last):
        ur0 = self.get_urobject_ur5kinetmatics()
        return ur0.best_sol(self.weights,q_last,T)
    def get_q_list(self,T_list,qzero):
        ur0 = self.get_urobject_ur5kinetmatics()
        tempq=[]
        resultq=[]
        for i in xrange(self.cont):
            if i==0:
                tempq=qzero
                firstq = ur0.best_sol(self.weights, tempq, T_list[i])
                tempq=firstq
                resultq.append(firstq.tolist())
                print "firstq", firstq
            else:
                qq = ur0.best_sol(self.weights, tempq, T_list[i])
                tempq=qq
                print "num i qq",i,qq
                resultq.append(tempq.tolist())
        return resultq
    # T is numpy.array
    def get_T_translation(self, T):
        trans_x = T[3]
        trans_y = T[7]
        trans_z = T[11]
        return [trans_x, trans_y, trans_z]
    def insert_new_xy(self,T,nx,ny,nz):
        temp=[]
        for i in xrange(12):
            if i==3:
                temp.append(nx)
            elif i==7:
                temp.append(ny)
            elif i == 11:
                temp.append(nz)
            else:
                temp.append(T[i])
        return temp
    def numpyarray_tolist(self,T):
        tt=T.tolist()
        temp=[]
        for i in range(4):
            for j in range(4):
                temp.append(tt[i][j])
        return temp
    def insert_new_x_y_z(self,T,nx,ny,nz):#Rob*R06
        temp=[]
        for i in xrange(16):
            if i==3:
                temp.append(nx)
            elif i==7:
                temp.append(ny)
            elif i==11:
                temp.append(nz)
            else:
                temp.append(T[i])
        temp_rotation=tr2r(numpy.array(temp).reshape((4,4)))
        new_rotation=numpy.dot(numpy.array(self.R0b).reshape((3,3)),temp_rotation)
        trans_part2=numpy.array([nx,ny,nz]).T
        homegeneous_T_part3=numpy.array([0,0,0,1])
        new_T_T = np.column_stack((new_rotation, trans_part2))
        new_T =np.row_stack((new_T_T, homegeneous_T_part3))
        print "new_T",new_T
        last_T=self.numpyarray_tolist(new_T)
        print "last_T",last_T
        return last_T
    def get_new_T(self,InitiT,xy_center_pos):
        temp=[]
        for i in xrange(self.cont):
            new_xy=self.get_draw_circle_xy(i,xy_center_pos)
            new_T=self.insert_new_xy(InitiT,new_xy[0],new_xy[1])
            #print "initial T\n",InitiT
            #print "new_T\n",i,new_T
            temp.append(new_T)
        return temp
    def urscript_pub(self, pub, qq, vel, ace, t):

        ss = "movej([" + str(qq[0]) + "," + str(qq[1]) + "," + str(qq[2]) + "," + str(
            qq[3]) + "," + str(qq[4]) + "," + str(qq[5]) + "]," + "a=" + str(ace) + "," + "v=" + str(
            vel) + "," + "t=" + str(t) + ")"
        print("---------ss:", ss)
            # ss="movej([-0.09577000000000001, -1.7111255555555556, 0.7485411111111111, 0.9948566666666667, 1.330836666666667, 2.3684322222222223], a=1.0, v=1.0,t=5)"
        pub.publish(ss)
def main():
    t=0
    vel=0.1
    ace=50
    # vel=1.05
    # ace=1.4
    transxyz_d=[0.6,0,0]
    qstart=[-46.72747516599795, -175.15460415232326, 80.15343104225853, -186.23553423153677, -90.25757394658287, -24.25125691705894]
    # qq=[45.91,-72.37,61.52,-78.56,-90.49,33.71]
    # q=display(getpi(qq))
    ratet = 30
    radius=0.1
    weights = [1.] * 6
    T_list=[]
    urc=UrLineCircle(weights,radius)
    pub=urc.Init_node()
    rate = rospy.Rate(ratet)

    # first step go to initial pos
    qzero = display(getpi(qstart))
    # urc.urscript_pub(pub,q,vel,ace,t)
    # second get T use urkinematics
    urk = urc.get_urobject_ur5kinetmatics()
    F_T = urk.Forward(qzero)

    print "F_T", F_T
    TransT = urc.get_T_translation(F_T)
    print "TransT", TransT

    # xy_center_pos = [TransT[0], TransT[1]]
    # print "xy_center_pos", xy_center_pos
    # T_list = urc.get_new_T(F_T, xy_center_pos)
    # print "T_list", T_list
    # reslut_q = urc.get_q_list(T_list, q)
    # print "reslut_q", reslut_ur0=
    uree_d_x_pub = rospy.Publisher("/uree_d_x", Float64, queue_size=10)
    uree_d_y_pub = rospy.Publisher("/uree_d_y", Float64, queue_size=10)
    uree_d_z_pub = rospy.Publisher("/uree_d_z", Float64, queue_size=10)
    uree_n_x_pub = rospy.Publisher("/uree_n_x", Float64, queue_size=10)
    uree_n_y_pub = rospy.Publisher("/uree_n_y", Float64, queue_size=10)
    uree_n_z_pub = rospy.Publisher("/uree_n_z", Float64, queue_size=10)
    ur0=urc.get_urobject_ur5kinetmatics()
    cn=0
    ZeroT = F_T
    tempq = qzero
    while not rospy.is_shutdown():

        x_new=urc.get_draw_line_x(cn,urc.get_T_translation(ZeroT),transxyz_d)
        uree_d_x_pub.publish(x_new)
        uree_d_y_pub.publish(TransT[1])
        uree_d_z_pub.publish(TransT[2])

        new_T=urc.insert_new_x_y_z(ZeroT,x_new,TransT[1],TransT[2])


        ZeroT=new_T
        q_new = ur0.best_sol(urc.weights, tempq, new_T)
        tempq=q_new
        new_trans=urc.get_T_translation(ur0.Forward(q_new))
        uree_n_x_pub.publish(new_trans[0])
        uree_n_y_pub.publish(new_trans[1])
        uree_n_z_pub.publish(new_trans[2])
        print "number:->",cn,"q_new",q_new
        urc.urscript_pub(pub, q_new, vel, ace, t)
        cn+=1
        if cn==urc.cont:
            urc.urscript_pub(pub, qzero, 1.05, 1.4, t)
            time.sleep(3)
            cn=0
        # print "cn-----\n",reslut_q[cn]

        rate.sleep()
if __name__ == '__main__':
        main()