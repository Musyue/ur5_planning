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
from sensor_msgs.msg import JointState
class UrLineCircle:
    def __init__(self,weights,radius):
        self.weights = weights
        self.radius=radius#m
        self.vel = 0.1
        self.ace = 50
        self.cont=450
        self.t=0
        self.theta=-math.pi / 4
        self.tempq=[]
        self.wRb = [math.cos(self.theta), -1*math.sin(self.theta), 0, math.sin(self.theta), math.cos(self.theta), 0, 0, 0,1]
        self.uree_world_d_x_pub = rospy.Publisher("/ur_world_frame_ee_d_x", Float64, queue_size=10)
        self.uree_world_d_y_pub = rospy.Publisher("/ur_world_frame_ee_d_y", Float64, queue_size=10)
        self.uree_world_d_z_pub = rospy.Publisher("/ur_world_frame_ee_d_z", Float64, queue_size=10)
        self.uree_world_n_x_pub = rospy.Publisher("/ur_world_frame_ee_n_x", Float64, queue_size=10)
        self.uree_world_n_y_pub = rospy.Publisher("/ur_world_frame_ee_n_y", Float64, queue_size=10)
        self.uree_world_n_z_pub = rospy.Publisher("/ur_world_frame_ee_n_z", Float64, queue_size=10)
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

    def get_IK_from_T(self,T,q_last):
        ur0 = self.get_urobject_ur5kinetmatics()
        return ur0.best_sol(self.weights,q_last,T)
    def get_T_translation(self, T):
        trans_x = T[3]
        trans_y = T[7]
        trans_z = T[11]
        return [trans_x, trans_y, trans_z]
    def insert_new_xy(self,T,nx,ny,nz):
        temp=[]
        for i in xrange(16):
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
    def urscript_pub(self, pub, qq, vel, ace, t):

        ss = "movej([" + str(qq[0]) + "," + str(qq[1]) + "," + str(qq[2]) + "," + str(
            qq[3]) + "," + str(qq[4]) + "," + str(qq[5]) + "]," + "a=" + str(ace) + "," + "v=" + str(
            vel) + "," + "t=" + str(t) + ")"
        print("---------ss:", ss)
        pub.publish(ss)
    def caculate_world_frame(self, T):
        bRe = tr2r(T)
        bte = transl(T)
        wRb = numpy.array(self.wRb).reshape((3, 3))
        homegeneous_T_part3 = numpy.array([0, 0, 0, 1])
        wRbbRe = numpy.dot(wRb, bRe)
        wRbbte = numpy.dot(wRb, bte)
        new_T_T = numpy.column_stack((wRbbRe, wRbbte))
        new_T = np.row_stack((new_T_T, homegeneous_T_part3))
        last_T = self.numpyarray_tolist(new_T)
        inv_wTb_part = numpy.array([0, 0, 0]).T
        inv_wTb_1 = numpy.column_stack((wRb, inv_wTb_part))
        inv_wTb_2 = np.row_stack((inv_wTb_1, homegeneous_T_part3))
        return last_T, inv_wTb_2

    def move_ee_to_desire_point_in_world_frame(self,ur_pub,ur0,Zero_T,world_frame_nx,world_frame_ny,world_frame_nz):
        WTe, inv_wTb = self.caculate_world_frame(numpy.array(Zero_T).reshape((4, 4)))
        new_T_in_world_frame=self.insert_new_xy(WTe,world_frame_nx,world_frame_ny,world_frame_nz)#0.55
        new_T_in_base_frame=numpy.dot(numpy.matrix(inv_wTb).I,numpy.matrix(new_T_in_world_frame).reshape((4,4)))
        new_T_list=self.numpyarray_tolist(new_T_in_base_frame)
        q_new = ur0.best_sol(self.weights, self.tempq, new_T_list)
        print "q_new"
        self.urscript_pub(ur_pub, q_new, self.vel, self.ace, self.t)

    def move_ee_to_left(self,ur5_pub,uro_kinematicx,Qstart,world_frame_nx):
        F_T = uro_kinematicx.Forward(Qstart)
        WTe, inv_wTb = self.caculate_world_frame(numpy.array(F_T).reshape((4, 4)))
        print "WTe",numpy.array(WTe).reshape((4,4))
        new_T_in_world_frame=self.insert_new_xy(WTe,WTe[3]+world_frame_nx,WTe[7],WTe[11])#0.55
        new_T_in_base_frame=numpy.dot(numpy.matrix(inv_wTb).I,numpy.matrix(new_T_in_world_frame).reshape((4,4)))
        new_T_list=self.numpyarray_tolist(new_T_in_base_frame)
        self.uree_world_d_x_pub.publish(WTe[3]+world_frame_nx)
        self.uree_world_d_y_pub.publish(WTe[7])
        self.uree_world_d_z_pub.publish(WTe[11])
        q_new = uro_kinematicx.best_sol(self.weights, self.tempq, new_T_list)
        new_T_inbase_after_caculate=uro_kinematicx.Forward(q_new)
        new_T_in_world_after_caculate,inv_new_T=self.caculate_world_frame(numpy.array(new_T_inbase_after_caculate).reshape((4,4)))
        self.uree_world_n_x_pub.publish(new_T_in_world_after_caculate[3])
        self.uree_world_n_y_pub.publish(new_T_in_world_after_caculate[7])
        self.uree_world_n_z_pub.publish(new_T_in_world_after_caculate[11])
        self.urscript_pub(ur5_pub, q_new, self.vel, self.ace, self.t)
        return q_new
    def move_ee_to_right(self,ur5_pub,uro_kinematicx,Qstart,world_frame_nx):
        F_T = uro_kinematicx.Forward(Qstart)
        WTe, inv_wTb = self.caculate_world_frame(numpy.array(F_T).reshape((4, 4)))
        print "WTe",numpy.array(WTe).reshape((4,4))
        self.uree_world_d_x_pub.publish(WTe[3] - world_frame_nx)
        self.uree_world_d_y_pub.publish(WTe[7])
        self.uree_world_d_z_pub.publish(WTe[11])
        new_T_in_world_frame=self.insert_new_xy(WTe,WTe[3]-world_frame_nx,WTe[7],WTe[11])#0.55
        new_T_in_base_frame=numpy.dot(numpy.matrix(inv_wTb).I,numpy.matrix(new_T_in_world_frame).reshape((4,4)))
        new_T_list=self.numpyarray_tolist(new_T_in_base_frame)
        q_new = uro_kinematicx.best_sol(self.weights, self.tempq, new_T_list)
        new_T_inbase_after_caculate=uro_kinematicx.Forward(q_new)
        new_T_in_world_after_caculate,inv_new_T=self.caculate_world_frame(numpy.array(new_T_inbase_after_caculate).reshape((4,4)))
        self.uree_world_n_x_pub.publish(new_T_in_world_after_caculate[3])
        self.uree_world_n_y_pub.publish(new_T_in_world_after_caculate[7])
        self.uree_world_n_z_pub.publish(new_T_in_world_after_caculate[11])
        self.urscript_pub(ur5_pub, q_new, self.vel, self.ace, self.t)
        return q_new
def main():
    t=0
    ace=50
    qstart=[-45, -180, 90, -180, -90, 180]
    ratet = 25
    radius = 0.1
    weights = [1.] * 6
    urc=UrLineCircle(weights,radius)
    pub=urc.Init_node()
    rate = rospy.Rate(ratet)
    qzero = display(getpi(qstart))
    urk = urc.get_urobject_ur5kinetmatics()
    F_T = urk.Forward(qzero)

    ur_reader = Urposition()
    WTe, inv_wTb = urc.caculate_world_frame(numpy.array(F_T).reshape((4, 4)))
    ur0_kinematics = urc.get_urobject_ur5kinetmatics()
    cn = 1
    ZeroT = F_T
    urc.tempq = qzero
    QTemp = []
    flag_to_zero = 0
    go_back_start_flag = 0
    while not rospy.is_shutdown():
        if flag_to_zero==0:
            cn+=1
            urc.move_ee_to_desire_point_in_world_frame(pub,ur0_kinematics,ZeroT,0.55,WTe[7],WTe[11])
            q_now = ur_reader.ave_ur_pose
            print cn, "Q_now", q_now
            time.sleep(0.1)
            if cn==100:
                if len(q_now) != 0:
                    QTemp = q_now
                    urc.tempq = q_now
                flag_to_zero = 1
                cn=1
        # if flag_to_right==1:
        #     print "first move to right -----"
        #     detax=urc.get_draw_line_x(cn,[-0.45,0,0],[0.45,0,0])
        #     qq=urc.move_ee_to_right(pub,ur0_kinematics,QTemp,detax)
        #     print cn, "move to right -----", qq
        #     cn+=1
        #     time.sleep(0.001)
        #     q_now = ur_reader.ave_ur_pose
        #     if cn==(urc.cont-2):
        #         if len(q_now) != 0:
        #             QTemp = q_now
        #
        #             urc.tempq = q_now
        #         flag_to_right = 0
        #         go_back_start_flag = 1
        #         time.sleep(1)
        #         cn=1
        # if go_back_start_flag==1:
        #     urc.urscript_pub(pub, qzero, 0.2, ace, t)
        #     time.sleep(3)
        #     cn=1
        #     go_back_start_flag=0
        #     print "path planning over ------"
        rate.sleep()
if __name__ == '__main__':
        main()